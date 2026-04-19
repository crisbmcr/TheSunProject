package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.Color
import com.example.sunproject.data.model.FrameRecord
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan
import android.util.Log
object ZenithTopFaceRefiner {
    // ECC translation intentionally disabled for zenith refinement.
// Keep only rotational residual refinement here.
    private const val FACE_SIZE_PX = 512
    private const val ANNULUS_MIN_ALT_DEG = 68f
    private const val ANNULUS_MAX_ALT_DEG = 76f
    private const val ECC_MIN_ALT_DEG = 66f
    private const val ECC_MAX_ALT_DEG = 86f
    private data class PolarPhaseStrip(
        val gray32: Mat,
        val valid32: Mat,
        val angleBins: Int,
        val radialBins: Int
    )

    private const val BLEND_MIN_ALT_DEG = 70f
    private const val POLAR_PHASE_MIN_ALT_DEG = 70f
    private const val POLAR_PHASE_MAX_ALT_DEG = 86f

    private const val POLAR_PHASE_ANGLE_BINS = 720
    private const val POLAR_PHASE_RADIAL_BINS = 96
    private const val MIN_POLAR_PHASE_RESPONSE = 0.03
    private const val MIN_POLAR_PHASE_COVERAGE = 0.12f

    private const val BLEND_FEATHER_START_ALT_DEG = 76f
    private const val BLEND_FEATHER_FULL_ALT_DEG = 86f
    private const val ZENITH_EDGE_FADE_START_ALT_DEG = BLEND_MIN_ALT_DEG
    private const val ZENITH_EDGE_FADE_FULL_ALT_DEG = 82f

    private const val ZENITH_EDGE_MIN_ALPHA = 0.15f

    private const val RGB_GAIN_MIN = 0.92f
    private const val RGB_GAIN_MAX = 1.08f

    private const val POLAR_CAP_FILL_MIN_ALT_DEG = 82f
    private const val POLAR_CAP_NEAREST_RADIUS_PX = 2
    private const val MIN_ECC_ACCEPT_SCORE = 0.12
    private const val STRONG_ECC_ACCEPT_SCORE = 0.20
    private const val MAX_ECC_ROT_IF_WEAK_SCORE_DEG = 3.0f
    private const val MAX_ECC_ROT_IF_STRONG_SCORE_DEG = 6.0f
    private const val COLOR_MATCH_FULL_AT_ALT_DEG = 70f
    private const val COLOR_MATCH_NEUTRAL_AT_ALT_DEG = 84f
    private const val MIN_COLOR_CORRECTION_STRENGTH = 0.35f
    private const val COLOR_GAIN_CLAMP_EPS = 0.002f
    private const val ZENITH_TOPFACE_YAW_OFFSET_DEG = 0f

    private const val MIN_FINAL_BLEND_ECC_SCORE = 0.18
    private const val MAX_FINAL_BLEND_ABS_ROT_DEG = 2.5f

    private const val MAX_FINAL_BLEND_SOFT_ABS_ROT_DEG = 6.5f
    private const val MAX_FINAL_BLEND_SOFT_RESIDUAL_TWIST_DEG = 12f
    private const val MAX_FINAL_BLEND_SOFT_MAX_LUMA_DIFF01 = 0.12f
    private const val MIN_FINAL_BLEND_SOFT_ECC_SCORE = 0.26

    private const val DIRECT_YAW_SEARCH_STEP_DEG = 6f
    private const val DIRECT_YAW_SEARCH_MAX_ABS_DEG = 45f

    private const val POLAR_CAP_COLOR_STRENGTH_SCALE = 0.35f
    private const val POLAR_CAP_MIN_COLOR_STRENGTH = 0.08f
    private const val POLAR_CAP_FILL_WEIGHT_AT_EDGE = 0.70f
    private const val POLAR_CAP_FILL_WEIGHT_AT_POLE = 0.45f

    private const val ZENITH_BLEND_BASE_WEIGHT_REF = 1.25f
    private const val ZENITH_BLEND_MAX_BASE_SUPPRESSION = 0.55f

    private const val DIRECT_SEAM_MIN_ALT_DEG = 70f
    private const val DIRECT_SEAM_MAX_ALT_DEG = 82f
    private const val DIRECT_ANNULUS_ANGLE_BINS = 720

    private const val LOCAL_PHASE_REFINE_MAX_ABS_DEG = 6f
    private const val LOCAL_ECC_REFINE_MAX_ABS_DEG = 6f

    private const val MIN_DIRECT_ANNULUS_CORR = 0.18f
    private const val MIN_DIRECT_ANNULUS_CORR_MARGIN = 0.015f
    private const val MAX_DIRECT_ACCEPT_MEAN_ABS_LUMA_DIFF01 = 0.23f
    private const val MAX_DIRECT_ACCEPT_ABS_YAW_DELTA_DEG = 24f
    private const val MAX_DIRECT_ACCEPT_RESIDUAL_TWIST_DEG = 24f

    // ==================================================================
    // GLOBAL ANNULUS CORRELATION — strategy in use.
    // Replaces the old discrete 17-candidate coarse search ±45° with a
    // single correlation over 360° of the seam band. Resolution ~0.5°.
    // ==================================================================
    private const val GLOBAL_ANNULUS_MIN_ALT_DEG = 70f
    private const val GLOBAL_ANNULUS_MAX_ALT_DEG = 82f
    private const val GLOBAL_ANNULUS_BINS = 720

    // Minimum absolute correlation of the winner for acceptance. Below this
    // we can't trust the estimate: atlas probably too empty in the seam band,
    // or the zenith has no structure in that band.
    private const val MIN_GLOBAL_ANNULUS_CORR = 0.55f

    // Winner must sit at least this much above the median-shift correlation.
    // Measures peak sharpness: big margin = clear peak, tiny = flat landscape.
    private const val MIN_GLOBAL_PEAK_MARGIN_OVER_MEDIAN = 0.12f

    // Sanity gate for large shifts: if the global search moved the yaw by more
    // than this, require an even higher correlation to accept. Catches cases
    // where a spurious correlation in a near-symmetric band could fool us.
    private const val GLOBAL_LARGE_SHIFT_THRESHOLD_DEG = 60f
    private const val GLOBAL_LARGE_SHIFT_MIN_CORR = 0.70f

    // Sign convention for applying the global shift to seedYaw.
    // If after testing the zenith cap rotates in the OPPOSITE direction of
    // what would fix it, flip this to -1f.
    private const val GLOBAL_SHIFT_SIGN = 1f

    // ==================================================================
    // DEAD CODE (pending cleanup in a follow-up patch).
    // Kept to avoid forcing a large deletion in the same commit as the
    // architectural change. These constants are no longer referenced.
    // ==================================================================
    private const val MIN_BROAD_EDGE_ABS_YAW_DELTA_DEG = 24f
    private const val MAX_BROAD_EDGE_ABS_YAW_DELTA_DEG = 45f
    private const val MIN_BROAD_EDGE_ANNULUS_CORR = 0.80f
    private const val MIN_BROAD_EDGE_PEAK_MARGIN = 0.08f
    private const val MAX_BROAD_EDGE_MEAN_ABS_LUMA_DIFF01 = 0.22f
    private const val MIN_BROAD_EDGE_ECC_SCORE = 0.22
    private const val MAX_BROAD_EDGE_ABS_ECC_ROT_DEG = 4.5f
    private const val MAX_BROAD_EDGE_RESIDUAL_TWIST_DEG = 48f

    private const val OBJECTIVE_ANNULUS_CORR_SCALE = 1000f
    private const val OBJECTIVE_DIRECT_LUMA_DIFF_SCALE = 350f
    private const val OBJECTIVE_YAW_PRIOR_SCALE = 3f

    private const val POLAR_CAP_MIN_EXISTING_WEIGHT = 0.18f

    private const val ZENITH_COLOR_CORR_MIN_ALT_DEG = 70f
    private const val ZENITH_COLOR_CORR_MAX_ALT_DEG = 80f
    private const val ZENITH_COLOR_CORR_MIN_BASE_WEIGHT = 0.05f
    private const val MAX_FINAL_BLEND_RESIDUAL_TWIST_DEG = 75f
    private const val MAX_FINAL_BLEND_HARD_RESIDUAL_TWIST_DEG = 60f

    private const val OBJECTIVE_ECC_SCALE = 1000f
    private const val OBJECTIVE_OVERLAP_SCALE = 0.02f
    private const val OBJECTIVE_LUMA_DIFF_SCALE = 400f
    private const val OBJECTIVE_RESIDUAL_TWIST_SCALE = 4f

    private const val LOCAL_YAW_PRIOR_STEP_DEG = 12f
    private const val LOCAL_YAW_PRIOR_MAX_ABS_DEG = 36f
    private const val MAX_PHASE_RESIDUAL_YAW_DEG = 24f
    // Usamos luma normalizada [0..1] para que la penalización no opaque por completo al ECC.
    private const val DEFAULT_REJECT_MEAN_ABS_LUMA_DIFF01 = 1f
    private data class ZenithRefinementCandidate(
        val label: String,
        val priorOffsetDeg: Float,
        val twistDeg: Float,
        val pitchOffsetDeg: Float,
        val rollOffsetDeg: Float,
        val absoluteYawDeg: Float?,
        val absolutePitchDeg: Float?,
        val absoluteRollDeg: Float?
    )

    private data class CandidateEvaluation(
        val candidate: ZenithRefinementCandidate,
        val result: RefinementResult,
        val seamScore: CandidateSeamScore,
        val residualTwistAbsDeg: Float,
        val acceptedForBlend: Boolean,
        val objectiveScore: Float
    )
    data class TopFace(
        val rgba: Mat,
        val gray32: Mat,
        val validMask: Mat,
        val faceSizePx: Int
    )

    data class RefinementResult(
        val initialTwistDeg: Float,
        val finalTwistDeg: Float,
        val eccRotationDeg: Float,
        val eccTxPx: Float,
        val eccTyPx: Float,
        val eccScore: Double,
        val alignedTopFace: TopFace
    )
    private data class ColorCorrection(
        val rGain: Float,
        val gGain: Float,
        val bGain: Float,
        val overlapCount: Int
    )

    private data class TopFaceSample(
        val color: Int,
        val usedFallback: Boolean
    )
    private data class DirectYawCandidate(
        val label: String,
        val yawDeltaDeg: Float,
        val twistDeg: Float,
        val absoluteYawDeg: Float?
    )

    private data class CandidateSeamScore(
        val overlapCount: Int,
        val meanAbsLumaDiff01: Float,
        val annulusCorr: Float
    )

    private data class DirectYawCandidateEvaluation(
        val candidate: DirectYawCandidate,
        val seamScore: CandidateSeamScore,
        val objectiveScore: Float
    )

    private fun buildDirectYawCandidates(
        seedTwistDeg: Float,
        seedAbsoluteYawDeg: Float?
    ): List<DirectYawCandidate> {
        // Coarse uniforme cada 6° en [-45, +45]. Sin saltos en los bordes (antes
        // el paso se abria a 6° entre 36 y 45), asi el ganador real no se aproxima
        // con un offset grueso que el refiner local (±6° de phase + ±6° de ECC)
        // no llega a cerrar. Esto importa especialmente para el modo broad_edge:
        // si el coarse aterriza limpio, eccRot baja y el candidato pasa el filtro
        // sin tocar umbrales.
        val deltas = listOf(
            0f,
            -DIRECT_YAW_SEARCH_STEP_DEG, DIRECT_YAW_SEARCH_STEP_DEG,
            -12f, 12f,
            -18f, 18f,
            -24f, 24f,
            -30f, 30f,
            -36f, 36f,
            -42f, 42f,
            -DIRECT_YAW_SEARCH_MAX_ABS_DEG, DIRECT_YAW_SEARCH_MAX_ABS_DEG
        )

        fun labelFor(deltaDeg: Float): String {
            return when {
                deltaDeg > 0f -> "yaw_p${deltaDeg.toInt()}"
                deltaDeg < 0f -> "yaw_m${abs(deltaDeg).toInt()}"
                else -> "yaw_0"
            }
        }

        return deltas.map { deltaDeg ->
            DirectYawCandidate(
                label = labelFor(deltaDeg),
                yawDeltaDeg = deltaDeg,
                twistDeg = normalizeDeg(seedTwistDeg + deltaDeg),
                absoluteYawDeg = seedAbsoluteYawDeg?.let { normalizeDeg(it + deltaDeg) }
            )
        }
    }

    private fun maxResidualTwistForBlend(
        frame: FrameRecord,
        seedAbsolutePitchDeg: Float?
    ): Float {
        val hardZenith =
            (seedAbsolutePitchDeg ?: Float.NEGATIVE_INFINITY) >= 87f ||
                    frame.targetPitchDeg >= 88f ||
                    frame.ringId.equals("Z0", ignoreCase = true)

        return if (hardZenith) {
            MAX_FINAL_BLEND_HARD_RESIDUAL_TWIST_DEG
        } else {
            MAX_FINAL_BLEND_RESIDUAL_TWIST_DEG
        }
    }

    private fun clampSignedResidualYawDeg(
        valueDeg: Float,
        maxAbsDeg: Float
    ): Float {
        return normalizeSignedDeg(valueDeg).coerceIn(-maxAbsDeg, maxAbsDeg)
    }

    private fun zeroShiftNormalizedCorrelation(
        a: FloatArray,
        b: FloatArray
    ): Float {
        var num = 0.0
        var denA = 0.0
        var denB = 0.0

        val n = minOf(a.size, b.size)
        for (i in 0 until n) {
            val av = a[i].toDouble()
            val bv = b[i].toDouble()
            num += av * bv
            denA += av * av
            denB += bv * bv
        }

        if (denA <= 1e-9 || denB <= 1e-9) return -1f
        return (num / sqrt(denA * denB)).toFloat()
    }

    private fun annulusZeroShiftCorrelation(
        reference: TopFace,
        moving: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int = DIRECT_ANNULUS_ANGLE_BINS
    ): Float {
        val refSig = buildAnnulusSignature(reference, minAltitudeDeg, maxAltitudeDeg, angleBins)
        val movSig = buildAnnulusSignature(moving, minAltitudeDeg, maxAltitudeDeg, angleBins)
        return zeroShiftNormalizedCorrelation(refSig, movSig)
    }

    private fun directYawObjectiveScore(
        seamScore: CandidateSeamScore,
        yawDeltaDeg: Float
    ): Float {
        return seamScore.annulusCorr * OBJECTIVE_ANNULUS_CORR_SCALE -
                seamScore.meanAbsLumaDiff01 * OBJECTIVE_DIRECT_LUMA_DIFF_SCALE -
                abs(yawDeltaDeg) * OBJECTIVE_YAW_PRIOR_SCALE
    }

    private fun acceptanceMode(
        result: RefinementResult,
        seamScore: CandidateSeamScore,
        residualTwistAbsDeg: Float,
        yawDeltaDeg: Float,
        peakCorrMargin: Float,
        maxResidualTwistDeg: Float
    ): String {
        // Modo 1: strict. El refiner cerro una solucion "chica" casi sobre el seed
        // y el ECC esta muy contento. Camino normal cuando el seed IMU es bueno.
        val strictAccept =
            result.eccScore >= MIN_FINAL_BLEND_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_FINAL_BLEND_ABS_ROT_DEG &&
                    residualTwistAbsDeg <= maxResidualTwistDeg

        if (strictAccept) return "strict"

        // Modo 2: direct_seam. El yaw esta dentro de ±24° del seed y hay correlacion
        // en la banda de seam. Rescata casos donde el ECC no convergio pero el
        // annulus muestra senal clara.
        val directSeamAccept =
            seamScore.annulusCorr >= MIN_DIRECT_ANNULUS_CORR &&
                    peakCorrMargin >= MIN_DIRECT_ANNULUS_CORR_MARGIN &&
                    seamScore.meanAbsLumaDiff01 <= MAX_DIRECT_ACCEPT_MEAN_ABS_LUMA_DIFF01 &&
                    abs(yawDeltaDeg) <= MAX_DIRECT_ACCEPT_ABS_YAW_DELTA_DEG &&
                    residualTwistAbsDeg <= minOf(
                maxResidualTwistDeg,
                MAX_DIRECT_ACCEPT_RESIDUAL_TWIST_DEG
            ) &&
                    abs(result.eccRotationDeg) <= LOCAL_ECC_REFINE_MAX_ABS_DEG

        if (directSeamAccept) return "direct_seam"

        // Modo 3: broad_edge. El coarse encontro un ganador lejos del seed
        // (|yawDelta| > 24°, hasta ±45°). Lo aceptamos solo si TODAS las metricas
        // de evidencia son fuertes: annulus alto, margen sobre el runner-up claro,
        // luma consistente, ECC score alto y eccRot pequeno (el refiner no esta
        // forzando la solucion al tope). Si el coarse cayo en el borde por yaw
        // genuino del frame, todos estos umbrales pasan comodos.
        val broadEdgeAccept =
            abs(yawDeltaDeg) > MIN_BROAD_EDGE_ABS_YAW_DELTA_DEG &&
                    abs(yawDeltaDeg) <= MAX_BROAD_EDGE_ABS_YAW_DELTA_DEG &&
                    seamScore.annulusCorr >= MIN_BROAD_EDGE_ANNULUS_CORR &&
                    peakCorrMargin >= MIN_BROAD_EDGE_PEAK_MARGIN &&
                    seamScore.meanAbsLumaDiff01 <= MAX_BROAD_EDGE_MEAN_ABS_LUMA_DIFF01 &&
                    result.eccScore >= MIN_BROAD_EDGE_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_BROAD_EDGE_ABS_ECC_ROT_DEG &&
                    residualTwistAbsDeg <= minOf(
                maxResidualTwistDeg,
                MAX_BROAD_EDGE_RESIDUAL_TWIST_DEG
            )

        return if (broadEdgeAccept) "broad_edge" else "rejected"
    }

    private fun isAcceptableFinalBlend(
        result: RefinementResult,
        seamScore: CandidateSeamScore,
        residualTwistAbsDeg: Float,
        maxResidualTwistDeg: Float
    ): Boolean {
        val strictAccept =
            result.eccScore >= MIN_FINAL_BLEND_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_FINAL_BLEND_ABS_ROT_DEG &&
                    residualTwistAbsDeg <= maxResidualTwistDeg

        val seamRescueAccept =
            result.eccScore >= MIN_FINAL_BLEND_SOFT_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_FINAL_BLEND_SOFT_ABS_ROT_DEG &&
                    residualTwistAbsDeg <= minOf(
                maxResidualTwistDeg,
                MAX_FINAL_BLEND_SOFT_RESIDUAL_TWIST_DEG
            ) &&
                    seamScore.meanAbsLumaDiff01 <= MAX_FINAL_BLEND_SOFT_MAX_LUMA_DIFF01

        return strictAccept || seamRescueAccept
    }

    private fun refinementObjectiveScore(
        result: RefinementResult,
        seamScore: CandidateSeamScore,
        residualTwistAbsDeg: Float
    ): Float {
        return result.eccScore.toFloat() * OBJECTIVE_ECC_SCALE -
                seamScore.meanAbsLumaDiff01 * OBJECTIVE_LUMA_DIFF_SCALE -
                residualTwistAbsDeg * OBJECTIVE_RESIDUAL_TWIST_SCALE
    }

    private fun releaseCandidateEvaluation(eval: CandidateEvaluation?) {
        if (eval == null) return
        releaseTopFace(eval.result.alignedTopFace)
    }
    private fun evaluateCandidateSeamScore(
        aligned: TopFace,
        atlas: SkyAtlas,
        atlasTop: TopFace,
        minAltitudeDeg: Float = DIRECT_SEAM_MIN_ALT_DEG,
        maxAltitudeDeg: Float = DIRECT_SEAM_MAX_ALT_DEG
    ): CandidateSeamScore {
        val facePixels = rgbaMatToArgb(aligned.rgba)

        val yA = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)
        val yB = AtlasMath.altitudeToY(maxAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        val yStart = minOf(yA, yB)
        val yEnd = maxOf(yA, yB)

        var overlapCount = 0
        var lumaDiffSum01 = 0f

        for (y in yStart..yEnd) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg || altDeg > maxAltitudeDeg) continue

            for (x in 0 until atlas.width) {
                if (atlas.weightAt(x, y) < ZENITH_COLOR_CORR_MIN_BASE_WEIGHT) continue

                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceWithFallback(
                    face = aligned,
                    facePixels = facePixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                ) ?: continue

                val atlasColor = atlas.pixels[atlas.index(x, y)]

                lumaDiffSum01 += abs(colorLuma01(atlasColor) - colorLuma01(sampled.color))
                overlapCount++
            }
        }

        val annulusCorr = annulusZeroShiftCorrelation(
            reference = atlasTop,
            moving = aligned,
            minAltitudeDeg = minAltitudeDeg,
            maxAltitudeDeg = maxAltitudeDeg,
            angleBins = DIRECT_ANNULUS_ANGLE_BINS
        )

        return CandidateSeamScore(
            overlapCount = overlapCount,
            meanAbsLumaDiff01 = if (overlapCount > 0) {
                lumaDiffSum01 / overlapCount.toFloat()
            } else {
                1f
            },
            annulusCorr = annulusCorr
        )
    }

    private fun colorLuma01(color: Int): Float {
        return (
                0.299f * Color.red(color) +
                        0.587f * Color.green(color) +
                        0.114f * Color.blue(color)
                ) / 255f
    }

    private fun shortestAngleAbsDeg(aDeg: Float, bDeg: Float): Float {
        var delta = normalizeDeg(aDeg - bDeg)
        if (delta > 180f) delta -= 360f
        return abs(delta)
    }
    fun refineAndBlendZenithIntoAtlas(
        atlas: SkyAtlas,
        frame: FrameRecord,
        srcBitmap: Bitmap,
        frameWeight: Float,
        seedTwistDeg: Float,
        seedPitchOffsetDeg: Float,
        seedRollOffsetDeg: Float,
        seedAbsoluteYawDeg: Float? = null,
        seedAbsolutePitchDeg: Float? = null,
        seedAbsoluteRollDeg: Float? = null
    ): RefinementResult? {
        com.example.sunproject.SunProjectApp.requireOpenCv()

        if (frameWeight <= 0f) return null

        val atlasTop = buildAtlasTopFace(atlas, FACE_SIZE_PX)
        val baseSeedTwistDeg = normalizeDeg(seedTwistDeg)
        val maxResidualTwistDeg = maxResidualTwistForBlend(
            frame = frame,
            seedAbsolutePitchDeg = seedAbsolutePitchDeg
        )

        try {
            // ===============================================================
            // STEP 1 — Build the "seed" topFace with the IMU pose as-is.
            // This topFace is only used to measure the global annulus shift;
            // it is NOT the one we blend into the atlas.
            // ===============================================================
            val seedTop = buildZenithTopFace(
                frame = frame,
                srcBitmap = srcBitmap,
                baseTwistDeg = baseSeedTwistDeg,
                basePitchOffsetDeg = seedPitchOffsetDeg,
                baseRollOffsetDeg = seedRollOffsetDeg,
                absoluteYawDegOverride = seedAbsoluteYawDeg,
                absolutePitchDegOverride = seedAbsolutePitchDeg,
                absoluteRollDegOverride = seedAbsoluteRollDeg,
                faceSizePx = FACE_SIZE_PX
            )

            // ===============================================================
            // STEP 2 — Global annulus correlation (360°, resolution ~0.5°).
            // Replaces the old 17-candidate coarse search ±45°.
            // ===============================================================
            val globalResult = try {
                annulusGlobalArgmax(
                    reference = atlasTop,
                    moving = seedTop,
                    minAltitudeDeg = GLOBAL_ANNULUS_MIN_ALT_DEG,
                    maxAltitudeDeg = GLOBAL_ANNULUS_MAX_ALT_DEG,
                    angleBins = GLOBAL_ANNULUS_BINS
                )
            } finally {
                releaseTopFace(seedTop)
            }

            val rawShiftDeg = globalResult.bestShiftDeg
            // Convert [0, 360) to signed (-180, 180]. Pick the shorter rotation.
            val signedShiftDeg = normalizeSignedDeg(rawShiftDeg) * GLOBAL_SHIFT_SIGN
            val peakMarginOverMedian = globalResult.bestCorr - globalResult.corrMedian
            val peakMarginOverSeed = globalResult.bestCorr - globalResult.corrAtZero

            Log.d(
                "AtlasZenithGlobalSearch",
                "frame=${frame.frameId} " +
                        "seedYaw=${seedAbsoluteYawDeg?.let { "%.2f".format(it) } ?: "NaN"} " +
                        "baseSeedTwist=${"%.2f".format(baseSeedTwistDeg)} " +
                        "rawShift=${"%.2f".format(rawShiftDeg)} " +
                        "signedShift=${"%.2f".format(signedShiftDeg)} " +
                        "globalCorr=${"%.5f".format(globalResult.bestCorr)} " +
                        "corrAtZero=${"%.5f".format(globalResult.corrAtZero)} " +
                        "corrMedian=${"%.5f".format(globalResult.corrMedian)} " +
                        "marginVsMedian=${"%.5f".format(peakMarginOverMedian)} " +
                        "marginVsSeed=${"%.5f".format(peakMarginOverSeed)}"
            )

            // ===============================================================
            // STEP 3 — Build the corrected topFace using the shifted yaw.
            // This one IS blended into the atlas.
            // ===============================================================
            val correctedYawDeg = seedAbsoluteYawDeg?.let { normalizeDeg(it + signedShiftDeg) }
            val correctedBaseTwistDeg = normalizeDeg(baseSeedTwistDeg + signedShiftDeg)

            val correctedTop = buildZenithTopFace(
                frame = frame,
                srcBitmap = srcBitmap,
                baseTwistDeg = correctedBaseTwistDeg,
                basePitchOffsetDeg = seedPitchOffsetDeg,
                baseRollOffsetDeg = seedRollOffsetDeg,
                absoluteYawDegOverride = correctedYawDeg,
                absolutePitchDegOverride = seedAbsolutePitchDeg,
                absoluteRollDegOverride = seedAbsoluteRollDeg,
                faceSizePx = FACE_SIZE_PX
            )

            try {
                // ===========================================================
                // STEP 4 — Local residual refinement (phase + ECC).
                // The global search should have gotten us within 1°. These are
                // sub-degree corrections. ECC can fail and that's OK now.
                // ===========================================================
                val rawPhaseResidualDeg = estimateTwistWithPolarPhaseCorrelation(
                    reference = atlasTop,
                    moving = correctedTop,
                    minAltitudeDeg = DIRECT_SEAM_MIN_ALT_DEG,
                    maxAltitudeDeg = DIRECT_SEAM_MAX_ALT_DEG,
                    coarseTwistDeg = 0f
                ) ?: 0f

                val phaseResidualDeg = clampSignedResidualYawDeg(
                    rawPhaseResidualDeg,
                    LOCAL_PHASE_REFINE_MAX_ABS_DEG
                )

                val residualRefined = refineWithEcc(
                    reference = atlasTop,
                    moving = correctedTop,
                    initialTwistDeg = phaseResidualDeg,
                    eccMinAltitudeDeg = DIRECT_SEAM_MIN_ALT_DEG,
                    eccMaxAltitudeDeg = DIRECT_SEAM_MAX_ALT_DEG
                )

                val absoluteInitialTwistDeg = normalizeDeg(
                    baseSeedTwistDeg + signedShiftDeg + phaseResidualDeg
                )
                val absoluteFinalTwistDeg = normalizeDeg(
                    baseSeedTwistDeg + signedShiftDeg + residualRefined.finalTwistDeg
                )

                val candidateResult = RefinementResult(
                    initialTwistDeg = absoluteInitialTwistDeg,
                    finalTwistDeg = absoluteFinalTwistDeg,
                    eccRotationDeg = residualRefined.eccRotationDeg,
                    eccTxPx = residualRefined.eccTxPx,
                    eccTyPx = residualRefined.eccTyPx,
                    eccScore = residualRefined.eccScore,
                    alignedTopFace = residualRefined.alignedTopFace
                )

                val finalSeamScore = evaluateCandidateSeamScore(
                    aligned = candidateResult.alignedTopFace,
                    atlas = atlas,
                    atlasTop = atlasTop,
                    minAltitudeDeg = DIRECT_SEAM_MIN_ALT_DEG,
                    maxAltitudeDeg = DIRECT_SEAM_MAX_ALT_DEG
                )

                val residualTwistAbsDeg = shortestAngleAbsDeg(
                    candidateResult.finalTwistDeg,
                    baseSeedTwistDeg
                )

                // ===========================================================
                // STEP 5 — Acceptance based on the global correlation quality.
                // No longer requires ECC convergence.
                // ===========================================================
                val acceptMode = globalAcceptanceMode(
                    finalSeamScore = finalSeamScore,
                    globalBestCorr = globalResult.bestCorr,
                    globalPeakMarginOverMedian = peakMarginOverMedian,
                    signedGlobalShiftDeg = signedShiftDeg,
                    residualTwistAbsDeg = residualTwistAbsDeg,
                    maxResidualTwistDeg = maxResidualTwistDeg
                )
                val acceptedForBlend = acceptMode != "rejected"

                Log.d(
                    "AtlasZenithTopCandidate",
                    "frame=${frame.frameId} " +
                            "globalShift=${"%.2f".format(signedShiftDeg)} " +
                            "globalCorr=${"%.5f".format(globalResult.bestCorr)} " +
                            "marginVsMedian=${"%.5f".format(peakMarginOverMedian)} " +
                            "marginVsSeed=${"%.5f".format(peakMarginOverSeed)} " +
                            "phaseResidual=${"%.2f".format(phaseResidualDeg)} " +
                            "initialTwist=${"%.2f".format(candidateResult.initialTwistDeg)} " +
                            "finalTwist=${"%.2f".format(candidateResult.finalTwistDeg)} " +
                            "residualTwist=${"%.2f".format(residualTwistAbsDeg)} " +
                            "maxResidualTwist=${"%.2f".format(maxResidualTwistDeg)} " +
                            "eccRot=${"%.2f".format(candidateResult.eccRotationDeg)} " +
                            "eccScore=${"%.5f".format(candidateResult.eccScore)} " +
                            "finalAnnulusCorr=${"%.5f".format(finalSeamScore.annulusCorr)} " +
                            "overlap=${finalSeamScore.overlapCount} " +
                            "meanAbsLumaDiff01=${"%.5f".format(finalSeamScore.meanAbsLumaDiff01)} " +
                            "accepted=$acceptedForBlend " +
                            "acceptMode=$acceptMode"
                )

                if (!acceptedForBlend) {
                    Log.w(
                        "AtlasZenithTopRefine",
                        "rejectAll frame=${frame.frameId} " +
                                "globalShift=${"%.2f".format(signedShiftDeg)} " +
                                "globalCorr=${"%.5f".format(globalResult.bestCorr)} " +
                                "marginVsMedian=${"%.5f".format(peakMarginOverMedian)} " +
                                "residualTwist=${"%.2f".format(residualTwistAbsDeg)} " +
                                "acceptMode=$acceptMode"
                    )
                    releaseTopFace(candidateResult.alignedTopFace)
                    return null
                }

                val colorCorrection = blendTopFaceIntoAtlas(
                    aligned = candidateResult.alignedTopFace,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    minAltitudeDeg = BLEND_MIN_ALT_DEG
                )

                fillUncoveredPolarCapFromTopFace(
                    aligned = candidateResult.alignedTopFace,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    minAltitudeDeg = POLAR_CAP_FILL_MIN_ALT_DEG,
                    colorCorrection = colorCorrection
                )

                Log.d(
                    "AtlasZenithTopRefine",
                    "frame=${frame.frameId} " +
                            "globalShift=${"%.2f".format(signedShiftDeg)} " +
                            "globalCorr=${"%.5f".format(globalResult.bestCorr)} " +
                            "marginVsMedian=${"%.5f".format(peakMarginOverMedian)} " +
                            "phaseResidual=${"%.2f".format(phaseResidualDeg)} " +
                            "initialTwist=${"%.2f".format(candidateResult.initialTwistDeg)} " +
                            "finalTwist=${"%.2f".format(candidateResult.finalTwistDeg)} " +
                            "residualTwist=${"%.2f".format(residualTwistAbsDeg)} " +
                            "eccRot=${"%.2f".format(candidateResult.eccRotationDeg)} " +
                            "eccScore=${"%.5f".format(candidateResult.eccScore)} " +
                            "finalAnnulusCorr=${"%.5f".format(finalSeamScore.annulusCorr)} " +
                            "meanAbsLumaDiff01=${"%.5f".format(finalSeamScore.meanAbsLumaDiff01)} " +
                            "acceptMode=$acceptMode"
                )

                return candidateResult
            } finally {
                releaseTopFace(correctedTop)
            }
        } finally {
            releaseTopFace(atlasTop)
        }
    }


    fun buildAtlasTopFace(
        atlas: SkyAtlas,
        faceSizePx: Int = FACE_SIZE_PX
    ): TopFace {
        val argb = IntArray(faceSizePx * faceSizePx)
        val mask = ByteArray(faceSizePx * faceSizePx)

        val center = (faceSizePx - 1) * 0.5f
        val radius = center

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dir = topFaceDirection(x, y, center, radius)
                val azDeg = normalizeDeg(radToDeg(atan2(dir[0], dir[1])))
                val altDeg = radToDeg(atan2(dir[2], sqrt(dir[0] * dir[0] + dir[1] * dir[1])))

                val atlasX = AtlasMath.azimuthToX(azDeg, atlas.config).coerceIn(0, atlas.width - 1)
                val atlasY = AtlasMath.altitudeToY(altDeg, atlas.config).coerceIn(0, atlas.height - 1)

                val idx = y * faceSizePx + x
                if (atlas.hasCoverageAt(atlasX, atlasY)) {
                    argb[idx] = atlas.pixels[atlas.index(atlasX, atlasY)]
                    mask[idx] = 0xFF.toByte()
                } else {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                }
            }
        }

        return topFaceFromArgbAndMask(argb, mask, faceSizePx)
    }

    fun buildZenithTopFace(
        frame: FrameRecord,
        srcBitmap: Bitmap,
        baseTwistDeg: Float,
        basePitchOffsetDeg: Float,
        baseRollOffsetDeg: Float,
        absoluteYawDegOverride: Float? = null,
        absolutePitchDegOverride: Float? = null,
        absoluteRollDegOverride: Float? = null,
        faceSizePx: Int = FACE_SIZE_PX
    ): TopFace {
        val srcW = srcBitmap.width
        val srcH = srcBitmap.height
        val srcPixels = IntArray(srcW * srcH)
        srcBitmap.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov * 0.5).toFloat())
        val tanHalfV = tan(Math.toRadians(vfov * 0.5).toFloat())

        val hasAbsoluteSeed =
            absoluteYawDegOverride != null &&
                    absolutePitchDegOverride != null &&
                    absoluteRollDegOverride != null

        val rawAbsoluteYawDeg = if (hasAbsoluteSeed) {
            absoluteYawDegOverride!!
        } else {
            Float.NaN
        }

        val yawDeg = if (hasAbsoluteSeed) {
            normalizeDeg(rawAbsoluteYawDeg + ZENITH_TOPFACE_YAW_OFFSET_DEG)
        } else {
            normalizeDeg(frame.measuredAzimuthDeg + baseTwistDeg)
        }

        val pitchDeg = if (hasAbsoluteSeed) {
            absolutePitchDegOverride!!.coerceIn(84f, 90f)
        } else {
            (frame.measuredPitchDeg + basePitchOffsetDeg).coerceIn(84f, 90f)
        }

        val rollDeg = if (hasAbsoluteSeed) {
            absoluteRollDegOverride!!
        } else {
            frame.measuredRollDeg + baseRollOffsetDeg
        }

        Log.d(
            "AtlasZenithSeed",
            "frame=${frame.frameId} " +
                    "source=${if (hasAbsoluteSeed) "absolute" else "legacy"} " +
                    "rawAbsoluteYaw=${"%.2f".format(rawAbsoluteYawDeg)} " +
                    "yawTopFaceUsed=${"%.2f".format(yawDeg)} " +
                    "pitchUsed=${"%.2f".format(pitchDeg)} " +
                    "rollUsed=${"%.2f".format(rollDeg)} " +
                    "yawOffset=${"%.2f".format(ZENITH_TOPFACE_YAW_OFFSET_DEG)}"
        )

        val yawRad = degToRad(yawDeg)
        val pitchRad = degToRad(pitchDeg)
        val rollRad = degToRad(-rollDeg)
        val forward = worldDirectionRad(yawRad, pitchRad)

        val useHardZenithBasis =
            frame.ringId.equals("Z0", ignoreCase = true) ||
                    frame.targetPitchDeg >= 88f ||
                    pitchDeg >= 89.5f

        val right0: FloatArray
        val up0: FloatArray

        if (useHardZenithBasis) {
            right0 = normalize(
                floatArrayOf(
                    cos(yawRad),
                    sin(yawRad),
                    0f
                )
            )
            up0 = normalize(
                floatArrayOf(
                    -sin(yawRad),
                    cos(yawRad),
                    0f
                )
            )
        } else {
            var r = cross(forward, floatArrayOf(0f, 0f, 1f))
            if (length(r) < 1e-4f) {
                r = floatArrayOf(1f, 0f, 0f)
            }
            r = normalize(r)
            val u = normalize(cross(r, forward))
            right0 = r
            up0 = u
        }

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

        val argb = IntArray(faceSizePx * faceSizePx)
        val mask = ByteArray(faceSizePx * faceSizePx)

        val center = (faceSizePx - 1) * 0.5f
        val radius = center

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dir = topFaceDirection(x, y, center, radius)

                val camX = dot(dir, right)
                val camY = dot(dir, up)
                val camZ = dot(dir, forward)

                val idx = y * faceSizePx + x
                if (camZ <= 0f) {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                    continue
                }

                val nx = (camX / camZ) / tanHalfH
                val ny = (camY / camZ) / tanHalfV

                if (abs(nx) > 1f || abs(ny) > 1f) {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                    continue
                }

                val u = (((nx + 1f) * 0.5f) * (srcW - 1)).roundToInt().coerceIn(0, srcW - 1)
                val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).roundToInt().coerceIn(0, srcH - 1)

                argb[idx] = srcPixels[v * srcW + u]
                mask[idx] = 0xFF.toByte()
            }
        }

        return topFaceFromArgbAndMask(argb, mask, faceSizePx)
    }
    // Result of the global 360° annulus argmax.
    // bestShiftDeg is in [0, 360). Convert to signed [-180, 180] at the caller
    // via normalizeSignedDeg if needed.
    private data class AnnulusGlobalResult(
        val bestShiftDeg: Float,
        val bestCorr: Float,
        val corrAtZero: Float,   // correlation at shift=0 (= stay at IMU seed)
        val corrMedian: Float    // median over all shifts (= random-chance baseline)
    )

    // Global argmax of the annulus correlation over 360°.
    // Unlike estimateTwistFromAnnulus (which returns only the best shift),
    // this also returns the correlation at shift=0 and the median across all
    // shifts, so the caller can reason about "how much better than seed" and
    // "how sharp is the peak".
    //
    // Cost: O(angleBins^2). With angleBins=720 that's ~0.5M float mult/adds,
    // which runs in a few milliseconds on a modern phone.
    private fun annulusGlobalArgmax(
        reference: TopFace,
        moving: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int
    ): AnnulusGlobalResult {
        val refSig = buildAnnulusSignature(reference, minAltitudeDeg, maxAltitudeDeg, angleBins)
        val movSig = buildAnnulusSignature(moving, minAltitudeDeg, maxAltitudeDeg, angleBins)

        var denA = 0.0
        var denB = 0.0
        for (i in 0 until angleBins) {
            denA += refSig[i] * refSig[i]
            denB += movSig[i] * movSig[i]
        }
        val denProd = sqrt(denA * denB).coerceAtLeast(1e-9)

        val allCorr = FloatArray(angleBins)
        var bestShift = 0
        var bestScore = Float.NEGATIVE_INFINITY

        for (shift in 0 until angleBins) {
            var num = 0.0
            for (i in 0 until angleBins) {
                num += refSig[i] * movSig[(i + shift) % angleBins]
            }
            val score = (num / denProd).toFloat()
            allCorr[shift] = score

            if (score > bestScore) {
                bestScore = score
                bestShift = shift
            }
        }

        val bestShiftDeg = bestShift.toFloat() * (360f / angleBins.toFloat())

        val sortedCorr = allCorr.sortedArray()
        val corrMedian = sortedCorr[angleBins / 2]

        return AnnulusGlobalResult(
            bestShiftDeg = bestShiftDeg,
            bestCorr = bestScore,
            corrAtZero = allCorr[0],
            corrMedian = corrMedian
        )
    }
    fun estimateTwistFromAnnulus(
        reference: TopFace,
        moving: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int = 360
    ): Float {
        val refSig = buildAnnulusSignature(reference, minAltitudeDeg, maxAltitudeDeg, angleBins)
        val movSig = buildAnnulusSignature(moving, minAltitudeDeg, maxAltitudeDeg, angleBins)

        var bestShift = 0
        var bestScore = Float.NEGATIVE_INFINITY

        for (shift in 0 until angleBins) {
            var num = 0.0
            var denA = 0.0
            var denB = 0.0

            for (i in 0 until angleBins) {
                val a = refSig[i]
                val b = movSig[(i + shift) % angleBins]
                num += a * b
                denA += a * a
                denB += b * b
            }

            val score =
                if (denA > 1e-9 && denB > 1e-9) {
                    (num / sqrt(denA * denB)).toFloat()
                } else {
                    Float.NEGATIVE_INFINITY
                }

            if (score > bestScore) {
                bestScore = score
                bestShift = shift
            }
        }

        return normalizeDeg(bestShift.toFloat())
    }

    fun refineWithEcc(
        reference: TopFace,
        moving: TopFace,
        initialTwistDeg: Float,
        eccMinAltitudeDeg: Float,
        eccMaxAltitudeDeg: Float
    ): RefinementResult {
        val center = Point((moving.faceSizePx - 1) * 0.5, (moving.faceSizePx - 1) * 0.5)

        val rotM = Imgproc.getRotationMatrix2D(center, initialTwistDeg.toDouble(), 1.0)

        val rotatedRgba = Mat()
        val rotatedMask = Mat()
        val rotatedGray = Mat()

        Imgproc.warpAffine(
            moving.rgba,
            rotatedRgba,
            rotM,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_LINEAR,
            Core.BORDER_CONSTANT,
            Scalar(0.0, 0.0, 0.0, 0.0)
        )

        Imgproc.warpAffine(
            moving.validMask,
            rotatedMask,
            rotM,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_NEAREST,
            Core.BORDER_CONSTANT,
            Scalar(0.0)
        )

        Imgproc.cvtColor(rotatedRgba, rotatedGray, Imgproc.COLOR_RGBA2GRAY)
        rotatedGray.convertTo(rotatedGray, CvType.CV_32F, 1.0 / 255.0)

        val altitudeMask = buildAltitudeMask(
            faceSizePx = moving.faceSizePx,
            minAltitudeDeg = eccMinAltitudeDeg,
            maxAltitudeDeg = eccMaxAltitudeDeg
        )

        val eccMask = Mat()
        Core.bitwise_and(reference.validMask, rotatedMask, eccMask)
        Core.bitwise_and(eccMask, altitudeMask, eccMask)

        val warp = Mat.eye(2, 3, CvType.CV_32F)
        val criteria = TermCriteria(
            TermCriteria.COUNT + TermCriteria.EPS,
            50,
            1e-4
        )

        val eccScore = try {
            Video.findTransformECC(
                reference.gray32,
                rotatedGray,
                warp,
                Video.MOTION_EUCLIDEAN,
                criteria,
                eccMask,
                5
            )
        } catch (_: Throwable) {
            0.0
        }

        val m00 = warp.get(0, 0)[0]
        val m01 = warp.get(0, 1)[0]

        val tx = warp.get(0, 2)[0].toFloat()
        val ty = warp.get(1, 2)[0].toFloat()

        val eccRotRawDeg = Math.toDegrees(atan2(m01, m00).toDouble()).toFloat()

        val acceptEccRotation =
            eccScore >= MIN_ECC_ACCEPT_SCORE &&
                    (
                            (eccScore >= STRONG_ECC_ACCEPT_SCORE &&
                                    abs(eccRotRawDeg) <= MAX_ECC_ROT_IF_STRONG_SCORE_DEG) ||
                                    (eccScore < STRONG_ECC_ACCEPT_SCORE &&
                                            abs(eccRotRawDeg) <= MAX_ECC_ROT_IF_WEAK_SCORE_DEG)
                            )

        val eccRotUsedDeg = if (acceptEccRotation) eccRotRawDeg else 0f

        // Zenith translation disabled on purpose: only residual rotation is allowed.
        val txUsed = 0f
        val tyUsed = 0f

        val eccRotRad = Math.toRadians(eccRotUsedDeg.toDouble())
        val cosR = kotlin.math.cos(eccRotRad)
        val sinR = kotlin.math.sin(eccRotRad)

        val warpUsed = Mat.eye(2, 3, CvType.CV_32F)
        warpUsed.put(0, 0, cosR)
        warpUsed.put(0, 1, sinR)
        warpUsed.put(1, 0, -sinR)
        warpUsed.put(1, 1, cosR)
        warpUsed.put(0, 2, txUsed.toDouble())
        warpUsed.put(1, 2, tyUsed.toDouble())

        Log.d(
            "AtlasZenithEccGate",
            "score=${"%.5f".format(eccScore)} " +
                    "rawRot=${"%.2f".format(eccRotRawDeg)} " +
                    "usedRot=${"%.2f".format(eccRotUsedDeg)} " +
                    "accepted=$acceptEccRotation " +
                    "txRaw=${"%.2f".format(tx)} " +
                    "tyRaw=${"%.2f".format(ty)} " +
                    "txUsed=0.00 tyUsed=0.00 translationDisabled=true"
        )

        val alignedRgba = Mat()
        val alignedMask = Mat()
        val alignedGray = Mat()

        Imgproc.warpAffine(
            rotatedRgba,
            alignedRgba,
            warpUsed,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_LINEAR + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0, 0.0, 0.0, 0.0)
        )

        Imgproc.warpAffine(
            rotatedMask,
            alignedMask,
            warpUsed,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_NEAREST + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0)
        )

        Imgproc.cvtColor(alignedRgba, alignedGray, Imgproc.COLOR_RGBA2GRAY)
        alignedGray.convertTo(alignedGray, CvType.CV_32F, 1.0 / 255.0)

        rotM.release()
        rotatedRgba.release()
        rotatedMask.release()
        rotatedGray.release()
        altitudeMask.release()
        eccMask.release()
        warp.release()
        warpUsed.release()

        return RefinementResult(
            initialTwistDeg = initialTwistDeg,
            finalTwistDeg = normalizeDeg(initialTwistDeg + eccRotUsedDeg),
            eccRotationDeg = eccRotUsedDeg,
            eccTxPx = txUsed,
            eccTyPx = tyUsed,
            eccScore = eccScore,
            alignedTopFace = TopFace(
                rgba = alignedRgba,
                gray32 = alignedGray,
                validMask = alignedMask,
                faceSizePx = moving.faceSizePx
            )
        )
    }

    private fun blendTopFaceIntoAtlas(
        aligned: TopFace,
        atlas: SkyAtlas,
        frameWeight: Float,
        minAltitudeDeg: Float
    ): ColorCorrection {
        val topPixels = rgbaMatToArgb(aligned.rgba)
        val correction = computeOverlapColorCorrection(
            aligned = aligned,
            atlas = atlas,
            facePixels = topPixels,
            minAltitudeDeg = minAltitudeDeg
        )
        val correctionStrength = computeColorCorrectionStrength(correction)

        var coveredWrites = 0
        var uncoveredWrites = 0
        var nullSamples = 0
        var directSamples = 0
        var fallbackSamples = 0
        var seamCorrectedWrites = 0
        var neutralWrites = 0
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg) continue

            for (x in 0 until atlas.width) {
                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceWithFallback(
                    face = aligned,
                    facePixels = topPixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                )
                if (sampled == null) {
                    nullSamples++
                    continue
                }

                if (sampled.usedFallback) fallbackSamples++ else directSamples++

                val hasBaseCoverage = atlas.hasCoverageAt(x, y)
                val baseWeight = atlas.weightAt(x, y)

                val edgeT = ((altDeg - ZENITH_EDGE_FADE_START_ALT_DEG) /
                        (ZENITH_EDGE_FADE_FULL_ALT_DEG - ZENITH_EDGE_FADE_START_ALT_DEG))
                    .coerceIn(0f, 1f)

                val zenithEdgeAlpha = ZENITH_EDGE_MIN_ALPHA +
                        (1f - ZENITH_EDGE_MIN_ALPHA) * (edgeT * edgeT * (3f - 2f * edgeT))

                val overlapAlpha = if (hasBaseCoverage) {
                    val t = ((altDeg - BLEND_FEATHER_START_ALT_DEG) /
                            (BLEND_FEATHER_FULL_ALT_DEG - BLEND_FEATHER_START_ALT_DEG))
                        .coerceIn(0f, 1f)
                    t * t * (3f - 2f * t)
                } else {
                    1f
                }

                val baseT = (baseWeight / ZENITH_BLEND_BASE_WEIGHT_REF).coerceIn(0f, 1f)
                val baseSuppression = if (hasBaseCoverage) {
                    1f - ZENITH_BLEND_MAX_BASE_SUPPRESSION * baseT
                } else {
                    1f
                }

                val finalWeight = frameWeight * zenithEdgeAlpha * overlapAlpha * baseSuppression

                if (finalWeight <= 0f) continue

                val applySeamColorCorrection = hasBaseCoverage

                val gainR = if (applySeamColorCorrection) {
                    remapGainTowardNeutralByAltitude(
                        gain = correction.rGain,
                        altDeg = altDeg,
                        correctionStrength = correctionStrength
                    )
                } else {
                    1f
                }

                val gainG = if (applySeamColorCorrection) {
                    remapGainTowardNeutralByAltitude(
                        gain = correction.gGain,
                        altDeg = altDeg,
                        correctionStrength = correctionStrength
                    )
                } else {
                    1f
                }

                val gainB = if (applySeamColorCorrection) {
                    remapGainTowardNeutralByAltitude(
                        gain = correction.bGain,
                        altDeg = altDeg,
                        correctionStrength = correctionStrength
                    )
                } else {
                    1f
                }

                val corrected = applyRgbGain(
                    sampled.color,
                    gainR,
                    gainG,
                    gainB
                )
                atlas.blendPixel(x, y, corrected, finalWeight)
                if (applySeamColorCorrection) seamCorrectedWrites++ else neutralWrites++
                if (hasBaseCoverage) coveredWrites++ else uncoveredWrites++
            }
        }

        Log.d(
            "AtlasZenithBlend",
            "minAlt=${"%.1f".format(minAltitudeDeg)} " +
                    "overlap=${correction.overlapCount} " +
                    "gainR=${"%.3f".format(correction.rGain)} " +
                    "gainG=${"%.3f".format(correction.gGain)} " +
                    "gainB=${"%.3f".format(correction.bGain)} " +
                    "correctionStrength=${"%.2f".format(correctionStrength)} " +
                    "coveredWrites=$coveredWrites " +
                    "uncoveredWrites=$uncoveredWrites " +
                    "seamCorrectedWrites=$seamCorrectedWrites " +
                    "neutralWrites=$neutralWrites " +
                    "nullSamples=$nullSamples " +
                    "directSamples=$directSamples " +
                    "fallbackSamples=$fallbackSamples"
        )

        return correction
    }

    private fun computeOverlapColorCorrection(
        aligned: TopFace,
        atlas: SkyAtlas,
        facePixels: IntArray,
        minAltitudeDeg: Float
    ): ColorCorrection {
        var refRSum = 0f
        var refGSum = 0f
        var refBSum = 0f

        var movRSum = 0f
        var movGSum = 0f
        var movBSum = 0f

        var overlapCount = 0

        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < maxOf(minAltitudeDeg, ZENITH_COLOR_CORR_MIN_ALT_DEG)) continue
            if (altDeg > ZENITH_COLOR_CORR_MAX_ALT_DEG) continue

            for (x in 0 until atlas.width) {
                if (atlas.weightAt(x, y) < ZENITH_COLOR_CORR_MIN_BASE_WEIGHT) continue

                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceWithFallback(
                    face = aligned,
                    facePixels = facePixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                ) ?: continue

                val refColor = atlas.pixels[atlas.index(x, y)]

                refRSum += Color.red(refColor)
                refGSum += Color.green(refColor)
                refBSum += Color.blue(refColor)

                movRSum += Color.red(sampled.color)
                movGSum += Color.green(sampled.color)
                movBSum += Color.blue(sampled.color)

                overlapCount++
            }
        }

        val rGain = if (overlapCount > 0 && movRSum > 1e-3f) {
            (refRSum / movRSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        val gGain = if (overlapCount > 0 && movGSum > 1e-3f) {
            (refGSum / movGSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        val bGain = if (overlapCount > 0 && movBSum > 1e-3f) {
            (refBSum / movBSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        return ColorCorrection(
            rGain = rGain,
            gGain = gGain,
            bGain = bGain,
            overlapCount = overlapCount
        )
    }

    private fun fillUncoveredPolarCapFromTopFace(
        aligned: TopFace,
        atlas: SkyAtlas,
        frameWeight: Float,
        minAltitudeDeg: Float,
        colorCorrection: ColorCorrection
    ) {
        if (frameWeight <= 0f) return

        val topPixels = rgbaMatToArgb(aligned.rgba)
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        val polarCorrectionStrength =
            (computeColorCorrectionStrength(colorCorrection) * POLAR_CAP_COLOR_STRENGTH_SCALE)
                .coerceIn(POLAR_CAP_MIN_COLOR_STRENGTH, 0.35f)

        var candidateCount = 0
        var filledCount = 0
        var directCount = 0
        var fallbackCount = 0
        var missedCount = 0

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg) continue

            val t = ((altDeg - minAltitudeDeg) / (90f - minAltitudeDeg).coerceAtLeast(1f))
                .coerceIn(0f, 1f)
            val smoothT = t * t * (3f - 2f * t)
            val fillWeightScale =
                POLAR_CAP_FILL_WEIGHT_AT_EDGE +
                        (POLAR_CAP_FILL_WEIGHT_AT_POLE - POLAR_CAP_FILL_WEIGHT_AT_EDGE) * smoothT

            for (x in 0 until atlas.width) {
                if (atlas.weightAt(x, y) >= POLAR_CAP_MIN_EXISTING_WEIGHT) continue

                candidateCount++

                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                val sampled = sampleTopFaceWithFallback(
                    face = aligned,
                    facePixels = topPixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                )

                if (sampled == null) {
                    missedCount++
                    continue
                }

                val gainR = remapGainTowardNeutralByAltitude(
                    gain = colorCorrection.rGain,
                    altDeg = altDeg,
                    correctionStrength = polarCorrectionStrength
                )
                val gainG = remapGainTowardNeutralByAltitude(
                    gain = colorCorrection.gGain,
                    altDeg = altDeg,
                    correctionStrength = polarCorrectionStrength
                )
                val gainB = remapGainTowardNeutralByAltitude(
                    gain = colorCorrection.bGain,
                    altDeg = altDeg,
                    correctionStrength = polarCorrectionStrength
                )

                val corrected = applyRgbGain(
                    sampled.color,
                    gainR,
                    gainG,
                    gainB
                )

                atlas.blendPixel(x, y, corrected, frameWeight * fillWeightScale)
                filledCount++

                if (sampled.usedFallback) fallbackCount++ else directCount++
            }
        }

        Log.d(
            "AtlasZenithPolarFill",
            "minAlt=${"%.1f".format(minAltitudeDeg)} " +
                    "correctionStrength=${"%.2f".format(polarCorrectionStrength)} " +
                    "candidates=$candidateCount filled=$filledCount " +
                    "direct=$directCount fallback=$fallbackCount missed=$missedCount"
        )
    }

    private fun sampleTopFaceWithFallback(
        face: TopFace,
        facePixels: IntArray,
        azDeg: Float,
        altDeg: Float
    ): TopFaceSample? {
        val coords = projectWorldToTopFace(face, azDeg, altDeg) ?: return null

        val direct = sampleTopFaceBilinearAt(
            face = face,
            facePixels = facePixels,
            fx = coords.first,
            fy = coords.second
        )

        if (direct != null) {
            return TopFaceSample(
                color = direct,
                usedFallback = false
            )
        }

        val fallback = sampleNearestValidTopFace(
            face = face,
            facePixels = facePixels,
            fx = coords.first,
            fy = coords.second,
            maxRadiusPx = POLAR_CAP_NEAREST_RADIUS_PX
        ) ?: return null

        return TopFaceSample(
            color = fallback,
            usedFallback = true
        )
    }

    private fun projectWorldToTopFace(
        face: TopFace,
        azDeg: Float,
        altDeg: Float
    ): Pair<Float, Float>? {
        val dir = worldDirectionRad(
            degToRad(azDeg),
            degToRad(altDeg)
        )

        if (dir[2] <= 1e-6f) return null

        val center = (face.faceSizePx - 1) * 0.5f
        val radius = center

        val fx = center + radius * (dir[0] / dir[2])
        val fy = center + radius * (dir[1] / dir[2])

        if (fx < 0f || fy < 0f || fx > face.faceSizePx - 1f || fy > face.faceSizePx - 1f) {
            return null
        }

        return fx to fy
    }

    private fun sampleNearestValidTopFace(
        face: TopFace,
        facePixels: IntArray,
        fx: Float,
        fy: Float,
        maxRadiusPx: Int
    ): Int? {
        val cx = fx.roundToInt().coerceIn(0, face.faceSizePx - 1)
        val cy = fy.roundToInt().coerceIn(0, face.faceSizePx - 1)

        for (radius in 1..maxRadiusPx) {
            var bestColor: Int? = null
            var bestD2 = Float.POSITIVE_INFINITY

            val left = (cx - radius).coerceAtLeast(0)
            val right = (cx + radius).coerceAtMost(face.faceSizePx - 1)
            val top = (cy - radius).coerceAtLeast(0)
            val bottom = (cy + radius).coerceAtMost(face.faceSizePx - 1)

            for (y in top..bottom) {
                for (x in left..right) {
                    if (x != left && x != right && y != top && y != bottom) continue

                    val valid = face.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                    if (valid <= 0) continue

                    val dx = x - fx
                    val dy = y - fy
                    val d2 = dx * dx + dy * dy

                    if (d2 < bestD2) {
                        bestD2 = d2
                        bestColor = facePixels[y * face.faceSizePx + x]
                    }
                }
            }

            if (bestColor != null) return bestColor
        }

        return null
    }

    private fun sampleTopFaceBilinear(
        face: TopFace,
        facePixels: IntArray,
        azDeg: Float,
        altDeg: Float
    ): Int? {
        val dir = worldDirectionRad(
            degToRad(azDeg),
            degToRad(altDeg)
        )

        if (dir[2] <= 1e-6f) return null

        val center = (face.faceSizePx - 1) * 0.5f
        val radius = center

        val fx = center + radius * (dir[0] / dir[2])
        val fy = center + radius * (dir[1] / dir[2])

        if (fx < 0f || fy < 0f || fx > face.faceSizePx - 1f || fy > face.faceSizePx - 1f) {
            return null
        }

        return sampleTopFaceBilinearAt(face, facePixels, fx, fy)
    }

    private fun sampleTopFaceBilinearAt(
        face: TopFace,
        facePixels: IntArray,
        fx: Float,
        fy: Float
    ): Int? {
        val x0 = fx.toInt().coerceIn(0, face.faceSizePx - 1)
        val y0 = fy.toInt().coerceIn(0, face.faceSizePx - 1)
        val x1 = (x0 + 1).coerceIn(0, face.faceSizePx - 1)
        val y1 = (y0 + 1).coerceIn(0, face.faceSizePx - 1)

        val m00 = face.validMask.get(y0, x0)?.firstOrNull()?.toInt() ?: 0
        val m10 = face.validMask.get(y0, x1)?.firstOrNull()?.toInt() ?: 0
        val m01 = face.validMask.get(y1, x0)?.firstOrNull()?.toInt() ?: 0
        val m11 = face.validMask.get(y1, x1)?.firstOrNull()?.toInt() ?: 0

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val w00 = (1f - dx) * (1f - dy)
        val w10 = dx * (1f - dy)
        val w01 = (1f - dx) * dy
        val w11 = dx * dy

        val c00 = facePixels[y0 * face.faceSizePx + x0]
        val c10 = facePixels[y0 * face.faceSizePx + x1]
        val c01 = facePixels[y1 * face.faceSizePx + x0]
        val c11 = facePixels[y1 * face.faceSizePx + x1]

        var sumW = 0f
        var sumA = 0f
        var sumR = 0f
        var sumG = 0f
        var sumB = 0f

        fun acc(mask: Int, weight: Float, color: Int) {
            if (mask <= 0 || weight <= 0f) return
            sumW += weight
            sumA += Color.alpha(color) * weight
            sumR += Color.red(color) * weight
            sumG += Color.green(color) * weight
            sumB += Color.blue(color) * weight
        }

        acc(m00, w00, c00)
        acc(m10, w10, c10)
        acc(m01, w01, c01)
        acc(m11, w11, c11)

        if (sumW <= 1e-6f) return null

        val a = (sumA / sumW).roundToInt().coerceIn(0, 255)
        val r = (sumR / sumW).roundToInt().coerceIn(0, 255)
        val g = (sumG / sumW).roundToInt().coerceIn(0, 255)
        val b = (sumB / sumW).roundToInt().coerceIn(0, 255)

        return Color.argb(a, r, g, b)
    }

    private fun applyRgbGain(
        color: Int,
        rGain: Float,
        gGain: Float,
        bGain: Float
    ): Int {
        val a = Color.alpha(color)
        val r = (Color.red(color) * rGain).roundToInt().coerceIn(0, 255)
        val g = (Color.green(color) * gGain).roundToInt().coerceIn(0, 255)
        val b = (Color.blue(color) * bGain).roundToInt().coerceIn(0, 255)
        return Color.argb(a, r, g, b)
    }

    private fun acceptanceMode(
        result: RefinementResult,
        seamScore: CandidateSeamScore,
        residualTwistAbsDeg: Float,
        maxResidualTwistDeg: Float
    ): String {
        val strictAccept =
            result.eccScore >= MIN_FINAL_BLEND_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_FINAL_BLEND_ABS_ROT_DEG &&
                    residualTwistAbsDeg <= maxResidualTwistDeg

        if (strictAccept) return "strict"

        val seamRescueAccept =
            result.eccScore >= MIN_FINAL_BLEND_SOFT_ECC_SCORE &&
                    abs(result.eccRotationDeg) <= MAX_FINAL_BLEND_SOFT_ABS_ROT_DEG &&
                    residualTwistAbsDeg <= minOf(
                maxResidualTwistDeg,
                MAX_FINAL_BLEND_SOFT_RESIDUAL_TWIST_DEG
            ) &&
                    seamScore.meanAbsLumaDiff01 <= MAX_FINAL_BLEND_SOFT_MAX_LUMA_DIFF01

        return if (seamRescueAccept) "seam_rescue" else "rejected"
    }

    // New acceptance policy for the global-annulus strategy.
    // Independent of the old acceptanceMode (which stays in the file as dead
    // code for now). Decouples acceptance from ECC convergence: the global
    // annulus correlation itself is strong enough evidence when the peak is
    // clear, and ECC in near-polar repetitive textures is known to be fragile.
    //
    // Return values:
    //   "global"       — normal acceptance (|shift| <= threshold)
    //   "global_large" — accepted but the shift was big; stricter corr required
    //   "rejected"     — failed one of the gates
    private fun globalAcceptanceMode(
        finalSeamScore: CandidateSeamScore,
        globalBestCorr: Float,
        globalPeakMarginOverMedian: Float,
        signedGlobalShiftDeg: Float,
        residualTwistAbsDeg: Float,
        maxResidualTwistDeg: Float
    ): String {
        // Gate 1: residual after local refine must not exceed the blend-max
        // (this catches pathological cases where phase+ECC blew up).
        if (residualTwistAbsDeg > maxResidualTwistDeg) return "rejected"

        // Gate 2: winner correlation must be above absolute minimum.
        if (globalBestCorr < MIN_GLOBAL_ANNULUS_CORR) return "rejected"

        // Gate 3: the peak must stand out clearly above the median shift.
        if (globalPeakMarginOverMedian < MIN_GLOBAL_PEAK_MARGIN_OVER_MEDIAN) return "rejected"

        // Gate 4: for large shifts, require extra-strong correlation.
        if (abs(signedGlobalShiftDeg) > GLOBAL_LARGE_SHIFT_THRESHOLD_DEG) {
            if (globalBestCorr < GLOBAL_LARGE_SHIFT_MIN_CORR) return "rejected"
            return "global_large"
        }

        // Note: finalSeamScore is passed in for potential future use (e.g.
        // a sanity check on post-refine meanAbsLumaDiff01), but NOT used as
        // a filter today. The global correlation is the authority.
        @Suppress("UNUSED_PARAMETER")
        val _ignored = finalSeamScore

        return "global"
    }
    private fun computeColorCorrectionStrength(
        correction: ColorCorrection
    ): Float {
        val allAtFloor =
            abs(correction.rGain - RGB_GAIN_MIN) <= COLOR_GAIN_CLAMP_EPS &&
                    abs(correction.gGain - RGB_GAIN_MIN) <= COLOR_GAIN_CLAMP_EPS &&
                    abs(correction.bGain - RGB_GAIN_MIN) <= COLOR_GAIN_CLAMP_EPS

        val allAtCeil =
            abs(correction.rGain - RGB_GAIN_MAX) <= COLOR_GAIN_CLAMP_EPS &&
                    abs(correction.gGain - RGB_GAIN_MAX) <= COLOR_GAIN_CLAMP_EPS &&
                    abs(correction.bGain - RGB_GAIN_MAX) <= COLOR_GAIN_CLAMP_EPS

        return if (allAtFloor || allAtCeil) {
            MIN_COLOR_CORRECTION_STRENGTH
        } else {
            1f
        }
    }

    private fun remapGainTowardNeutralByAltitude(
        gain: Float,
        altDeg: Float,
        correctionStrength: Float
    ): Float {
        val seamT = 1f - (
                (altDeg - COLOR_MATCH_FULL_AT_ALT_DEG) /
                        (COLOR_MATCH_NEUTRAL_AT_ALT_DEG - COLOR_MATCH_FULL_AT_ALT_DEG)
                ).coerceIn(0f, 1f)

        val effectiveT = (seamT * correctionStrength).coerceIn(0f, 1f)

        return 1f + (gain - 1f) * effectiveT
    }



    fun releaseTopFace(face: TopFace) {
        face.rgba.release()
        face.gray32.release()
        face.validMask.release()
    }
    private fun estimateTwistWithPolarPhaseCorrelation(
        reference: TopFace,
        moving: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        coarseTwistDeg: Float,
        angleBins: Int = POLAR_PHASE_ANGLE_BINS,
        radialBins: Int = POLAR_PHASE_RADIAL_BINS
    ): Float? {
        val refStrip = buildPolarStripForPhaseCorrelation(
            face = reference,
            minAltitudeDeg = minAltitudeDeg,
            maxAltitudeDeg = maxAltitudeDeg,
            angleBins = angleBins,
            radialBins = radialBins
        )
        val movStrip = buildPolarStripForPhaseCorrelation(
            face = moving,
            minAltitudeDeg = minAltitudeDeg,
            maxAltitudeDeg = maxAltitudeDeg,
            angleBins = angleBins,
            radialBins = radialBins
        )

        val refGrayTiled = tilePolarStripX(refStrip.gray32, repeatCount = 2)
        val movGrayTiled = tilePolarStripX(movStrip.gray32, repeatCount = 2)
        val refValidTiled = tilePolarStripX(refStrip.valid32, repeatCount = 2)
        val movValidTiled = tilePolarStripX(movStrip.valid32, repeatCount = 2)

        val commonMask = Mat()
        val hann = Mat()
        val window = Mat()

        try {
            Core.min(refValidTiled, movValidTiled, commonMask)

            val totalCells = (commonMask.rows() * commonMask.cols()).coerceAtLeast(1)
            val commonCoverage = (Core.sumElems(commonMask).`val`[0] / totalCells.toDouble()).toFloat()

            if (commonCoverage < MIN_POLAR_PHASE_COVERAGE) {
                Log.d(
                    "AtlasZenithTwistPhase",
                    "coverage=${"%.3f".format(commonCoverage)} tooLow fallback=${"%.2f".format(coarseTwistDeg)}"
                )
                return null
            }

            Imgproc.createHanningWindow(
                hann,
                Size(refGrayTiled.cols().toDouble(), refGrayTiled.rows().toDouble()),
                CvType.CV_32F
            )
            Core.multiply(hann, commonMask, window)

            val response = doubleArrayOf(0.0)
            val shift = Imgproc.phaseCorrelate(
                refGrayTiled,
                movGrayTiled,
                window,
                response
            )

            var shiftX = shift.x.toFloat() % angleBins.toFloat()
            if (shiftX > angleBins * 0.5f) shiftX -= angleBins.toFloat()
            if (shiftX < -angleBins * 0.5f) shiftX += angleBins.toFloat()

            val rawTwistDeg = shiftX * 360f / angleBins.toFloat()

            val candidatePos = normalizeDeg(rawTwistDeg)
            val candidateNeg = normalizeDeg(-rawTwistDeg)

            val chosen = if (
                angularDistanceDeg(candidatePos, coarseTwistDeg) <=
                angularDistanceDeg(candidateNeg, coarseTwistDeg)
            ) {
                candidatePos
            } else {
                candidateNeg
            }

            Log.d(
                "AtlasZenithTwistPhase",
                "coverage=${"%.3f".format(commonCoverage)} " +
                        "response=${"%.5f".format(response[0])} " +
                        "shiftX=${"%.2f".format(shiftX)} " +
                        "candPos=${"%.2f".format(candidatePos)} " +
                        "candNeg=${"%.2f".format(candidateNeg)} " +
                        "coarse=${"%.2f".format(coarseTwistDeg)} " +
                        "chosen=${"%.2f".format(chosen)}"
            )

            return if (response[0] >= MIN_POLAR_PHASE_RESPONSE) chosen else null
        } finally {
            refStrip.gray32.release()
            refStrip.valid32.release()
            movStrip.gray32.release()
            movStrip.valid32.release()
            refGrayTiled.release()
            movGrayTiled.release()
            refValidTiled.release()
            movValidTiled.release()
            commonMask.release()
            hann.release()
            window.release()
        }
    }

    private fun buildPolarStripForPhaseCorrelation(
        face: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int,
        radialBins: Int
    ): PolarPhaseStrip {
        val gray = Mat.zeros(radialBins, angleBins, CvType.CV_32F)
        val valid = Mat.zeros(radialBins, angleBins, CvType.CV_32F)

        val center = (face.faceSizePx - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToTopRadiusPx(maxAltitudeDeg, maxRadius)
        val outerRadius = altitudeToTopRadiusPx(minAltitudeDeg, maxRadius)

        for (ry in 0 until radialBins) {
            val tR = if (radialBins <= 1) 0f else ry.toFloat() / (radialBins - 1).toFloat()
            val radius = innerRadius + (outerRadius - innerRadius) * tR

            val grayRow = FloatArray(angleBins)
            val validRow = FloatArray(angleBins)

            for (ax in 0 until angleBins) {
                val theta = (2.0 * PI * ax.toDouble()) / angleBins.toDouble()
                val fx = center + radius * cos(theta).toFloat()
                val fy = center + radius * sin(theta).toFloat()

                val sampled = sampleGray32WithMaskBilinear(face, fx, fy) ?: continue
                grayRow[ax] = sampled.first
                validRow[ax] = sampled.second
            }

            gray.put(ry, 0, grayRow)
            valid.put(ry, 0, validRow)
        }

        normalizePolarStripInPlace(gray, valid)

        return PolarPhaseStrip(
            gray32 = gray,
            valid32 = valid,
            angleBins = angleBins,
            radialBins = radialBins
        )
    }

    private fun sampleGray32WithMaskBilinear(
        face: TopFace,
        fx: Float,
        fy: Float
    ): Pair<Float, Float>? {
        if (fx < 0f || fy < 0f || fx > face.faceSizePx - 1f || fy > face.faceSizePx - 1f) {
            return null
        }

        val x0 = fx.toInt().coerceIn(0, face.faceSizePx - 1)
        val y0 = fy.toInt().coerceIn(0, face.faceSizePx - 1)
        val x1 = (x0 + 1).coerceIn(0, face.faceSizePx - 1)
        val y1 = (y0 + 1).coerceIn(0, face.faceSizePx - 1)

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val w00 = (1f - dx) * (1f - dy)
        val w10 = dx * (1f - dy)
        val w01 = (1f - dx) * dy
        val w11 = dx * dy

        val m00 = if ((face.validMask.get(y0, x0)?.firstOrNull() ?: 0.0) > 0.0) 1f else 0f
        val m10 = if ((face.validMask.get(y0, x1)?.firstOrNull() ?: 0.0) > 0.0) 1f else 0f
        val m01 = if ((face.validMask.get(y1, x0)?.firstOrNull() ?: 0.0) > 0.0) 1f else 0f
        val m11 = if ((face.validMask.get(y1, x1)?.firstOrNull() ?: 0.0) > 0.0) 1f else 0f

        val g00 = (face.gray32.get(y0, x0)?.firstOrNull() ?: 0.0).toFloat()
        val g10 = (face.gray32.get(y0, x1)?.firstOrNull() ?: 0.0).toFloat()
        val g01 = (face.gray32.get(y1, x0)?.firstOrNull() ?: 0.0).toFloat()
        val g11 = (face.gray32.get(y1, x1)?.firstOrNull() ?: 0.0).toFloat()

        var sumW = 0f
        var sumG = 0f

        fun acc(mask: Float, weight: Float, value: Float) {
            if (mask <= 0f || weight <= 0f) return
            val ww = mask * weight
            sumW += ww
            sumG += value * ww
        }

        acc(m00, w00, g00)
        acc(m10, w10, g10)
        acc(m01, w01, g01)
        acc(m11, w11, g11)

        if (sumW <= 1e-6f) return null

        val gray = sumG / sumW
        val coverage = sumW.coerceIn(0f, 1f)
        return gray to coverage
    }

    private fun normalizePolarStripInPlace(
        gray: Mat,
        valid: Mat
    ) {
        val rows = gray.rows()
        val cols = gray.cols()

        var sum = 0.0
        var weightSum = 0.0

        for (y in 0 until rows) {
            val grayRow = FloatArray(cols)
            val validRow = FloatArray(cols)
            gray.get(y, 0, grayRow)
            valid.get(y, 0, validRow)

            for (x in 0 until cols) {
                val w = validRow[x].toDouble()
                if (w <= 1e-6) continue
                sum += grayRow[x] * w
                weightSum += w
            }
        }

        if (weightSum <= 1e-6) {
            gray.setTo(Scalar(0.0))
            return
        }

        val mean = (sum / weightSum).toFloat()

        var varSum = 0.0
        for (y in 0 until rows) {
            val grayRow = FloatArray(cols)
            val validRow = FloatArray(cols)
            gray.get(y, 0, grayRow)
            valid.get(y, 0, validRow)

            for (x in 0 until cols) {
                val w = validRow[x].toDouble()
                if (w <= 1e-6) continue
                val d = (grayRow[x] - mean).toDouble()
                varSum += d * d * w
            }
        }

        val std = sqrt((varSum / weightSum).coerceAtLeast(1e-9)).toFloat()

        for (y in 0 until rows) {
            val grayRow = FloatArray(cols)
            val validRow = FloatArray(cols)
            gray.get(y, 0, grayRow)
            valid.get(y, 0, validRow)

            for (x in 0 until cols) {
                val w = validRow[x]
                grayRow[x] = if (w > 1e-6f) {
                    ((grayRow[x] - mean) / std) * w
                } else {
                    0f
                }
            }

            gray.put(y, 0, grayRow)
        }
    }

    private fun tilePolarStripX(
        src: Mat,
        repeatCount: Int
    ): Mat {
        val dst = Mat.zeros(src.rows(), src.cols() * repeatCount, src.type())
        for (i in 0 until repeatCount) {
            val roi = dst.colRange(i * src.cols(), (i + 1) * src.cols())
            src.copyTo(roi)
            roi.release()
        }
        return dst
    }

    private fun angularDistanceDeg(
        aDeg: Float,
        bDeg: Float
    ): Float {
        return abs(normalizeSignedDeg(aDeg - bDeg))
    }

    private fun normalizeSignedDeg(valueDeg: Float): Float {
        var v = valueDeg
        while (v <= -180f) v += 360f
        while (v > 180f) v -= 360f
        return v
    }
    private fun buildAnnulusSignature(
        face: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int
    ): FloatArray {
        val signature = FloatArray(angleBins)

        val center = (face.faceSizePx - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToTopRadiusPx(maxAltitudeDeg, maxRadius)
        val outerRadius = altitudeToTopRadiusPx(minAltitudeDeg, maxRadius)

        for (bin in 0 until angleBins) {
            val theta = (2.0 * PI * bin.toDouble()) / angleBins.toDouble()

            var sum = 0.0
            var count = 0

            var r = innerRadius
            while (r <= outerRadius + 1e-3f) {
                val x = (center + r * cos(theta).toFloat()).roundToInt().coerceIn(0, face.faceSizePx - 1)
                val y = (center + r * sin(theta).toFloat()).roundToInt().coerceIn(0, face.faceSizePx - 1)

                val valid = face.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                if (valid > 0) {
                    sum += face.gray32.get(y, x)[0]
                    count++
                }
                r += 2f
            }

            signature[bin] = if (count > 0) (sum / count).toFloat() else 0f
        }

        val mean = signature.average().toFloat()
        var varSum = 0f
        for (i in signature.indices) {
            signature[i] -= mean
            varSum += signature[i] * signature[i]
        }
        val std = sqrt((varSum / signature.size).coerceAtLeast(1e-9f))
        for (i in signature.indices) {
            signature[i] /= std
        }

        return signature
    }

    private fun buildAltitudeMask(
        faceSizePx: Int,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float
    ): Mat {
        val mask = Mat.zeros(faceSizePx, faceSizePx, CvType.CV_8UC1)

        val center = (faceSizePx - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToTopRadiusPx(maxAltitudeDeg, maxRadius)
        val outerRadius = altitudeToTopRadiusPx(minAltitudeDeg, maxRadius)

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dx = x - center
                val dy = y - center
                val r = sqrt(dx * dx + dy * dy)
                if (r in innerRadius..outerRadius) {
                    mask.put(y, x, 255.0)
                }
            }
        }

        return mask
    }

    private fun topFaceFromArgbAndMask(
        argb: IntArray,
        mask: ByteArray,
        faceSizePx: Int
    ): TopFace {
        val bmp = Bitmap.createBitmap(faceSizePx, faceSizePx, Bitmap.Config.ARGB_8888)
        bmp.setPixels(argb, 0, faceSizePx, 0, 0, faceSizePx, faceSizePx)

        val rgba = Mat()
        Utils.bitmapToMat(bmp, rgba)

        val gray32 = Mat()
        Imgproc.cvtColor(rgba, gray32, Imgproc.COLOR_RGBA2GRAY)
        gray32.convertTo(gray32, CvType.CV_32F, 1.0 / 255.0)

        val validMask = Mat(faceSizePx, faceSizePx, CvType.CV_8UC1)
        validMask.put(0, 0, mask)

        return TopFace(
            rgba = rgba,
            gray32 = gray32,
            validMask = validMask,
            faceSizePx = faceSizePx
        )
    }

    private fun rgbaMatToArgb(rgba: Mat): IntArray {
        val width = rgba.cols()
        val height = rgba.rows()
        val raw = ByteArray(width * height * 4)
        rgba.get(0, 0, raw)

        val out = IntArray(width * height)
        var j = 0
        for (i in out.indices) {
            val r = raw[j].toInt() and 0xFF
            val g = raw[j + 1].toInt() and 0xFF
            val b = raw[j + 2].toInt() and 0xFF
            val a = raw[j + 3].toInt() and 0xFF
            out[i] = Color.argb(a, r, g, b)
            j += 4
        }
        return out
    }

    private fun altitudeToTopRadiusPx(altitudeDeg: Float, maxRadius: Float): Float {
        val zenithAngleDeg = 90f - altitudeDeg
        return tan(degToRad(zenithAngleDeg)) * maxRadius
    }

    private fun topFaceDirection(
        x: Int,
        y: Int,
        center: Float,
        radius: Float
    ): FloatArray {
        val px = (x - center) / radius
        val py = (y - center) / radius
        return normalize(floatArrayOf(px, py, 1f))
    }

    private fun degToRad(value: Float): Float = Math.toRadians(value.toDouble()).toFloat()
    private fun radToDeg(value: Float): Float = Math.toDegrees(value.toDouble()).toFloat()

    private fun normalizeDeg(value: Float): Float {
        var v = value % 360f
        if (v < 0f) v += 360f
        return v
    }

    private fun worldDirectionRad(yawRad: Float, pitchRad: Float): FloatArray {
        val cp = cos(pitchRad)
        val sp = sin(pitchRad)

        val xEast = cp * sin(yawRad)
        val yNorth = cp * cos(yawRad)

        return floatArrayOf(xEast, yNorth, sp)
    }

    private fun dot(a: FloatArray, b: FloatArray): Float =
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun cross(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        )

    private fun length(v: FloatArray): Float = sqrt(dot(v, v))

    private fun normalize(v: FloatArray): FloatArray {
        val n = length(v).coerceAtLeast(1e-6f)
        return floatArrayOf(v[0] / n, v[1] / n, v[2] / n)
    }

    private fun scale(v: FloatArray, s: Float): FloatArray =
        floatArrayOf(v[0] * s, v[1] * s, v[2] * s)

    private fun add(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])
}