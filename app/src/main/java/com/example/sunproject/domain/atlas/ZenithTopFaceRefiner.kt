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
    private const val ANNULUS_MIN_ALT_DEG = 72f
    private const val ANNULUS_MAX_ALT_DEG = 78f
    private const val ECC_MIN_ALT_DEG = 68f
    private const val ECC_MAX_ALT_DEG = 88f
    private data class PolarPhaseStrip(
        val gray32: Mat,
        val valid32: Mat,
        val angleBins: Int,
        val radialBins: Int
    )

    private const val BLEND_MIN_ALT_DEG = 74f
    private const val POLAR_PHASE_MIN_ALT_DEG = 74f
    private const val POLAR_PHASE_MAX_ALT_DEG = 88f
    private const val POLAR_PHASE_ANGLE_BINS = 720
    private const val POLAR_PHASE_RADIAL_BINS = 96
    private const val MIN_POLAR_PHASE_RESPONSE = 0.03
    private const val MIN_POLAR_PHASE_COVERAGE = 0.12f

    private const val BLEND_FEATHER_START_ALT_DEG = 80f
    private const val BLEND_FEATHER_FULL_ALT_DEG = 88f
    private const val ZENITH_EDGE_FADE_START_ALT_DEG = BLEND_MIN_ALT_DEG
    private const val ZENITH_EDGE_FADE_FULL_ALT_DEG = 84f
    private const val ZENITH_EDGE_MIN_ALPHA = 0.15f

    private const val RGB_GAIN_MIN = 0.92f
    private const val RGB_GAIN_MAX = 1.08f

    private const val POLAR_CAP_FILL_MIN_ALT_DEG = 84f
    private const val POLAR_CAP_NEAREST_RADIUS_PX = 2
    private const val MIN_ECC_ACCEPT_SCORE = 0.12
    private const val STRONG_ECC_ACCEPT_SCORE = 0.20
    private const val MAX_ECC_ROT_IF_WEAK_SCORE_DEG = 3.0f
    private const val MAX_ECC_ROT_IF_STRONG_SCORE_DEG = 6.0f
    private const val COLOR_MATCH_FULL_AT_ALT_DEG = 70f
    private const val COLOR_MATCH_NEUTRAL_AT_ALT_DEG = 84f
    private const val MIN_COLOR_CORRECTION_STRENGTH = 0.35f
    private const val COLOR_GAIN_CLAMP_EPS = 0.002f
    private const val ZENITH_TOPFACE_YAW_OFFSET_DEG = -90f

    private const val MIN_FINAL_BLEND_ECC_SCORE = 0.18
    private const val MAX_FINAL_BLEND_ABS_ROT_DEG = 2.5f

    private const val POLAR_CAP_COLOR_STRENGTH_SCALE = 0.35f
    private const val POLAR_CAP_MIN_COLOR_STRENGTH = 0.08f
    private const val POLAR_CAP_FILL_WEIGHT_AT_EDGE = 0.70f
    private const val POLAR_CAP_FILL_WEIGHT_AT_POLE = 0.45f

    private const val ZENITH_BLEND_BASE_WEIGHT_REF = 1.25f
    private const val ZENITH_BLEND_MAX_BASE_SUPPRESSION = 0.55f

    private const val POLAR_CAP_MIN_EXISTING_WEIGHT = 0.18f

    private const val ZENITH_COLOR_CORR_MIN_ALT_DEG = 76f
    private const val ZENITH_COLOR_CORR_MAX_ALT_DEG = 84f
    private const val ZENITH_COLOR_CORR_MIN_BASE_WEIGHT = 0.18f


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
        val zenithTop = buildZenithTopFace(
            frame = frame,
            srcBitmap = srcBitmap,
            baseTwistDeg = seedTwistDeg,
            basePitchOffsetDeg = seedPitchOffsetDeg,
            baseRollOffsetDeg = seedRollOffsetDeg,
            absoluteYawDegOverride = seedAbsoluteYawDeg,
            absolutePitchDegOverride = seedAbsolutePitchDeg,
            absoluteRollDegOverride = seedAbsoluteRollDeg,
            faceSizePx = FACE_SIZE_PX
        )

        try {
            val residualAnnulusTwistDeg = estimateTwistFromAnnulus(
                reference = atlasTop,
                moving = zenithTop,
                minAltitudeDeg = ANNULUS_MIN_ALT_DEG,
                maxAltitudeDeg = ANNULUS_MAX_ALT_DEG
            )

            val residualInitialTwistDeg = estimateTwistWithPolarPhaseCorrelation(
                reference = atlasTop,
                moving = zenithTop,
                minAltitudeDeg = POLAR_PHASE_MIN_ALT_DEG,
                maxAltitudeDeg = POLAR_PHASE_MAX_ALT_DEG,
                coarseTwistDeg = residualAnnulusTwistDeg
            ) ?: residualAnnulusTwistDeg

            val residualRefined = refineWithEcc(
                reference = atlasTop,
                moving = zenithTop,
                initialTwistDeg = residualInitialTwistDeg,
                eccMinAltitudeDeg = ECC_MIN_ALT_DEG,
                eccMaxAltitudeDeg = ECC_MAX_ALT_DEG
            )

            val acceptFinalBlend =
                residualRefined.eccScore >= MIN_FINAL_BLEND_ECC_SCORE &&
                        abs(residualRefined.eccRotationDeg) <= MAX_FINAL_BLEND_ABS_ROT_DEG

            if (!acceptFinalBlend) {
                Log.w(
                    "AtlasZenithTopRefine",
                    "rejectBlend frame=${frame.frameId} " +
                            "initialTwist=${"%.2f".format(residualInitialTwistDeg)} " +
                            "eccRot=${"%.2f".format(residualRefined.eccRotationDeg)} " +
                            "eccScore=${"%.5f".format(residualRefined.eccScore)}"
                )
                return null
            }

            val colorCorrection = blendTopFaceIntoAtlas(
                aligned = residualRefined.alignedTopFace,
                atlas = atlas,
                frameWeight = frameWeight,
                minAltitudeDeg = BLEND_MIN_ALT_DEG
            )

            fillUncoveredPolarCapFromTopFace(
                aligned = residualRefined.alignedTopFace,
                atlas = atlas,
                frameWeight = frameWeight,
                minAltitudeDeg = POLAR_CAP_FILL_MIN_ALT_DEG,
                colorCorrection = colorCorrection
            )

            val absoluteInitialTwistDeg = normalizeDeg(seedTwistDeg + residualInitialTwistDeg)
            val absoluteFinalTwistDeg = normalizeDeg(seedTwistDeg + residualRefined.finalTwistDeg)

            return RefinementResult(
                initialTwistDeg = absoluteInitialTwistDeg,
                finalTwistDeg = absoluteFinalTwistDeg,
                eccRotationDeg = residualRefined.eccRotationDeg,
                eccTxPx = residualRefined.eccTxPx,
                eccTyPx = residualRefined.eccTyPx,
                eccScore = residualRefined.eccScore,
                alignedTopFace = residualRefined.alignedTopFace
            )
        } finally {
            releaseTopFace(atlasTop)
            releaseTopFace(zenithTop)
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
                val azDeg = normalizeDeg(radToDeg(atan2(dir[1], dir[0])))
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
                    "targetAz=${"%.2f".format(frame.targetAzimuthDeg)} " +
                    "measuredAz=${"%.2f".format(frame.measuredAzimuthDeg)} " +
                    "rawAbsYaw=${if (hasAbsoluteSeed) "%.2f".format(rawAbsoluteYawDeg) else "NaN"} " +
                    "topFaceYaw=${"%.2f".format(yawDeg)} " +
                    "pitchSeed=${"%.2f".format(pitchDeg)} " +
                    "rollSeed=${"%.2f".format(rollDeg)}"
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
        val cy = cos(yawRad)
        val sy = sin(yawRad)
        return floatArrayOf(cp * cy, cp * sy, sp)
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