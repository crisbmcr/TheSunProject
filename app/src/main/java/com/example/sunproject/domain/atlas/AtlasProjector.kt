package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import com.example.sunproject.data.model.FrameRecord
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan
import android.util.Log
import android.graphics.Color
import android.os.SystemClock
import com.example.sunproject.domain.atlas.ZenithTopFaceRefiner

private const val ZENITH_TWIST_FALLBACK_DEG = 90f
private const val ZENITH_PITCH_OFFSET_FALLBACK_DEG = 0f
private const val ZENITH_ROLL_OFFSET_FALLBACK_DEG = 0f

private const val ZENITH_PITCH_OFFSET_MIN_DEG = -4f
private const val ZENITH_PITCH_OFFSET_MAX_DEG = 1f

private const val ZENITH_ROLL_OFFSET_MIN_DEG = -6f
private const val ZENITH_ROLL_OFFSET_MAX_DEG = 6f

private const val ZENITH_OVERLAP_MIN_ALT_DEG = 72f
private const val ZENITH_OVERLAP_MAX_ALT_DEG = 78f

private const val ZENITH_ESTIMATE_MIN_PIXELS = 700
private const val ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE = 480

private const val ZENITH_ESTIMATE_MIN_SCORE = 0.05f
private const val ZENITH_ESTIMATE_MIN_CONFIDENCE = 0.002f

private const val ZENITH_ESTIMATE_MIN_GRADIENT = 10.0
private const val ZENITH_ESTIMATE_MIN_LUMA = 12.0
private const val ZENITH_ESTIMATE_MAX_LUMA = 245.0

private const val ZENITH_ESTIMATE_MAX_CANDIDATES = 96
private const val ZENITH_ESTIMATE_TIME_BUDGET_MS = 6000L
private const val ZENITH_ESTIMATE_PROGRESS_EVERY = 12

private const val ZENITH_ESTIMATE_PROJ_X_STEP = 4
private const val ZENITH_ESTIMATE_PROJ_Y_STEP = 2
private const val ZENITH_SCORE_X_STEP = 4
private const val ZENITH_SCORE_Y_STEP = 2

private const val ZENITH_ESTIMATE_MIN_IMPROVEMENT_OVER_FALLBACK = 0.015f

private const val ZENITH_ESTIMATE_DISTINCT_TWIST_DEG = 20f
private const val ZENITH_ESTIMATE_DISTINCT_PITCH_DEG = 0.75f
private const val ZENITH_ESTIMATE_DISTINCT_ROLL_DEG = 1.5f

private const val FRAME_RGB_GAIN_MIN = 0.92f
private const val FRAME_RGB_GAIN_MAX = 1.08f
private const val FRAME_GAIN_ESTIMATE_X_STEP = 6
private const val FRAME_GAIN_ESTIMATE_Y_STEP = 3
private const val FRAME_GAIN_ESTIMATE_MIN_SAMPLES = 400

private data class RgbGain(
    val r: Float,
    val g: Float,
    val b: Float
)


data class FrameFootprint(
    val minAzimuthDeg: Float,
    val maxAzimuthDeg: Float,
    val minAltitudeDeg: Float,
    val maxAltitudeDeg: Float
)

data class ZenithPoseEstimate(
    val twistDeg: Float,
    val pitchOffsetDeg: Float,
    val rollOffsetDeg: Float,
    val score: Float,
    val comparedPixels: Int,
    val confidence: Float
)


object AtlasProjector {

    fun approximateFootprint(frame: FrameRecord): FrameFootprint {
        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val centerAz = AtlasMath.normalizeAzimuthDeg(frame.measuredAzimuthDeg)
        val centerAlt = frame.measuredPitchDeg.coerceIn(0f, 90f)
        val rollAbs = abs(frame.measuredRollDeg)

        val cosAlt = cos(Math.toRadians(centerAlt.toDouble())).toFloat().coerceAtLeast(0.08f)

        val azHalf = when {
            centerAlt >= 80f -> 180f
            centerAlt >= 60f -> (
                    hfov * 0.5f * (1f / cosAlt).coerceAtMost(2.2f) +
                            (rollAbs * 0.15f).coerceAtMost(6f)
                    ).coerceIn(hfov * 0.5f, 180f)
            centerAlt >= 40f -> (
                    hfov * 0.5f * (1f / cosAlt).coerceAtMost(1.6f) +
                            (rollAbs * 0.10f).coerceAtMost(4f)
                    ).coerceIn(hfov * 0.5f, 180f)
            else -> (
                    hfov * 0.5f +
                            (rollAbs * 0.05f).coerceAtMost(2f)
                    ).coerceIn(hfov * 0.5f, 180f)
        }

        val altHalf = (
                vfov * 0.5f +
                        when {
                            centerAlt >= 80f -> 6f
                            centerAlt >= 60f -> 3f
                            else -> 0f
                        } +
                        (rollAbs * 0.08f).coerceAtMost(3f)
                ).coerceIn(vfov * 0.5f, 90f)

        val fullAzimuth = centerAlt >= 80f || azHalf >= 179.5f

        val minAlt = (centerAlt - altHalf).coerceIn(0f, 90f)
        val maxAlt = (centerAlt + altHalf).coerceIn(0f, 90f)

        val constrainedMinAlt = when {
            frame.targetPitchDeg >= 80f || centerAlt >= 80f -> max(minAlt, 68f)
            else -> minAlt
        }

        return FrameFootprint(
            minAzimuthDeg = if (fullAzimuth) -180f else centerAz - azHalf,
            maxAzimuthDeg = if (fullAzimuth) 180f else centerAz + azHalf,
            minAltitudeDeg = constrainedMinAlt,
            maxAltitudeDeg = maxAlt
        )
    }

    fun projectSingleFrameToAtlas(frame: FrameRecord, atlas: SkyAtlas) {
        val src = BitmapFactory.decodeFile(frame.originalPath) ?: return
        try {
            val frameWeight = qualityWeightForFrame(frame)
            projectBitmapToAtlas(
                frame = frame,
                src = src,
                atlas = atlas,
                frameWeight = frameWeight,
                zenithTwistDegOverride = null,
                zenithPitchOffsetDegOverride = null,
                zenithRollOffsetDegOverride = null,
                emitLogs = true
            )
        } finally {
            src.recycle()
        }
    }

    fun projectFramesToAtlas(frames: List<FrameRecord>, atlas: SkyAtlas) {
        val ordered = frames.sortedBy { it.shotIndex }
        val nonZenithFrames = ordered.filterNot { isZenithFrame(it) }
        val zenithFrames = ordered.filter { isZenithFrame(it) }

        nonZenithFrames.forEach { frame ->
            val src = BitmapFactory.decodeFile(frame.originalPath) ?: run {
                Log.w("AtlasProjector", "No se pudo decodificar ${frame.originalPath}")
                return@forEach
            }

            try {
                val frameWeight = qualityWeightForFrame(frame)

                Log.d(
                    "AtlasProjector",
                    "frame=${frame.frameId} ring=${frame.ringId} " +
                            "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                            "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                            "weight=${"%.3f".format(frameWeight)}"
                )

                projectBitmapToAtlas(
                    frame = frame,
                    src = src,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    zenithTwistDegOverride = null,
                    zenithPitchOffsetDegOverride = null,
                    zenithRollOffsetDegOverride = null,
                    emitLogs = true
                )
            } finally {
                src.recycle()
            }
        }

        zenithFrames.forEach { frame ->
            val src = BitmapFactory.decodeFile(frame.originalPath) ?: run {
                Log.w("AtlasProjector", "No se pudo decodificar ${frame.originalPath}")
                return@forEach
            }

            try {
                val frameWeight = qualityWeightForFrame(frame)

                val zenithPose = estimateZenithPoseDeg(
                    frame = frame,
                    src = src,
                    baseAtlas = atlas,
                    frameWeight = frameWeight
                )

                Log.d(
                    "AtlasZenithPoseSeed",
                    "frame=${frame.frameId} " +
                            "twist=${"%.2f".format(zenithPose.twistDeg)} " +
                            "pitchOffset=${"%.2f".format(zenithPose.pitchOffsetDeg)} " +
                            "rollOffset=${"%.2f".format(zenithPose.rollOffsetDeg)} " +
                            "score=${"%.5f".format(zenithPose.score)} " +
                            "compared=${zenithPose.comparedPixels} " +
                            "confidence=${"%.3f".format(zenithPose.confidence)}"
                )

                try {
                    val refined = ZenithTopFaceRefiner.refineAndBlendZenithIntoAtlas(
                        atlas = atlas,
                        frame = frame,
                        srcBitmap = src,
                        frameWeight = frameWeight,
                        seedTwistDeg = zenithPose.twistDeg,
                        seedPitchOffsetDeg = zenithPose.pitchOffsetDeg,
                        seedRollOffsetDeg = zenithPose.rollOffsetDeg
                    )

                    if (refined == null) {
                        Log.w(
                            "AtlasZenithTopRefine",
                            "frame=${frame.frameId} no refinement result; fallback to ERP zenith"
                        )

                        projectBitmapToAtlas(
                            frame = frame,
                            src = src,
                            atlas = atlas,
                            frameWeight = frameWeight,
                            zenithTwistDegOverride = zenithPose.twistDeg,
                            zenithPitchOffsetDegOverride = zenithPose.pitchOffsetDeg,
                            zenithRollOffsetDegOverride = zenithPose.rollOffsetDeg,
                            emitLogs = true
                        )
                    } else {
                        try {
                            Log.d(
                                "AtlasZenithTopRefine",
                                "frame=${frame.frameId} " +
                                        "initialTwist=${"%.2f".format(refined.initialTwistDeg)} " +
                                        "finalTwist=${"%.2f".format(refined.finalTwistDeg)} " +
                                        "eccRot=${"%.2f".format(refined.eccRotationDeg)} " +
                                        "eccTx=${"%.2f".format(refined.eccTxPx)} " +
                                        "eccTy=${"%.2f".format(refined.eccTyPx)} " +
                                        "eccScore=${"%.5f".format(refined.eccScore)}"
                            )
                        } finally {
                            ZenithTopFaceRefiner.releaseTopFace(refined.alignedTopFace)
                        }
                    }
                } catch (t: Throwable) {
                    Log.e(
                        "AtlasZenithTopRefine",
                        "Fallo en refineAndBlendZenithIntoAtlas, usando fallback ERP",
                        t
                    )

                    projectBitmapToAtlas(
                        frame = frame,
                        src = src,
                        atlas = atlas,
                        frameWeight = frameWeight,
                        zenithTwistDegOverride = zenithPose.twistDeg,
                        zenithPitchOffsetDegOverride = zenithPose.pitchOffsetDeg,
                        zenithRollOffsetDegOverride = zenithPose.rollOffsetDeg,
                        emitLogs = true
                    )
                }
            } finally {
                src.recycle()
            }
        }
    }

    private fun projectBitmapToAtlas(
        frame: FrameRecord,
        src: Bitmap,
        atlas: SkyAtlas,
        frameWeight: Float,
        zenithTwistDegOverride: Float? = null,
        zenithPitchOffsetDegOverride: Float? = null,
        zenithRollOffsetDegOverride: Float? = null,
        emitLogs: Boolean = true,
        atlasXStep: Int = 1,
        atlasYStep: Int = 1
    ) {
        if (frameWeight <= 0f) return

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        val zenithLike = frame.targetPitchDeg >= 80f || frame.measuredPitchDeg >= 80f

        val effectiveZenithTwistDeg = zenithTwistDegOverride ?: ZENITH_TWIST_FALLBACK_DEG
        val effectiveZenithPitchOffsetDeg =
            zenithPitchOffsetDegOverride ?: ZENITH_PITCH_OFFSET_FALLBACK_DEG
        val effectiveZenithRollOffsetDeg =
            zenithRollOffsetDegOverride ?: ZENITH_ROLL_OFFSET_FALLBACK_DEG

        val projectionAzimuthDeg = if (zenithLike) {
            frame.measuredAzimuthDeg + effectiveZenithTwistDeg
        } else {
            frame.measuredAzimuthDeg
        }

        val projectionPitchDeg = if (zenithLike) {
            (frame.measuredPitchDeg + effectiveZenithPitchOffsetDeg).coerceIn(84f, 90f)
        } else {
            frame.measuredPitchDeg
        }

        val projectionRollDeg = if (zenithLike) {
            (frame.measuredRollDeg + effectiveZenithRollOffsetDeg).coerceIn(
                ZENITH_ROLL_OFFSET_MIN_DEG,
                ZENITH_ROLL_OFFSET_MAX_DEG
            )
        } else {
            frame.measuredRollDeg
        }

        val yawRad = Math.toRadians(
            AtlasMath.normalizeAzimuthDeg(projectionAzimuthDeg).toDouble()
        ).toFloat()

        val pitchRad = Math.toRadians(projectionPitchDeg.toDouble()).toFloat()
        val rollRad = Math.toRadians((-projectionRollDeg).toDouble()).toFloat()

        if (emitLogs) {
            Log.d(
                "AtlasPose",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "zenithLike=$zenithLike " +
                        "yawUsed=${"%.2f".format(projectionAzimuthDeg)} " +
                        "yawMeasured=${"%.2f".format(frame.measuredAzimuthDeg)} " +
                        "pitchUsed=${"%.2f".format(projectionPitchDeg)} " +
                        "pitchMeasured=${"%.2f".format(frame.measuredPitchDeg)} " +
                        "rollUsed=${"%.2f".format(projectionRollDeg)} " +
                        "rollMeasured=${"%.2f".format(frame.measuredRollDeg)} " +
                        "twist=${"%.2f".format(effectiveZenithTwistDeg)} " +
                        "pitchOffset=${"%.2f".format(effectiveZenithPitchOffsetDeg)} " +
                        "rollOffset=${"%.2f".format(effectiveZenithRollOffsetDeg)}"
            )
        }

        val forward = worldDirectionRad(yawRad, pitchRad)

        val right0: FloatArray
        val up0: FloatArray

        if (zenithLike && projectionPitchDeg >= 89.5f) {
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

            var u = cross(r, forward)
            u = normalize(u)

            right0 = r
            up0 = u
        }

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

        if (emitLogs) {
            Log.d(
                "AtlasPole",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "zenithLike=$zenithLike " +
                        "pitchUsed=${"%.2f".format(projectionPitchDeg)} " +
                        "rollUsed=${"%.2f".format(projectionRollDeg)} " +
                        "twist=${"%.2f".format(effectiveZenithTwistDeg)} " +
                        "pitchOffset=${"%.2f".format(effectiveZenithPitchOffsetDeg)} " +
                        "rollOffset=${"%.2f".format(effectiveZenithRollOffsetDeg)}"
            )
        }

        val fp = approximateFootprint(frame)
        val yTop = AtlasMath.altitudeToY(fp.maxAltitudeDeg, atlas.config)
        val yBottom = AtlasMath.altitudeToY(fp.minAltitudeDeg, atlas.config)

        val srcW = src.width
        val srcH = src.height
        val srcPixels = IntArray(srcW * srcH)
        src.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val coversAllAzimuth =
            frame.targetPitchDeg >= 80f ||
                    frame.measuredPitchDeg >= 80f ||
                    (fp.maxAzimuthDeg - fp.minAzimuthDeg) >= 359f

        val xSpans = if (coversAllAzimuth) {
            listOf(0 to (atlas.width - 1))
        } else {
            AtlasMath.splitAzimuthSpan(
                fp.minAzimuthDeg,
                fp.maxAzimuthDeg,
                atlas.config
            )
        }
        val frameRgbGain = estimateFrameRgbGain(
            atlas = atlas,
            frame = frame,
            srcPixels = srcPixels,
            srcW = srcW,
            srcH = srcH,
            forward = forward,
            right = right,
            up = up,
            tanHalfH = tanHalfH,
            tanHalfV = tanHalfV,
            xSpans = xSpans,
            yTop = yTop,
            yBottom = yBottom
        )

        if (emitLogs) {
            Log.d(
                "AtlasFrameGain",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "gainR=${"%.3f".format(frameRgbGain.r)} " +
                        "gainG=${"%.3f".format(frameRgbGain.g)} " +
                        "gainB=${"%.3f".format(frameRgbGain.b)}"
            )
        }
        if (emitLogs) {
            Log.d(
                "AtlasFootprint",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                        "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                        "fpAz=${"%.2f".format(fp.minAzimuthDeg)}..${"%.2f".format(fp.maxAzimuthDeg)} " +
                        "fpAlt=${"%.2f".format(fp.minAltitudeDeg)}..${"%.2f".format(fp.maxAltitudeDeg)} " +
                        "coversAll=$coversAllAzimuth spans=${xSpans.joinToString()}"
            )
        }

        val safeXStep = atlasXStep.coerceAtLeast(1)
        val safeYStep = atlasYStep.coerceAtLeast(1)

        for ((x0, x1) in xSpans) {
            for (y in yTop..yBottom step safeYStep) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

                for (x in x0..x1 step safeXStep) {
                    val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                    val azRad = Math.toRadians(azDeg.toDouble()).toFloat()

                    val dir = worldDirectionRad(azRad, altRad)

                    val camX = dot(dir, right)
                    val camY = dot(dir, up)
                    val camZ = dot(dir, forward)

                    if (camZ <= 0f) continue

                    val nx = (camX / camZ) / tanHalfH
                    val ny = (camY / camZ) / tanHalfV

                    if (abs(nx) > 1f || abs(ny) > 1f) continue

                    val u = (((nx + 1f) * 0.5f) * (srcW - 1)).coerceIn(0f, (srcW - 1).toFloat())
                    val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).coerceIn(0f, (srcH - 1).toFloat())

                    val sampled = bilinearSampleArgb(
                        pixels = srcPixels,
                        width = srcW,
                        height = srcH,
                        fx = u,
                        fy = v
                    )

                    val color = applyRgbGain(sampled, frameRgbGain)

                    val wx = (1f - abs(nx)).coerceIn(0f, 1f)
                    val wy = (1f - abs(ny)).coerceIn(0f, 1f)

                    val localWeight = max(
                        0.001f,
                        (wx * wx) * (wy * wy)
                    )

                    val finalWeight = frameWeight * localWeight
                    atlas.blendPixel(x, y, color, finalWeight)
                }
            }
        }
    }
    private fun isZenithFrame(frame: FrameRecord): Boolean {
        return frame.ringId.equals("Z0", ignoreCase = true) ||
                frame.targetPitchDeg >= 80f ||
                frame.measuredPitchDeg >= 80f
    }

    private fun estimateZenithPoseDeg(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float
    ): ZenithPoseEstimate {
        if (frameWeight <= 0f) {
            return ZenithPoseEstimate(
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG,
                score = Float.NEGATIVE_INFINITY,
                comparedPixels = 0,
                confidence = 0f
            )
        }

        val estimationSrc = downscaleForZenithEstimation(src)
        val startedAt = SystemClock.elapsedRealtime()
        val deadlineAt = startedAt + ZENITH_ESTIMATE_TIME_BUDGET_MS

        try {
            var best: ZenithPoseEstimate? = null
            val allCandidates = ArrayList<ZenithPoseEstimate>(ZENITH_ESTIMATE_MAX_CANDIDATES + 8)

            val visited = HashSet<String>()
            var evaluatedCount = 0
            var stoppedByBudget = false

            fun outOfBudget(): Boolean {
                return evaluatedCount >= ZENITH_ESTIMATE_MAX_CANDIDATES ||
                        SystemClock.elapsedRealtime() >= deadlineAt
            }

            fun considerCandidate(
                twistDeg: Float,
                pitchOffsetDeg: Float,
                rollOffsetDeg: Float
            ) {
                if (outOfBudget()) {
                    stoppedByBudget = true
                    return
                }

                val normTwist = normalizeTwistDeg(twistDeg)
                val normPitch = clampZenithPitchOffsetDeg(pitchOffsetDeg)
                val normRoll = clampZenithRollOffsetDeg(rollOffsetDeg)

                val key = buildString {
                    append((normTwist * 10f).roundToInt())
                    append(':')
                    append((normPitch * 10f).roundToInt())
                    append(':')
                    append((normRoll * 10f).roundToInt())
                }

                if (!visited.add(key)) return

                val candidate = evaluateZenithPoseCandidate(
                    frame = frame,
                    src = estimationSrc,
                    baseAtlas = baseAtlas,
                    frameWeight = frameWeight,
                    twistDeg = normTwist,
                    pitchOffsetDeg = normPitch,
                    rollOffsetDeg = normRoll
                )

                evaluatedCount++
                allCandidates.add(candidate)

                if (evaluatedCount % ZENITH_ESTIMATE_PROGRESS_EVERY == 0) {
                    Log.d(
                        "AtlasZenithPoseProgress",
                        "frame=${frame.frameId} evaluated=$evaluatedCount " +
                                "bestTwist=${"%.2f".format(best?.twistDeg ?: ZENITH_TWIST_FALLBACK_DEG)} " +
                                "bestPitchOffset=${"%.2f".format(best?.pitchOffsetDeg ?: 0f)} " +
                                "bestRollOffset=${"%.2f".format(best?.rollOffsetDeg ?: 0f)} " +
                                "bestScore=${"%.4f".format(best?.score ?: Float.NEGATIVE_INFINITY)}"
                    )
                }

                if (best == null || candidate.score > best!!.score) {
                    best = candidate
                }
            }

            val coarsePitchOffsets = floatArrayOf(-2f, 0f)
            val coarseRollOffsets = floatArrayOf(-4f, 0f, 4f)

            loop@ for (twist in 0 until 360 step 45) {
                for (pitchOffset in coarsePitchOffsets) {
                    for (rollOffset in coarseRollOffsets) {
                        considerCandidate(
                            twistDeg = twist.toFloat(),
                            pitchOffsetDeg = pitchOffset,
                            rollOffsetDeg = rollOffset
                        )
                        if (stoppedByBudget) break@loop
                    }
                }
            }

            val coarseBest = best ?: ZenithPoseEstimate(
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG,
                score = Float.NEGATIVE_INFINITY,
                comparedPixels = 0,
                confidence = 0f
            )

            if (!stoppedByBudget) {
                var twistMid = coarseBest.twistDeg - 10f
                loop@ while (twistMid <= coarseBest.twistDeg + 10f + 1e-3f) {
                    var pitchMid = coarseBest.pitchOffsetDeg - 1f
                    while (pitchMid <= coarseBest.pitchOffsetDeg + 1f + 1e-3f) {
                        var rollMid = coarseBest.rollOffsetDeg - 2f
                        while (rollMid <= coarseBest.rollOffsetDeg + 2f + 1e-3f) {
                            considerCandidate(
                                twistDeg = twistMid,
                                pitchOffsetDeg = pitchMid,
                                rollOffsetDeg = rollMid
                            )
                            if (stoppedByBudget) break@loop
                            rollMid += 2f
                        }
                        pitchMid += 1f
                    }
                    twistMid += 5f
                }
            }

            val finalBest = best ?: coarseBest

            val fallbackCandidate = evaluateZenithPoseCandidate(
                frame = frame,
                src = estimationSrc,
                baseAtlas = baseAtlas,
                frameWeight = frameWeight,
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG
            )

            val distinctRunnerUpScore = allCandidates
                .asSequence()
                .filter { isDistinctZenithPoseCandidate(it, finalBest) }
                .maxOfOrNull { it.score }
                ?: Float.NEGATIVE_INFINITY

            val confidence =
                if (distinctRunnerUpScore.isFinite()) {
                    finalBest.score - distinctRunnerUpScore
                } else {
                    0f
                }

            val improvementOverFallback = finalBest.score - fallbackCandidate.score

            val lowPixels = finalBest.comparedPixels < ZENITH_ESTIMATE_MIN_PIXELS
            val lowScore = !finalBest.score.isFinite() || finalBest.score < ZENITH_ESTIMATE_MIN_SCORE
            val lowConfidence = confidence < ZENITH_ESTIMATE_MIN_CONFIDENCE

            val acceptByAbsoluteGate = !lowPixels && !lowScore && !lowConfidence
            val acceptByRelativeGain = !lowPixels &&
                    improvementOverFallback >= ZENITH_ESTIMATE_MIN_IMPROVEMENT_OVER_FALLBACK

            Log.d(
                "AtlasZenithPoseSummary",
                "frame=${frame.frameId} evaluated=$evaluatedCount " +
                        "stoppedByBudget=$stoppedByBudget " +
                        "bestTwist=${"%.2f".format(finalBest.twistDeg)} " +
                        "bestPitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                        "bestRollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                        "score=${"%.4f".format(finalBest.score)} " +
                        "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                        "improvement=${"%.4f".format(improvementOverFallback)} " +
                        "pixels=${finalBest.comparedPixels} " +
                        "confidence=${"%.4f".format(confidence)}"
            )

            if (acceptByAbsoluteGate || acceptByRelativeGain) {
                Log.d(
                    "AtlasZenithPose",
                    "accept frame=${frame.frameId} " +
                            "twist=${"%.2f".format(finalBest.twistDeg)} " +
                            "pitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                            "rollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                            "score=${"%.4f".format(finalBest.score)} " +
                            "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                            "improvement=${"%.4f".format(improvementOverFallback)} " +
                            "confidence=${"%.4f".format(confidence)} " +
                            "acceptByAbsoluteGate=$acceptByAbsoluteGate " +
                            "acceptByRelativeGain=$acceptByRelativeGain"
                )
                return finalBest.copy(confidence = confidence)
            }

            Log.w(
                "AtlasZenithPose",
                "fallback frame=${frame.frameId} " +
                        "bestTwist=${"%.2f".format(finalBest.twistDeg)} " +
                        "bestPitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                        "bestRollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                        "score=${"%.4f".format(finalBest.score)} " +
                        "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                        "improvement=${"%.4f".format(improvementOverFallback)} " +
                        "pixels=${finalBest.comparedPixels} " +
                        "confidence=${"%.4f".format(confidence)} " +
                        "lowPixels=$lowPixels lowScore=$lowScore lowConfidence=$lowConfidence"
            )

            return ZenithPoseEstimate(
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG,
                score = fallbackCandidate.score,
                comparedPixels = fallbackCandidate.comparedPixels,
                confidence = confidence
            )
        } finally {
            if (estimationSrc !== src) {
                estimationSrc.recycle()
            }
        }
    }

    private fun evaluateZenithPoseCandidate(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float,
        twistDeg: Float,
        pitchOffsetDeg: Float,
        rollOffsetDeg: Float
    ): ZenithPoseEstimate {
        val candidateAtlas = SkyAtlas(baseAtlas.config)

        projectBitmapToAtlas(
            frame = frame,
            src = src,
            atlas = candidateAtlas,
            frameWeight = frameWeight,
            zenithTwistDegOverride = twistDeg,
            zenithPitchOffsetDegOverride = pitchOffsetDeg,
            zenithRollOffsetDegOverride = rollOffsetDeg,
            emitLogs = false,
            atlasXStep = ZENITH_ESTIMATE_PROJ_X_STEP,
            atlasYStep = ZENITH_ESTIMATE_PROJ_Y_STEP
        )

        val (score, comparedPixels) = scoreZenithOverlap(
            baseAtlas = baseAtlas,
            candidateAtlas = candidateAtlas,
            minAltitudeDeg = ZENITH_OVERLAP_MIN_ALT_DEG,
            maxAltitudeDeg = ZENITH_OVERLAP_MAX_ALT_DEG,
            xStep = ZENITH_SCORE_X_STEP,
            yStep = ZENITH_SCORE_Y_STEP
        )

        return ZenithPoseEstimate(
            twistDeg = normalizeTwistDeg(twistDeg),
            pitchOffsetDeg = clampZenithPitchOffsetDeg(pitchOffsetDeg),
            rollOffsetDeg = clampZenithRollOffsetDeg(rollOffsetDeg),
            score = score,
            comparedPixels = comparedPixels,
            confidence = 0f
        )
    }
    private fun scoreZenithOverlap(
        baseAtlas: SkyAtlas,
        candidateAtlas: SkyAtlas,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        xStep: Int = 1,
        yStep: Int = 1
    ): Pair<Float, Int> {
        val yTop = AtlasMath.altitudeToY(maxAltitudeDeg, baseAtlas.config)
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, baseAtlas.config)

        val safeXStep = xStep.coerceAtLeast(1)
        val safeYStep = yStep.coerceAtLeast(1)

        var sumW = 0.0
        var sumLumA = 0.0
        var sumLumB = 0.0
        var sumGradA = 0.0
        var sumGradB = 0.0
        var count = 0

        for (y in yTop..yBottom step safeYStep) {
            for (x in 0 until baseAtlas.width step safeXStep) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val idxA = baseAtlas.index(x, y)
                val idxB = candidateAtlas.index(x, y)

                val colorA = baseAtlas.pixels[idxA]
                val colorB = candidateAtlas.pixels[idxB]

                val lumA = luma(colorA)
                val lumB = luma(colorB)
                val gradA = gradientMag(baseAtlas, x, y)
                val gradB = gradientMag(candidateAtlas, x, y)

                if (!isInformativeZenithSample(lumA, lumB, gradA, gradB)) continue

                val w = zenithSampleWeight(
                    baseWeight = baseAtlas.weightAt(x, y).toDouble(),
                    candidateWeight = candidateAtlas.weightAt(x, y).toDouble(),
                    gradA = gradA,
                    gradB = gradB
                )

                sumW += w
                sumLumA += w * lumA
                sumLumB += w * lumB
                sumGradA += w * gradA
                sumGradB += w * gradB
                count++
            }
        }

        if (count < ZENITH_ESTIMATE_MIN_PIXELS || sumW <= 0.0) {
            return Float.NEGATIVE_INFINITY to count
        }

        val meanLumA = sumLumA / sumW
        val meanLumB = sumLumB / sumW
        val meanGradA = sumGradA / sumW
        val meanGradB = sumGradB / sumW

        var numLum = 0.0
        var denLumA = 0.0
        var denLumB = 0.0

        var numGrad = 0.0
        var denGradA = 0.0
        var denGradB = 0.0

        for (y in yTop..yBottom step safeYStep) {
            for (x in 0 until baseAtlas.width step safeXStep) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val idxA = baseAtlas.index(x, y)
                val idxB = candidateAtlas.index(x, y)

                val colorA = baseAtlas.pixels[idxA]
                val colorB = candidateAtlas.pixels[idxB]

                val lumA = luma(colorA)
                val lumB = luma(colorB)
                val gradA = gradientMag(baseAtlas, x, y)
                val gradB = gradientMag(candidateAtlas, x, y)

                if (!isInformativeZenithSample(lumA, lumB, gradA, gradB)) continue

                val w = zenithSampleWeight(
                    baseWeight = baseAtlas.weightAt(x, y).toDouble(),
                    candidateWeight = candidateAtlas.weightAt(x, y).toDouble(),
                    gradA = gradA,
                    gradB = gradB
                )

                val daLum = lumA - meanLumA
                val dbLum = lumB - meanLumB
                numLum += w * daLum * dbLum
                denLumA += w * daLum * daLum
                denLumB += w * dbLum * dbLum

                val daGrad = gradA - meanGradA
                val dbGrad = gradB - meanGradB
                numGrad += w * daGrad * dbGrad
                denGradA += w * daGrad * daGrad
                denGradB += w * dbGrad * dbGrad
            }
        }

        val lumScore =
            if (denLumA > 1e-9 && denLumB > 1e-9) {
                (numLum / sqrt(denLumA * denLumB)).toFloat()
            } else {
                -1f
            }

        val gradScore =
            if (denGradA > 1e-9 && denGradB > 1e-9) {
                (numGrad / sqrt(denGradA * denGradB)).toFloat()
            } else {
                -1f
            }

        val finalScore = 0.20f * lumScore + 0.80f * gradScore
        return finalScore to count
    }

    private fun isInformativeZenithSample(
        lumA: Double,
        lumB: Double,
        gradA: Double,
        gradB: Double
    ): Boolean {
        if (lumA <= ZENITH_ESTIMATE_MIN_LUMA || lumA >= ZENITH_ESTIMATE_MAX_LUMA) return false
        if (lumB <= ZENITH_ESTIMATE_MIN_LUMA || lumB >= ZENITH_ESTIMATE_MAX_LUMA) return false

        val maxGrad = if (gradA > gradB) gradA else gradB
        return maxGrad >= ZENITH_ESTIMATE_MIN_GRADIENT
    }

    private fun zenithSampleWeight(
        baseWeight: Double,
        candidateWeight: Double,
        gradA: Double,
        gradB: Double
    ): Double {
        val minCoverage =
            if (baseWeight < candidateWeight) baseWeight else candidateWeight

        val maxGrad =
            if (gradA > gradB) gradA else gradB

        val structureBoost = 1.0 + (maxGrad * 0.02).coerceIn(0.0, 6.0)
        val coverageBoost = 1.0 + (minCoverage * 0.15).coerceIn(0.0, 1.5)

        return structureBoost * coverageBoost
    }

    private fun downscaleForZenithEstimation(src: Bitmap): Bitmap {
        val longSide = max(src.width, src.height)
        if (longSide <= ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE) return src

        val scale = ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE.toFloat() / longSide.toFloat()
        val dstW = (src.width * scale).roundToInt().coerceAtLeast(1)
        val dstH = (src.height * scale).roundToInt().coerceAtLeast(1)

        return Bitmap.createScaledBitmap(src, dstW, dstH, true)
    }

    private fun normalizeTwistDeg(value: Float): Float {
        var v = value % 360f
        if (v < 0f) v += 360f
        return v
    }
    private fun isDistinctZenithPoseCandidate(
        a: ZenithPoseEstimate,
        b: ZenithPoseEstimate
    ): Boolean {
        val twistDiff = angleDiffDeg(a.twistDeg, b.twistDeg)
        val pitchDiff = abs(a.pitchOffsetDeg - b.pitchOffsetDeg)
        val rollDiff = abs(a.rollOffsetDeg - b.rollOffsetDeg)

        return twistDiff >= ZENITH_ESTIMATE_DISTINCT_TWIST_DEG ||
                pitchDiff >= ZENITH_ESTIMATE_DISTINCT_PITCH_DEG ||
                rollDiff >= ZENITH_ESTIMATE_DISTINCT_ROLL_DEG
    }
    private fun luma(color: Int): Double {
        return 0.299 * Color.red(color) +
                0.587 * Color.green(color) +
                0.114 * Color.blue(color)
    }

    private fun clampZenithPitchOffsetDeg(value: Float): Float {
        return value.coerceIn(
            ZENITH_PITCH_OFFSET_MIN_DEG,
            ZENITH_PITCH_OFFSET_MAX_DEG
        )
    }

    private fun clampZenithRollOffsetDeg(value: Float): Float {
        return value.coerceIn(
            ZENITH_ROLL_OFFSET_MIN_DEG,
            ZENITH_ROLL_OFFSET_MAX_DEG
        )
    }

    private fun gradientMag(atlas: SkyAtlas, x: Int, y: Int): Double {
        val xl = if (x > 0) x - 1 else x
        val xr = if (x < atlas.width - 1) x + 1 else x
        val yt = if (y > 0) y - 1 else y
        val yb = if (y < atlas.height - 1) y + 1 else y

        val gx = luma(atlas.pixels[atlas.index(xr, y)]) - luma(atlas.pixels[atlas.index(xl, y)])
        val gy = luma(atlas.pixels[atlas.index(x, yb)]) - luma(atlas.pixels[atlas.index(x, yt)])

        return kotlin.math.abs(gx) + kotlin.math.abs(gy)
    }
    private fun estimateFrameRgbGain(
        atlas: SkyAtlas,
        frame: FrameRecord,
        srcPixels: IntArray,
        srcW: Int,
        srcH: Int,
        forward: FloatArray,
        right: FloatArray,
        up: FloatArray,
        tanHalfH: Float,
        tanHalfV: Float,
        xSpans: List<Pair<Int, Int>>,
        yTop: Int,
        yBottom: Int
    ): RgbGain {
        var refRSum = 0f
        var refGSum = 0f
        var refBSum = 0f

        var movRSum = 0f
        var movGSum = 0f
        var movBSum = 0f

        var count = 0

        for ((x0, x1) in xSpans) {
            for (y in yTop..yBottom step FRAME_GAIN_ESTIMATE_Y_STEP) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

                for (x in x0..x1 step FRAME_GAIN_ESTIMATE_X_STEP) {
                    if (!atlas.hasCoverageAt(x, y)) continue

                    val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                    val azRad = Math.toRadians(azDeg.toDouble()).toFloat()

                    val dir = worldDirectionRad(azRad, altRad)

                    val camX = dot(dir, right)
                    val camY = dot(dir, up)
                    val camZ = dot(dir, forward)

                    if (camZ <= 0f) continue

                    val nx = (camX / camZ) / tanHalfH
                    val ny = (camY / camZ) / tanHalfV

                    if (abs(nx) > 1f || abs(ny) > 1f) continue

                    val u = (((nx + 1f) * 0.5f) * (srcW - 1)).coerceIn(0f, (srcW - 1).toFloat())
                    val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).coerceIn(0f, (srcH - 1).toFloat())

                    val movColor = bilinearSampleArgb(
                        pixels = srcPixels,
                        width = srcW,
                        height = srcH,
                        fx = u,
                        fy = v
                    )

                    val refColor = atlas.pixels[atlas.index(x, y)]

                    refRSum += Color.red(refColor)
                    refGSum += Color.green(refColor)
                    refBSum += Color.blue(refColor)

                    movRSum += Color.red(movColor)
                    movGSum += Color.green(movColor)
                    movBSum += Color.blue(movColor)

                    count++
                }
            }
        }

        if (count < FRAME_GAIN_ESTIMATE_MIN_SAMPLES) {
            return RgbGain(1f, 1f, 1f)
        }

        val rGain = if (movRSum > 1e-3f) {
            (refRSum / movRSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        val gGain = if (movGSum > 1e-3f) {
            (refGSum / movGSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        val bGain = if (movBSum > 1e-3f) {
            (refBSum / movBSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        return RgbGain(rGain, gGain, bGain)
    }

    private fun bilinearSampleArgb(
        pixels: IntArray,
        width: Int,
        height: Int,
        fx: Float,
        fy: Float
    ): Int {
        val x0 = fx.toInt().coerceIn(0, width - 1)
        val y0 = fy.toInt().coerceIn(0, height - 1)
        val x1 = (x0 + 1).coerceIn(0, width - 1)
        val y1 = (y0 + 1).coerceIn(0, height - 1)

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val c00 = pixels[y0 * width + x0]
        val c10 = pixels[y0 * width + x1]
        val c01 = pixels[y1 * width + x0]
        val c11 = pixels[y1 * width + x1]

        val a = bilerp(
            Color.alpha(c00).toFloat(),
            Color.alpha(c10).toFloat(),
            Color.alpha(c01).toFloat(),
            Color.alpha(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val r = bilerp(
            Color.red(c00).toFloat(),
            Color.red(c10).toFloat(),
            Color.red(c01).toFloat(),
            Color.red(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val g = bilerp(
            Color.green(c00).toFloat(),
            Color.green(c10).toFloat(),
            Color.green(c01).toFloat(),
            Color.green(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val b = bilerp(
            Color.blue(c00).toFloat(),
            Color.blue(c10).toFloat(),
            Color.blue(c01).toFloat(),
            Color.blue(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        return Color.argb(a, r, g, b)
    }

    private fun bilerp(
        v00: Float,
        v10: Float,
        v01: Float,
        v11: Float,
        dx: Float,
        dy: Float
    ): Float {
        val top = v00 * (1f - dx) + v10 * dx
        val bottom = v01 * (1f - dx) + v11 * dx
        return top * (1f - dy) + bottom * dy
    }

    private fun applyRgbGain(color: Int, gain: RgbGain): Int {
        val a = Color.alpha(color)
        val r = (Color.red(color) * gain.r).roundToInt().coerceIn(0, 255)
        val g = (Color.green(color) * gain.g).roundToInt().coerceIn(0, 255)
        val b = (Color.blue(color) * gain.b).roundToInt().coerceIn(0, 255)
        return Color.argb(a, r, g, b)
    }
    private fun qualityWeightForFrame(frame: FrameRecord): Float {
        val zenithLike = frame.targetPitchDeg >= 80f || frame.measuredPitchDeg >= 80f

        val azErr = if (zenithLike) 0f
        else angleDiffDeg(frame.targetAzimuthDeg, frame.measuredAzimuthDeg)

        val pitchErr = abs(frame.targetPitchDeg - frame.measuredPitchDeg)
        val rollAbs = abs(frame.measuredRollDeg)

        val maxAzErr = when {
            zenithLike -> Float.MAX_VALUE
            frame.targetPitchDeg >= 40f -> 4.5f
            else -> 4.0f
        }
        val maxPitchErr = if (zenithLike) 2.0f else 2.75f
        val maxRollErr = if (zenithLike) 4.5f else 3.5f

        if (!zenithLike && azErr > maxAzErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=azErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "azErr=${"%.2f".format(azErr)} max=${"%.2f".format(maxAzErr)}"
            )
            return 0f
        }

        if (pitchErr > maxPitchErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=pitchErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "pitchErr=${"%.2f".format(pitchErr)} max=${"%.2f".format(maxPitchErr)}"
            )
            return 0f
        }

        if (rollAbs > maxRollErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=rollErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "roll=${"%.2f".format(rollAbs)} max=${"%.2f".format(maxRollErr)}"
            )
            return 0f
        }

        val azScore = if (zenithLike) 1f
        else (1f - azErr / maxAzErr).coerceIn(0f, 1f)

        val pitchScore = (1f - pitchErr / maxPitchErr).coerceIn(0f, 1f)
        val rollScore = (1f - rollAbs / maxRollErr).coerceIn(0f, 1f)

        val weight = 0.15f + 0.85f * azScore * pitchScore * rollScore

        Log.d(
            "AtlasWeight",
            "accept frame=${frame.frameId} ring=${frame.ringId} " +
                    "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                    "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                    "azErr=${"%.2f".format(azErr)} pitchErr=${"%.2f".format(pitchErr)} roll=${"%.2f".format(rollAbs)} " +
                    "weight=${"%.3f".format(weight)}"
        )

        return weight
    }

    private fun angleDiffDeg(a: Float, b: Float): Float {
        var d = AtlasMath.normalizeAzimuthDeg(a) - AtlasMath.normalizeAzimuthDeg(b)
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return abs(d)
    }

    private fun worldDirectionRad(azimuthRad: Float, altitudeRad: Float): FloatArray {
        val cosAlt = cos(altitudeRad)
        val xEast = cosAlt * sin(azimuthRad)
        val yNorth = cosAlt * cos(azimuthRad)
        val zUp = sin(altitudeRad)
        return floatArrayOf(xEast, yNorth, zUp)
    }

    private fun dot(a: FloatArray, b: FloatArray): Float =
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun cross(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        )

    private fun scale(v: FloatArray, s: Float): FloatArray =
        floatArrayOf(v[0] * s, v[1] * s, v[2] * s)

    private fun add(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])

    private fun length(v: FloatArray): Float =
        sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    private fun normalize(v: FloatArray): FloatArray {
        val len = length(v)
        if (len < 1e-6f) return floatArrayOf(0f, 0f, 0f)
        return floatArrayOf(v[0] / len, v[1] / len, v[2] / len)
    }
}