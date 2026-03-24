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
private const val ZENITH_TWIST_FALLBACK_DEG = 90f
private const val ZENITH_OVERLAP_MIN_ALT_DEG = 68f
private const val ZENITH_OVERLAP_MAX_ALT_DEG = 80f
private const val ZENITH_ESTIMATE_MIN_PIXELS = 2500
private const val ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE = 1280

data class FrameFootprint(
    val minAzimuthDeg: Float,
    val maxAzimuthDeg: Float,
    val minAltitudeDeg: Float,
    val maxAltitudeDeg: Float
)

data class ZenithTwistEstimate(
    val twistDeg: Float,
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

                val twistEstimate = estimateZenithTwistDeg(
                    frame = frame,
                    src = src,
                    baseAtlas = atlas,
                    frameWeight = frameWeight
                )

                Log.d(
                    "AtlasZenithTwist",
                    "frame=${frame.frameId} ring=${frame.ringId} " +
                            "twist=${"%.2f".format(twistEstimate.twistDeg)} " +
                            "score=${"%.4f".format(twistEstimate.score)} " +
                            "pixels=${twistEstimate.comparedPixels} " +
                            "confidence=${"%.4f".format(twistEstimate.confidence)}"
                )

                projectBitmapToAtlas(
                    frame = frame,
                    src = src,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    zenithTwistDegOverride = twistEstimate.twistDeg,
                    emitLogs = true
                )
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
        emitLogs: Boolean = true
    ) {
        if (frameWeight <= 0f) return

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        val zenithLike = frame.targetPitchDeg >= 80f || frame.measuredPitchDeg >= 80f

        val projectionAzimuthDeg = if (zenithLike) {
            frame.targetAzimuthDeg
        } else {
            frame.measuredAzimuthDeg
        }

        val projectionPitchDeg = if (zenithLike) {
            90f
        } else {
            frame.measuredPitchDeg
        }

        val projectionRollDeg = if (zenithLike) {
            0f
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
                        "rollMeasured=${"%.2f".format(frame.measuredRollDeg)}"
            )
        }

        val forward = worldDirectionRad(yawRad, pitchRad)

        val right0: FloatArray
        val up0: FloatArray

        val effectiveZenithTwistDeg = zenithTwistDegOverride ?: ZENITH_TWIST_FALLBACK_DEG

        if (zenithLike && projectionPitchDeg >= 89.5f) {
            val twistRad = Math.toRadians(effectiveZenithTwistDeg.toDouble()).toFloat()

            right0 = normalize(
                floatArrayOf(
                    cos(twistRad),
                    sin(twistRad),
                    0f
                )
            )

            up0 = normalize(
                floatArrayOf(
                    -sin(twistRad),
                    cos(twistRad),
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
                        "twist=${"%.2f".format(effectiveZenithTwistDeg)}"
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

        for ((x0, x1) in xSpans) {
            for (y in yTop..yBottom) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

                for (x in x0..x1) {
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

                    val u = (((nx + 1f) * 0.5f) * (srcW - 1))
                        .roundToInt()
                        .coerceIn(0, srcW - 1)

                    val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1))
                        .roundToInt()
                        .coerceIn(0, srcH - 1)

                    val color = srcPixels[v * srcW + u]

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

    private fun estimateZenithTwistDeg(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float
    ): ZenithTwistEstimate {
        if (frameWeight <= 0f) {
            return ZenithTwistEstimate(
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                score = Float.NEGATIVE_INFINITY,
                comparedPixels = 0,
                confidence = 0f
            )
        }

        val estimationSrc = downscaleForZenithEstimation(src)

        try {
            var best: ZenithTwistEstimate? = null
            var secondBestScore = Float.NEGATIVE_INFINITY
            val visited = HashSet<Int>()

            fun considerCandidate(twistDeg: Float) {
                val normalized = normalizeTwistDeg(twistDeg)
                val key = (normalized * 100f).roundToInt()
                if (!visited.add(key)) return

                val candidate = evaluateZenithTwistCandidate(
                    frame = frame,
                    src = estimationSrc,
                    baseAtlas = baseAtlas,
                    frameWeight = frameWeight,
                    twistDeg = normalized
                )

                if (best == null || candidate.score > best!!.score) {
                    if (best != null) secondBestScore = best!!.score
                    best = candidate
                } else if (candidate.score > secondBestScore) {
                    secondBestScore = candidate.score
                }
            }

            for (deg in 0 until 360 step 10) {
                considerCandidate(deg.toFloat())
            }

            val coarseBest = best ?: ZenithTwistEstimate(
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                score = Float.NEGATIVE_INFINITY,
                comparedPixels = 0,
                confidence = 0f
            )

            var mid = coarseBest.twistDeg - 12f
            while (mid <= coarseBest.twistDeg + 12f + 1e-3f) {
                considerCandidate(mid)
                mid += 2f
            }

            val midBest = best ?: coarseBest

            var fine = midBest.twistDeg - 2f
            while (fine <= midBest.twistDeg + 2f + 1e-3f) {
                considerCandidate(fine)
                fine += 0.5f
            }

            val finalBest = best ?: coarseBest
            val confidence =
                if (secondBestScore.isFinite()) finalBest.score - secondBestScore
                else 0f

            if (finalBest.comparedPixels < ZENITH_ESTIMATE_MIN_PIXELS || !finalBest.score.isFinite()) {
                Log.w(
                    "AtlasZenithTwist",
                    "fallback twist frame=${frame.frameId} " +
                            "pixels=${finalBest.comparedPixels} score=${finalBest.score}"
                )
                return ZenithTwistEstimate(
                    twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                    score = finalBest.score,
                    comparedPixels = finalBest.comparedPixels,
                    confidence = 0f
                )
            }

            return finalBest.copy(confidence = confidence)
        } finally {
            if (estimationSrc !== src) {
                estimationSrc.recycle()
            }
        }
    }

    private fun evaluateZenithTwistCandidate(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float,
        twistDeg: Float
    ): ZenithTwistEstimate {
        val candidateAtlas = SkyAtlas(baseAtlas.config)

        projectBitmapToAtlas(
            frame = frame,
            src = src,
            atlas = candidateAtlas,
            frameWeight = frameWeight,
            zenithTwistDegOverride = twistDeg,
            emitLogs = false
        )

        val (score, comparedPixels) = scoreZenithOverlap(
            baseAtlas = baseAtlas,
            candidateAtlas = candidateAtlas,
            minAltitudeDeg = ZENITH_OVERLAP_MIN_ALT_DEG,
            maxAltitudeDeg = ZENITH_OVERLAP_MAX_ALT_DEG
        )

        return ZenithTwistEstimate(
            twistDeg = normalizeTwistDeg(twistDeg),
            score = score,
            comparedPixels = comparedPixels,
            confidence = 0f
        )
    }

    private fun scoreZenithOverlap(
        baseAtlas: SkyAtlas,
        candidateAtlas: SkyAtlas,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float
    ): Pair<Float, Int> {
        val yTop = AtlasMath.altitudeToY(maxAltitudeDeg, baseAtlas.config)
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, baseAtlas.config)

        var sumW = 0.0
        var sumLumA = 0.0
        var sumLumB = 0.0
        var sumGradA = 0.0
        var sumGradB = 0.0
        var count = 0

        for (y in yTop..yBottom) {
            for (x in 0 until baseAtlas.width) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val a = luma(baseAtlas.pixels[baseAtlas.index(x, y)])
                val b = luma(candidateAtlas.pixels[candidateAtlas.index(x, y)])
                val ga = gradientMag(baseAtlas, x, y)
                val gb = gradientMag(candidateAtlas, x, y)

                val w = (1f + ((ga + gb) * 0.01f).coerceIn(0f, 4f)).toDouble()

                sumW += w
                sumLumA += w * a
                sumLumB += w * b
                sumGradA += w * ga
                sumGradB += w * gb
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

        for (y in yTop..yBottom) {
            for (x in 0 until baseAtlas.width) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val a = luma(baseAtlas.pixels[baseAtlas.index(x, y)])
                val b = luma(candidateAtlas.pixels[candidateAtlas.index(x, y)])
                val ga = gradientMag(baseAtlas, x, y)
                val gb = gradientMag(candidateAtlas, x, y)

                val w = (1f + ((ga + gb) * 0.01f).coerceIn(0f, 4f)).toDouble()

                val daLum = a - meanLumA
                val dbLum = b - meanLumB

                numLum += w * daLum * dbLum
                denLumA += w * daLum * daLum
                denLumB += w * dbLum * dbLum

                val daGrad = ga - meanGradA
                val dbGrad = gb - meanGradB

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

        val finalScore = 0.35f * lumScore + 0.65f * gradScore
        return finalScore to count
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

    private fun luma(color: Int): Double {
        return 0.299 * Color.red(color) +
                0.587 * Color.green(color) +
                0.114 * Color.blue(color)
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