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

data class FrameFootprint(
    val minAzimuthDeg: Float,
    val maxAzimuthDeg: Float,
    val minAltitudeDeg: Float,
    val maxAltitudeDeg: Float
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
            centerAlt >= 60f -> (hfov * 0.5f * (1f / cosAlt).coerceAtMost(2.2f) + (rollAbs * 0.15f).coerceAtMost(6f))
                .coerceIn(hfov * 0.5f, 180f)
            centerAlt >= 40f -> (hfov * 0.5f * (1f / cosAlt).coerceAtMost(1.6f) + (rollAbs * 0.10f).coerceAtMost(4f))
                .coerceIn(hfov * 0.5f, 180f)
            else -> (hfov * 0.5f + (rollAbs * 0.05f).coerceAtMost(2f))
                .coerceIn(hfov * 0.5f, 180f)
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

        return FrameFootprint(
            minAzimuthDeg = if (fullAzimuth) -180f else centerAz - azHalf,
            maxAzimuthDeg = if (fullAzimuth) 180f else centerAz + azHalf,
            minAltitudeDeg = (centerAlt - altHalf).coerceIn(0f, 90f),
            maxAltitudeDeg = (centerAlt + altHalf).coerceIn(0f, 90f)
        )
    }

    fun projectSingleFrameToAtlas(frame: FrameRecord, atlas: SkyAtlas) {
        val src = BitmapFactory.decodeFile(frame.originalPath) ?: return
        try {
            val frameWeight = qualityWeightForFrame(frame)
            projectBitmapToAtlas(frame, src, atlas, frameWeight)
        } finally {
            src.recycle()
        }
    }

    fun projectFramesToAtlas(frames: List<FrameRecord>, atlas: SkyAtlas) {
        frames.sortedBy { it.shotIndex }.forEach { frame ->
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

                projectBitmapToAtlas(frame, src, atlas, frameWeight)
            } finally {
                src.recycle()
            }
        }
    }

    private fun projectBitmapToAtlas(
        frame: FrameRecord,
        src: Bitmap,
        atlas: SkyAtlas,
        frameWeight: Float
    ) {
        if (frameWeight <= 0f) return

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        val yawRad = Math.toRadians(
            AtlasMath.normalizeAzimuthDeg(frame.measuredAzimuthDeg).toDouble()
        ).toFloat()

        val pitchRad = Math.toRadians(frame.measuredPitchDeg.toDouble()).toFloat()

        // Mantengo la misma convención de la fase 2
        val rollRad = Math.toRadians((-frame.measuredRollDeg).toDouble()).toFloat()

        val forward = worldDirectionRad(yawRad, pitchRad)

        var right0 = cross(forward, floatArrayOf(0f, 0f, 1f))
        if (length(right0) < 1e-4f) {
            right0 = floatArrayOf(1f, 0f, 0f)
        }
        right0 = normalize(right0)

        var up0 = cross(right0, forward)
        up0 = normalize(up0)

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

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

        Log.d(
            "AtlasFootprint",
            "frame=${frame.frameId} ring=${frame.ringId} " +
                    "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                    "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                    "fpAz=${"%.2f".format(fp.minAzimuthDeg)}..${"%.2f".format(fp.maxAzimuthDeg)} " +
                    "fpAlt=${"%.2f".format(fp.minAltitudeDeg)}..${"%.2f".format(fp.maxAltitudeDeg)} " +
                    "coversAll=$coversAllAzimuth spans=${xSpans.joinToString()}"
        )

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