package com.example.sunproject.domain.atlas

import android.graphics.BitmapFactory
import com.example.sunproject.data.model.FrameRecord
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

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

        return FrameFootprint(
            minAzimuthDeg = centerAz - hfov / 2f,
            maxAzimuthDeg = centerAz + hfov / 2f,
            minAltitudeDeg = (centerAlt - vfov / 2f).coerceIn(0f, 90f),
            maxAltitudeDeg = (centerAlt + vfov / 2f).coerceIn(0f, 90f)
        )
    }

    fun projectSingleFrameToAtlas(frame: FrameRecord, atlas: SkyAtlas) {
        val src = BitmapFactory.decodeFile(frame.originalPath) ?: return

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        val yawRad = Math.toRadians(
            AtlasMath.normalizeAzimuthDeg(frame.measuredAzimuthDeg).toDouble()
        ).toFloat()

        val pitchRad = Math.toRadians(frame.measuredPitchDeg.toDouble()).toFloat()

        // En esta primera versión conviene invertir el signo para compensar la forma
        // en que ya estás nivelando el overlay visual con canvas.rotate(-cameraRoll).
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

        for (y in 0 until atlas.height) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

            for (x in 0 until atlas.width) {
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

                val u = (((nx + 1f) * 0.5f) * (src.width - 1))
                    .roundToInt()
                    .coerceIn(0, src.width - 1)

                val v = ((1f - ((ny + 1f) * 0.5f)) * (src.height - 1))
                    .roundToInt()
                    .coerceIn(0, src.height - 1)

                val color = src.getPixel(u, v)

                val centerWeight = max(
                    0.001f,
                    (1f - abs(nx)).coerceIn(0f, 1f) *
                            (1f - abs(ny)).coerceIn(0f, 1f)
                )

                atlas.blendPixel(x, y, color, centerWeight)
            }
        }
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