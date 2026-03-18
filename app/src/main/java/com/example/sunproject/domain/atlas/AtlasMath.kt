package com.example.sunproject.domain.atlas

import kotlin.math.roundToInt

object AtlasMath {

    fun normalizeAzimuthDeg(azimuthDeg: Float): Float {
        var az = azimuthDeg % 360f
        if (az < -180f) az += 360f
        if (az >= 180f) az -= 360f
        return az
    }

    fun azimuthToX(azimuthDeg: Float, cfg: AtlasConfig): Int {
        val az = normalizeAzimuthDeg(azimuthDeg)
        val t = (az - cfg.minAzimuthDeg) / (cfg.maxAzimuthDeg - cfg.minAzimuthDeg)
        return (t * (cfg.widthPx - 1)).roundToInt().coerceIn(0, cfg.widthPx - 1)
    }

    fun altitudeToY(altitudeDeg: Float, cfg: AtlasConfig): Int {
        val alt = altitudeDeg.coerceIn(cfg.minAltitudeDeg, cfg.maxAltitudeDeg)
        val t = (cfg.maxAltitudeDeg - alt) / (cfg.maxAltitudeDeg - cfg.minAltitudeDeg)
        return (t * (cfg.heightPx - 1)).roundToInt().coerceIn(0, cfg.heightPx - 1)
    }

    fun xToAzimuth(x: Int, cfg: AtlasConfig): Float {
        val denom = (cfg.widthPx - 1).coerceAtLeast(1)
        val t = x.coerceIn(0, cfg.widthPx - 1).toFloat() / denom.toFloat()
        return cfg.minAzimuthDeg + t * (cfg.maxAzimuthDeg - cfg.minAzimuthDeg)
    }

    fun yToAltitude(y: Int, cfg: AtlasConfig): Float {
        val denom = (cfg.heightPx - 1).coerceAtLeast(1)
        val t = y.coerceIn(0, cfg.heightPx - 1).toFloat() / denom.toFloat()
        return cfg.maxAltitudeDeg - t * (cfg.maxAltitudeDeg - cfg.minAltitudeDeg)
    }

    fun splitAzimuthSpan(
        minAzimuthDeg: Float,
        maxAzimuthDeg: Float,
        cfg: AtlasConfig
    ): List<Pair<Int, Int>> {
        val minNorm = normalizeAzimuthDeg(minAzimuthDeg)
        val maxNorm = normalizeAzimuthDeg(maxAzimuthDeg)

        return if (minNorm <= maxNorm) {
            listOf(azimuthToX(minNorm, cfg) to azimuthToX(maxNorm, cfg))
        } else {
            listOf(
                0 to azimuthToX(maxNorm, cfg),
                azimuthToX(minNorm, cfg) to (cfg.widthPx - 1)
            )
        }
    }
}