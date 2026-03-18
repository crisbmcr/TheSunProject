package com.example.sunproject.domain.atlas

object AtlasMath {
    fun azimuthToX(azimuthDeg: Float, cfg: AtlasConfig): Int {
        val az = azimuthDeg.coerceIn(cfg.minAzimuthDeg, cfg.maxAzimuthDeg)
        val t = (az - cfg.minAzimuthDeg) / (cfg.maxAzimuthDeg - cfg.minAzimuthDeg)
        return (t * (cfg.widthPx - 1)).toInt()
    }

    fun altitudeToY(altitudeDeg: Float, cfg: AtlasConfig): Int {
        val alt = altitudeDeg.coerceIn(cfg.minAltitudeDeg, cfg.maxAltitudeDeg)
        val t = (cfg.maxAltitudeDeg - alt) / (cfg.maxAltitudeDeg - cfg.minAltitudeDeg)
        return (t * (cfg.heightPx - 1)).toInt()
    }
}