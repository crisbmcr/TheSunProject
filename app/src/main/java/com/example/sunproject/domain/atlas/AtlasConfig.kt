package com.example.sunproject.domain.atlas

data class AtlasConfig(
    val widthPx: Int,
    val heightPx: Int,
    val minAzimuthDeg: Float = -180f,
    val maxAzimuthDeg: Float = 180f,
    val minAltitudeDeg: Float = 0f,
    val maxAltitudeDeg: Float = 90f
)