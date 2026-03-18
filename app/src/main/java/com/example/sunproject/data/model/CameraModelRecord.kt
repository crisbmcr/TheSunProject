package com.example.sunproject.data.model

data class CameraModelRecord(
    val cameraId: String,
    val imageWidthPx: Int,
    val imageHeightPx: Int,
    val hfovDeg: Float,
    val vfovDeg: Float,
    val fxPx: Double? = null,
    val fyPx: Double? = null,
    val cxPx: Double? = null,
    val cyPx: Double? = null
)