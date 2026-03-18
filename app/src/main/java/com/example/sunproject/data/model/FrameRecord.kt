package com.example.sunproject.data.model

data class FrameRecord(
    val frameId: String,
    val sessionId: String,
    val ringId: String,
    val shotIndex: Int,
    val originalPath: String,
    val capturedAtUtcMs: Long,
    val targetAzimuthDeg: Float,
    val targetPitchDeg: Float,
    val measuredAzimuthDeg: Float,
    val measuredPitchDeg: Float,
    val measuredRollDeg: Float,
    val latitudeDeg: Double?,
    val longitudeDeg: Double?,
    val altitudeM: Double?,
    val imageWidthPx: Int?,
    val imageHeightPx: Int?,
    val hfovDeg: Float?,
    val vfovDeg: Float?
)