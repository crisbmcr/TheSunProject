package com.example.sunproject.data.model

data class CameraMountCalibration(
    val forwardInDevice: FloatArray,
    val rightInDevice: FloatArray,
    val upInDevice: FloatArray,
    val sampleCount: Int,
    val qualityScore: Float,
    val updatedAtUtcMs: Long
)