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

    // Pose display / UI
    val measuredAzimuthDeg: Float,

    val measuredPitchDeg: Float,

    val measuredRollDeg: Float,

    // Pose absoluta derivada del mismo TYPE_ROTATION_VECTOR
    val absAzimuthDeg: Float? = null,

    val absPitchDeg: Float? = null,

    val absRollDeg: Float? = null,

    // Matriz device->world persistida al capturar
    val rotationM00: Float? = null,

    val rotationM01: Float? = null,

    val rotationM02: Float? = null,

    val rotationM10: Float? = null,

    val rotationM11: Float? = null,

    val rotationM12: Float? = null,

    val rotationM20: Float? = null,

    val rotationM21: Float? = null,

    val rotationM22: Float? = null,
    val latitudeDeg: Double?,

    val longitudeDeg: Double?,

    val altitudeM: Double?,

    val imageWidthPx: Int?,

    val imageHeightPx: Int?,

    val hfovDeg: Float?,

    val vfovDeg: Float?
)