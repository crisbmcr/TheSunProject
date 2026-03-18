package com.example.sunproject.data.model

data class SessionRecord(
    val sessionId: String,
    val startedAtUtcMs: Long,
    val latitudeDeg: Double?,
    val longitudeDeg: Double?,
    val altitudeM: Double?,
    val declinationDeg: Float?,
    val cameraId: String?,
    val sensorMode: String,
    val sessionYawAnchorDeg: Float?,
    val sessionPitchAnchorDeg: Float?,
    val sessionRollAnchorDeg: Float?,
    val notes: String? = null
)