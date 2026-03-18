package com.example.sunproject.data.model

data class RingPlanRecord(
    val ringId: String,
    val targetPitchDeg: Float,
    val azimuthStepDeg: Float,
    val shotCount: Int,
    val pitchToleranceDeg: Float,
    val rollToleranceDeg: Float,
    val holdMs: Long
)