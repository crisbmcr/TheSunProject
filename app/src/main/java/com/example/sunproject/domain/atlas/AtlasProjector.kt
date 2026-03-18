package com.example.sunproject.domain.atlas

import com.example.sunproject.data.model.FrameRecord

data class FrameFootprint(
    val minAzimuthDeg: Float,
    val maxAzimuthDeg: Float,
    val minAltitudeDeg: Float,
    val maxAltitudeDeg: Float
)

object AtlasProjector {
    fun approximateFootprint(frame: FrameRecord): FrameFootprint {
        val hfov = frame.hfovDeg ?: 65f
        val vfov = frame.vfovDeg ?: 50f

        return FrameFootprint(
            minAzimuthDeg = frame.measuredAzimuthDeg - hfov / 2f,
            maxAzimuthDeg = frame.measuredAzimuthDeg + hfov / 2f,
            minAltitudeDeg = frame.measuredPitchDeg - vfov / 2f,
            maxAltitudeDeg = frame.measuredPitchDeg + vfov / 2f
        )
    }
}