package com.example.sunproject.data.storage

import android.content.Context
import com.example.sunproject.data.model.CameraMountCalibration

object CameraMountCalibrationStore {

    private const val PREFS_NAME = "camera_mount_calibration"

    private const val KEY_FX = "fx"
    private const val KEY_FY = "fy"
    private const val KEY_FZ = "fz"

    private const val KEY_RX = "rx"
    private const val KEY_RY = "ry"
    private const val KEY_RZ = "rz"

    private const val KEY_UX = "ux"
    private const val KEY_UY = "uy"
    private const val KEY_UZ = "uz"

    private const val KEY_SAMPLE_COUNT = "sample_count"
    private const val KEY_QUALITY_SCORE = "quality_score"
    private const val KEY_UPDATED_AT = "updated_at"

    fun load(context: Context): CameraMountCalibration? {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        if (!prefs.contains(KEY_FX)) return null

        return CameraMountCalibration(
            forwardInDevice = floatArrayOf(
                prefs.getFloat(KEY_FX, 0f),
                prefs.getFloat(KEY_FY, 0f),
                prefs.getFloat(KEY_FZ, -1f)
            ),
            rightInDevice = floatArrayOf(
                prefs.getFloat(KEY_RX, 1f),
                prefs.getFloat(KEY_RY, 0f),
                prefs.getFloat(KEY_RZ, 0f)
            ),
            upInDevice = floatArrayOf(
                prefs.getFloat(KEY_UX, 0f),
                prefs.getFloat(KEY_UY, -1f),
                prefs.getFloat(KEY_UZ, 0f)
            ),
            sampleCount = prefs.getInt(KEY_SAMPLE_COUNT, 0),
            qualityScore = prefs.getFloat(KEY_QUALITY_SCORE, 0f),
            updatedAtUtcMs = prefs.getLong(KEY_UPDATED_AT, 0L)
        )
    }

    fun save(context: Context, calibration: CameraMountCalibration) {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        prefs.edit()
            .putFloat(KEY_FX, calibration.forwardInDevice[0])
            .putFloat(KEY_FY, calibration.forwardInDevice[1])
            .putFloat(KEY_FZ, calibration.forwardInDevice[2])
            .putFloat(KEY_RX, calibration.rightInDevice[0])
            .putFloat(KEY_RY, calibration.rightInDevice[1])
            .putFloat(KEY_RZ, calibration.rightInDevice[2])
            .putFloat(KEY_UX, calibration.upInDevice[0])
            .putFloat(KEY_UY, calibration.upInDevice[1])
            .putFloat(KEY_UZ, calibration.upInDevice[2])
            .putInt(KEY_SAMPLE_COUNT, calibration.sampleCount)
            .putFloat(KEY_QUALITY_SCORE, calibration.qualityScore)
            .putLong(KEY_UPDATED_AT, calibration.updatedAtUtcMs)
            .apply()
    }

    fun clear(context: Context) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
            .edit()
            .clear()
            .apply()
    }
}