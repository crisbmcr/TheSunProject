package com.example.sunproject.domain.horizon

import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/**
 * Persistencia del HorizonProfile en disco como JSON.
 *
 * Schema versionado en HorizonProfile.CURRENT_SCHEMA_VERSION. Versión 2
 * corresponde al detector basado en TFLite. Versión 1 (clásico V+S+Otsu)
 * tiene un set distinto de DetectionParams; si load() detecta v1, igual
 * carga el perfil (altByAzimuthDeg sigue siendo válido) pero usa
 * DetectionParams.DEFAULT para los nuevos params.
 */
object HorizonProfileStore {

    private const val TAG = "HorizonProfileStore"
    private const val FILE_NAME = "horizon_profile.json"

    fun fileFor(atlasDir: File): File = File(atlasDir, FILE_NAME)
    fun exists(atlasDir: File): Boolean = fileFor(atlasDir).exists()

    fun save(profile: HorizonProfile, atlasDir: File): File {
        atlasDir.mkdirs()
        val file = fileFor(atlasDir)

        val altArray = JSONArray()
        profile.altByAzimuthDeg.forEach { altArray.put(it.toDouble()) }

        val paramsObj = JSONObject()
            .put("aggregationStrategy", profile.detectionParams.aggregationStrategy.name)
            .put("smoothingWindowDeg", profile.detectionParams.smoothingWindowDeg)

        val obj = JSONObject()
            .put("schemaVersion", profile.schemaVersion)
            .put("sessionId", profile.sessionId)
            .put("createdAtUtcMs", profile.createdAtUtcMs)
            .put("sourceAtlasName", profile.sourceAtlasName)
            .put("azimuthBuckets", profile.azimuthBuckets)
            .put("altByAzimuthDeg", altArray)
            .put("detectionParams", paramsObj)

        file.writeText(obj.toString(2))
        Log.i(TAG, "Saved horizon profile to ${file.absolutePath} (${profile.azimuthBuckets} buckets, schema v${profile.schemaVersion})")
        return file
    }

    fun load(atlasDir: File): HorizonProfile? {
        val file = fileFor(atlasDir)
        if (!file.exists()) return null

        return try {
            val obj = JSONObject(file.readText())

            val schemaVersion = obj.optInt("schemaVersion", 1)
            if (schemaVersion > HorizonProfile.CURRENT_SCHEMA_VERSION) {
                Log.w(
                    TAG,
                    "Profile schema v$schemaVersion newer than supported " +
                            "(v${HorizonProfile.CURRENT_SCHEMA_VERSION}). Ignoring."
                )
                return null
            }

            val altArray = obj.getJSONArray("altByAzimuthDeg")
            val alt = List(altArray.length()) { altArray.getDouble(it).toFloat() }

            val paramsObj = obj.optJSONObject("detectionParams") ?: JSONObject()
            val aggregationName = paramsObj.optString("aggregationStrategy", AggregationStrategy.P75.name)
            val aggregation = try {
                AggregationStrategy.valueOf(aggregationName)
            } catch (_: IllegalArgumentException) {
                Log.w(TAG, "Unknown aggregationStrategy '$aggregationName', defaulting to P75")
                AggregationStrategy.P75
            }

            val params = DetectionParams(
                aggregationStrategy = aggregation,
                smoothingWindowDeg = paramsObj.optInt("smoothingWindowDeg", 5)
            )

            HorizonProfile(
                schemaVersion = schemaVersion,
                sessionId = obj.optString("sessionId", ""),
                createdAtUtcMs = obj.optLong("createdAtUtcMs", 0L),
                sourceAtlasName = obj.optString("sourceAtlasName", ""),
                azimuthBuckets = obj.optInt("azimuthBuckets", alt.size),
                altByAzimuthDeg = alt,
                detectionParams = params
            )
        } catch (t: Throwable) {
            Log.w(TAG, "Failed to parse ${file.absolutePath}", t)
            null
        }
    }

    fun delete(atlasDir: File): Boolean {
        val file = fileFor(atlasDir)
        return if (file.exists()) file.delete() else false
    }
}