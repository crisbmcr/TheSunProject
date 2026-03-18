package com.example.sunproject.data.storage

import com.example.sunproject.data.model.FrameRecord
import com.example.sunproject.data.model.SessionRecord
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

class JsonSessionStore : SessionRepository {

    override fun createSessionPaths(sessionDir: File): SessionPaths {
        val atlasDir = File(sessionDir, "atlas").apply { mkdirs() }
        return SessionPaths(
            sessionDir = sessionDir,
            sessionJson = File(sessionDir, "session.json"),
            framesJson = File(sessionDir, "frames.json"),
            atlasDir = atlasDir
        )
    }

    override fun saveSession(session: SessionRecord, paths: SessionPaths) {
        val obj = JSONObject()
            .put("sessionId", session.sessionId)
            .put("startedAtUtcMs", session.startedAtUtcMs)
            .put("latitudeDeg", session.latitudeDeg)
            .put("longitudeDeg", session.longitudeDeg)
            .put("altitudeM", session.altitudeM)
            .put("declinationDeg", session.declinationDeg)
            .put("cameraId", session.cameraId)
            .put("sensorMode", session.sensorMode)
            .put("sessionYawAnchorDeg", session.sessionYawAnchorDeg)
            .put("sessionPitchAnchorDeg", session.sessionPitchAnchorDeg)
            .put("sessionRollAnchorDeg", session.sessionRollAnchorDeg)
            .put("notes", session.notes)

        paths.sessionJson.writeText(obj.toString(2))
    }

    override fun appendFrame(frame: FrameRecord, paths: SessionPaths) {
        val arr = if (paths.framesJson.exists()) {
            JSONArray(paths.framesJson.readText())
        } else {
            JSONArray()
        }

        val obj = JSONObject()
            .put("frameId", frame.frameId)
            .put("sessionId", frame.sessionId)
            .put("ringId", frame.ringId)
            .put("shotIndex", frame.shotIndex)
            .put("originalPath", frame.originalPath)
            .put("capturedAtUtcMs", frame.capturedAtUtcMs)
            .put("targetAzimuthDeg", frame.targetAzimuthDeg)
            .put("targetPitchDeg", frame.targetPitchDeg)
            .put("measuredAzimuthDeg", frame.measuredAzimuthDeg)
            .put("measuredPitchDeg", frame.measuredPitchDeg)
            .put("measuredRollDeg", frame.measuredRollDeg)
            .put("latitudeDeg", frame.latitudeDeg)
            .put("longitudeDeg", frame.longitudeDeg)
            .put("altitudeM", frame.altitudeM)
            .put("imageWidthPx", frame.imageWidthPx)
            .put("imageHeightPx", frame.imageHeightPx)
            .put("hfovDeg", frame.hfovDeg)
            .put("vfovDeg", frame.vfovDeg)

        arr.put(obj)
        paths.framesJson.writeText(arr.toString(2))
    }

    override fun loadSession(paths: SessionPaths): SessionRecord? {
        if (!paths.sessionJson.exists()) return null
        val obj = JSONObject(paths.sessionJson.readText())
        return SessionRecord(
            sessionId = obj.getString("sessionId"),
            startedAtUtcMs = obj.getLong("startedAtUtcMs"),
            latitudeDeg = obj.optDouble("latitudeDeg").takeIf { !it.isNaN() },
            longitudeDeg = obj.optDouble("longitudeDeg").takeIf { !it.isNaN() },
            altitudeM = obj.optDouble("altitudeM").takeIf { !it.isNaN() },
            declinationDeg = obj.optDouble("declinationDeg").toFloat().takeIf { !it.isNaN() },
            cameraId = obj.optString("cameraId", null),
            sensorMode = obj.optString("sensorMode", "rotation_vector"),
            sessionYawAnchorDeg = obj.optDouble("sessionYawAnchorDeg").toFloat().takeIf { !it.isNaN() },
            sessionPitchAnchorDeg = obj.optDouble("sessionPitchAnchorDeg").toFloat().takeIf { !it.isNaN() },
            sessionRollAnchorDeg = obj.optDouble("sessionRollAnchorDeg").toFloat().takeIf { !it.isNaN() },
            notes = obj.optString("notes", null)
        )
    }

    override fun loadFrames(paths: SessionPaths): List<FrameRecord> {
        if (!paths.framesJson.exists()) return emptyList()
        val arr = JSONArray(paths.framesJson.readText())
        val out = mutableListOf<FrameRecord>()
        for (i in 0 until arr.length()) {
            val o = arr.getJSONObject(i)
            out += FrameRecord(
                frameId = o.getString("frameId"),
                sessionId = o.getString("sessionId"),
                ringId = o.getString("ringId"),
                shotIndex = o.getInt("shotIndex"),
                originalPath = o.getString("originalPath"),
                capturedAtUtcMs = o.getLong("capturedAtUtcMs"),
                targetAzimuthDeg = o.getDouble("targetAzimuthDeg").toFloat(),
                targetPitchDeg = o.getDouble("targetPitchDeg").toFloat(),
                measuredAzimuthDeg = o.getDouble("measuredAzimuthDeg").toFloat(),
                measuredPitchDeg = o.getDouble("measuredPitchDeg").toFloat(),
                measuredRollDeg = o.getDouble("measuredRollDeg").toFloat(),
                latitudeDeg = o.optDouble("latitudeDeg").takeIf { !it.isNaN() },
                longitudeDeg = o.optDouble("longitudeDeg").takeIf { !it.isNaN() },
                altitudeM = o.optDouble("altitudeM").takeIf { !it.isNaN() },
                imageWidthPx = o.optInt("imageWidthPx").takeIf { it != 0 },
                imageHeightPx = o.optInt("imageHeightPx").takeIf { it != 0 },
                hfovDeg = o.optDouble("hfovDeg").toFloat().takeIf { it != 0f },
                vfovDeg = o.optDouble("vfovDeg").toFloat().takeIf { it != 0f }
            )
        }
        return out
    }
}