package com.example.sunproject.data.storage

import com.example.sunproject.data.model.FrameRecord
import com.example.sunproject.data.model.SessionRecord
import java.io.File

interface SessionRepository {
    fun createSessionPaths(sessionDir: File): SessionPaths
    fun saveSession(session: SessionRecord, paths: SessionPaths)
    fun appendFrame(frame: FrameRecord, paths: SessionPaths)
    fun loadSession(paths: SessionPaths): SessionRecord?
    fun loadFrames(paths: SessionPaths): List<FrameRecord>
}