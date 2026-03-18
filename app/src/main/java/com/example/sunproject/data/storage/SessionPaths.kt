package com.example.sunproject.data.storage

import java.io.File

data class SessionPaths(
    val sessionDir: File,
    val sessionJson: File,
    val framesJson: File,
    val atlasDir: File
)