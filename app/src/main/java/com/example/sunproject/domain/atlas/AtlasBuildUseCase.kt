package com.example.sunproject.domain.atlas

import com.example.sunproject.data.storage.JsonSessionStore
import java.io.File

class AtlasBuildUseCase(
    private val store: JsonSessionStore = JsonSessionStore()
) {
    fun buildDebugAtlas(sessionDir: File): File {
        val paths = store.createSessionPaths(sessionDir)
        val frames = store.loadFrames(paths)

        val atlas = SkyAtlas(
            AtlasConfig(
                widthPx = 3600,
                heightPx = 900
            )
        )

        val out = File(paths.atlasDir, "atlas_debug.png")
        AtlasDebugRenderer.renderFootprints(frames, atlas, out)
        return out
    }
}