package com.example.sunproject.domain.atlas

import com.example.sunproject.data.storage.JsonSessionStore
import java.io.File

class AtlasBuildUseCase(
    private val store: JsonSessionStore = JsonSessionStore()
) {
    private fun newAtlas(): SkyAtlas =
        SkyAtlas(
            AtlasConfig(
                widthPx = 3600,
                heightPx = 900
            )
        )

    fun buildDebugAtlas(sessionDir: File): File {
        val paths = store.createSessionPaths(sessionDir)
        val frames = store.loadFrames(paths).sortedBy { it.shotIndex }

        val atlas = newAtlas()
        val out = File(paths.atlasDir, "atlas_debug.png")

        AtlasDebugRenderer.renderFootprints(frames, atlas, out)
        return out
    }

    fun buildSingleFrameProjection(sessionDir: File, frameIndex: Int = 0): File {
        val paths = store.createSessionPaths(sessionDir)
        val frames = store.loadFrames(paths).sortedBy { it.shotIndex }

        require(frames.isNotEmpty()) { "No hay frames en la sesión." }

        val selected = frames[frameIndex.coerceIn(0, frames.lastIndex)]
        val atlas = newAtlas()

        AtlasProjector.projectSingleFrameToAtlas(selected, atlas)

        val out = File(
            paths.atlasDir,
            "atlas_projected_${selected.shotIndex.toString().padStart(2, '0')}.png"
        )

        atlas.writePng(out)
        return out
    }

    fun buildProjectedAtlas(sessionDir: File): File {
        val paths = store.createSessionPaths(sessionDir)
        val frames = store.loadFrames(paths).sortedBy { it.shotIndex }

        require(frames.isNotEmpty()) { "No hay frames en la sesión." }

        val atlas = newAtlas()
        AtlasProjector.projectFramesToAtlas(frames, atlas)

        val out = File(paths.atlasDir, "atlas_projected_all.png")
        atlas.writePng(out)
        return out
    }

    fun renderZenithYawDebug(sessionDir: File): File {
        ZenithYawDebugRenderer.renderDebugSheet(sessionDir)
        val paths = store.createSessionPaths(sessionDir)
        return File(paths.atlasDir, "atlas_zenith_yaw_debug.png")
    }
}