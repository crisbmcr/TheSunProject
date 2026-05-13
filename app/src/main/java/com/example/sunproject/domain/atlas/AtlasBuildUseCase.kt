package com.example.sunproject.domain.atlas

import android.util.Log
import com.example.sunproject.data.storage.JsonSessionStore
import java.io.File

class AtlasBuildUseCase(
    private val store: JsonSessionStore = JsonSessionStore()
) {
    companion object {
        /**
         * Activa el refinamiento global de poses H0/H45 con bundle adjustment
         * antes de proyectar al atlas. Para A/B comparativo: poner false
         * deja el comportamiento original (seed IMU sin refinar).
         *
         * Sesión 2026-05-13: desactivado para evaluación. Con palancas A+B
         * (stability gate + pose averaging) y técnica de roll cuidadoso, la
         * pose IMU tiene precisión sub-0.1° y el BA está sobre-refinando
         * sobre datos que ya son buenos. Sospecha: BA introduce seams
         * verticales por convergencia a matches ORB ruidosos entre frames
         * H0↔H45 (contenido visual cambia mucho entre pitches, matches
         * espurios pasan el threshold MIN_MATCHES_PER_PAIR=8). Pendiente
         * confirmar con A/B: con true vs false sobre misma sesión.
         *
         * Si A/B confirma que false es mejor, dejar así por defecto.
         * El código del BA se mantiene en AtlasBundleAdjuster.kt como
         * dead code disponible para revisitar.
         */
        const val USE_BUNDLE_ADJUSTMENT_H0_H45 = false
    }

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
        val rawFrames = store.loadFrames(paths).sortedBy { it.shotIndex }

        require(rawFrames.isNotEmpty()) { "No hay frames en la sesión." }

        // Refinamiento global de poses H0/H45 via bundle adjustment.
        // Si está deshabilitado o el BA no converge, refineOrFallback devuelve
        // los frames sin tocar. La geometría absoluta del atlas (azimut/altitud
        // respecto del norte verdadero) está protegida por la regularización
        // al seed IMU dentro del BA — ver AtlasBundleAdjuster.kt.
        val frames = if (USE_BUNDLE_ADJUSTMENT_H0_H45) {
            val tStart = System.currentTimeMillis()
            val refined = AtlasBundleAdjuster.refineOrFallback(rawFrames)
            Log.i(
                "AtlasBuildUseCase",
                "BA total elapsed=${System.currentTimeMillis() - tStart}ms"
            )
            refined
        } else {
            rawFrames
        }

        // FIX TRUE-NORTH: inyectar declinación de la sesión antes de proyectar.
        // ZenithMatrixProjector la consume para alinear el Z0 (mag-N) con
        // los H0/H45 (true-N anclado al inicio).
        val session = store.loadSession(paths)
        AtlasProjector.setSessionDeclinationDeg(session?.declinationDeg)
        Log.i(
            "AtlasBuildUseCase",
            "sessionDeclination=${session?.declinationDeg?.let { "%.2f".format(it) } ?: "null"}°"
        )

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