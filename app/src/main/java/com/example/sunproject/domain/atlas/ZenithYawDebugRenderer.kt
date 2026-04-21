package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.util.Log
import com.example.sunproject.data.model.FrameRecord
import java.io.File
import java.io.FileOutputStream
import kotlin.math.atan2

/**
 * Debug renderer to identify the correct yaw formula for the Z0 cap.
 * Produces a single PNG with multiple rows:
 *   Row 0: full atlas (all H0 + H45, no Z0) as REFERENCE.
 *   Row 1..N: same full atlas + Z0 rendered at candidate yaw N.
 *
 * The user compares rows 1..N against row 0 and identifies which Z0
 * row continues the H45 ring features seamlessly. That candidate's
 * formula is the correct one.
 */
object ZenithYawDebugRenderer {

    private data class YawCandidate(
        val label: String,
        val yawDeg: Float
    )

    fun renderDebugSheet(sessionDir: File) {
        val store = com.example.sunproject.data.storage.JsonSessionStore()
        val paths = store.createSessionPaths(sessionDir)
        val frames = store.loadFrames(paths).sortedBy { it.shotIndex }

        val z0 = frames.firstOrNull { it.ringId.equals("Z0", ignoreCase = true) }
            ?: run {
                Log.w("ZenithYawDebug", "No Z0 in session ${sessionDir.name}")
                return
            }

        val h45Frames = frames.filter { it.ringId.equals("H45", ignoreCase = true) }
        val refFrame = h45Frames.lastOrNull()
            ?: run {
                Log.w("ZenithYawDebug", "No H45 in session")
                return
            }

        val nonZenithFrames = frames.filterNot { it.ringId.equals("Z0", ignoreCase = true) }

        val candidates = buildCandidates(z0, refFrame)

        val rowW = 1800
        val rowH = 450
        val labelH = 36
        val rows = candidates.size + 1
        val totalW = rowW
        val totalH = rows * (rowH + labelH)

        val out = Bitmap.createBitmap(totalW, totalH, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(out)
        canvas.drawColor(Color.BLACK)

        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 26f
        }

        // Build the reference atlas ONCE. Reuse it as the starting point
        // for every candidate row by copying its pixels.
        val refAtlasPixels = buildReferenceAtlasPixels(nonZenithFrames, rowW, rowH)

        // Row 0: reference atlas alone.
        drawLabel(
            canvas = canvas,
            text = "REFERENCE: H0 + H45 only (no Z0)",
            x = 0,
            y = 0,
            paint = textPaint
        )
        blitPixels(canvas, refAtlasPixels, rowW, rowH, 0, labelH)

        // Rows 1..N: reference + Z0 at candidate yaw.
        for ((i, cand) in candidates.withIndex()) {
            val rowY = (i + 1) * (rowH + labelH)
            drawLabel(
                canvas = canvas,
                text = "[${i + 1}] ${cand.label}: yaw=${"%.2f".format(cand.yawDeg)}",
                x = 0,
                y = rowY,
                paint = textPaint
            )

            val atlas = SkyAtlas(AtlasConfig(widthPx = rowW, heightPx = rowH))
            // Prime atlas with reference pixels so Z0 overlays onto a real panorama.
            primeAtlasFromPixels(atlas, refAtlasPixels)
            renderZ0OnAtlas(atlas, z0, cand.yawDeg)

            blitPixels(canvas, atlas.pixels, rowW, rowH, 0, rowY + labelH)
        }

        val outFile = File(paths.atlasDir, "atlas_zenith_yaw_debug.png")
        FileOutputStream(outFile).use {
            out.compress(Bitmap.CompressFormat.PNG, 100, it)
        }
        Log.i("ZenithYawDebug", "Wrote ${outFile.absolutePath}")
    }

    private fun buildReferenceAtlasPixels(
        nonZenithFrames: List<FrameRecord>,
        widthPx: Int,
        heightPx: Int
    ): IntArray {
        val atlas = SkyAtlas(AtlasConfig(widthPx = widthPx, heightPx = heightPx))
        for (fr in nonZenithFrames) {
            val src = BitmapFactory.decodeFile(fr.originalPath) ?: continue
            try {
                AtlasProjector.projectSingleFrameToAtlas(fr, atlas)
            } catch (t: Throwable) {
                Log.e("ZenithYawDebug", "render failed for ${fr.frameId}", t)
            } finally {
                src.recycle()
            }
        }
        return atlas.pixels.copyOf()
    }

    private fun primeAtlasFromPixels(atlas: SkyAtlas, pixels: IntArray) {
        // Copy reference pixels into atlas. Note that weightSums stays at 0,
        // which is fine — we're only overlaying Z0 where Z0 has data, and
        // the Z0 will blend over the primed pixels with default weights.
        System.arraycopy(pixels, 0, atlas.pixels, 0, pixels.size)
    }

    /**
     * Renders the Z0 frame on the atlas using the forced yaw. Bypasses
     * AtlasProjector.projectSingleFrameToAtlas because that helper uses
     * the frame's own absAzimuthDeg and ignores any override. Here we
     * replicate the core projection loop directly so the candidate yaw
     * actually takes effect.
     */
    private fun renderZ0OnAtlas(atlas: SkyAtlas, z0: FrameRecord, yawDeg: Float) {
        // Rewrite the frame's absolute pose to force the projector into
        // the hard-zenith branch at the given yaw. Roll is forced to 0
        // because forceHardZenith path already discards it.
        val forcedZ0 = z0.copy(
            absAzimuthDeg = normalizeDeg(yawDeg),
            absPitchDeg = (z0.absPitchDeg ?: z0.measuredPitchDeg).coerceIn(84f, 90f),
            absRollDeg = 0f
        )
        val src = BitmapFactory.decodeFile(forcedZ0.originalPath) ?: return
        try {
            AtlasProjector.projectSingleFrameToAtlas(forcedZ0, atlas)
        } catch (t: Throwable) {
            Log.e("ZenithYawDebug", "Z0 render failed", t)
        } finally {
            src.recycle()
        }
    }

    private fun buildCandidates(
        z0: FrameRecord,
        refFrame: FrameRecord
    ): List<YawCandidate> {
        val refYaw = refFrame.measuredAzimuthDeg

        val mRef = storedMatrix(refFrame)
        val mZ0 = storedMatrix(z0)

        val (delta01, delta10) = if (mRef != null && mZ0 != null) {
            val r00 = mZ0[0] * mRef[0] + mZ0[1] * mRef[1] + mZ0[2] * mRef[2]
            val r01 = mZ0[0] * mRef[3] + mZ0[1] * mRef[4] + mZ0[2] * mRef[5]
            val r10 = mZ0[3] * mRef[0] + mZ0[4] * mRef[1] + mZ0[5] * mRef[2]
            Pair(
                Math.toDegrees(atan2(r01.toDouble(), r00.toDouble())).toFloat(),
                Math.toDegrees(atan2(r10.toDouble(), r00.toDouble())).toFloat()
            )
        } else {
            Pair(0f, 0f)
        }

        val snapshotYaw = z0.absAzimuthDeg

        val out = mutableListOf<YawCandidate>()
        out += YawCandidate("refYaw (no delta)", normalizeDeg(refYaw))
        out += YawCandidate("refYaw + 90", normalizeDeg(refYaw + 90f))
        out += YawCandidate("refYaw + 180", normalizeDeg(refYaw + 180f))
        out += YawCandidate("refYaw + 270", normalizeDeg(refYaw + 270f))
        out += YawCandidate("refYaw + delta01", normalizeDeg(refYaw + delta01))
        out += YawCandidate("refYaw - delta01", normalizeDeg(refYaw - delta01))
        out += YawCandidate("refYaw + delta10", normalizeDeg(refYaw + delta10))
        out += YawCandidate("refYaw - delta10", normalizeDeg(refYaw - delta10))
        if (snapshotYaw != null) {
            out += YawCandidate("snapshotYaw", normalizeDeg(snapshotYaw))
            out += YawCandidate("snapshotYaw + 90", normalizeDeg(snapshotYaw + 90f))
            out += YawCandidate("snapshotYaw + 180", normalizeDeg(snapshotYaw + 180f))
            out += YawCandidate("snapshotYaw + 270", normalizeDeg(snapshotYaw + 270f))
        }

        out.forEachIndexed { i, c ->
            Log.i(
                "ZenithYawDebug",
                "cand[${i + 1}] ${c.label}: yaw=${"%.2f".format(c.yawDeg)}"
            )
        }

        return out
    }

    private fun blitPixels(
        canvas: Canvas,
        pixels: IntArray,
        srcW: Int,
        srcH: Int,
        dstX: Int,
        dstY: Int
    ) {
        val bmp = Bitmap.createBitmap(srcW, srcH, Bitmap.Config.ARGB_8888)
        bmp.setPixels(pixels, 0, srcW, 0, 0, srcW, srcH)
        canvas.drawBitmap(
            bmp,
            Rect(0, 0, srcW, srcH),
            Rect(dstX, dstY, dstX + srcW, dstY + srcH),
            null
        )
        bmp.recycle()
    }

    private fun drawLabel(
        canvas: Canvas,
        text: String,
        x: Int,
        y: Int,
        paint: Paint
    ) {
        canvas.drawText(text, x.toFloat() + 8f, y.toFloat() + 28f, paint)
    }

    private fun storedMatrix(fr: FrameRecord): FloatArray? {
        val m00 = fr.rotationM00 ?: return null
        val m01 = fr.rotationM01 ?: return null
        val m02 = fr.rotationM02 ?: return null
        val m10 = fr.rotationM10 ?: return null
        val m11 = fr.rotationM11 ?: return null
        val m12 = fr.rotationM12 ?: return null
        val m20 = fr.rotationM20 ?: return null
        val m21 = fr.rotationM21 ?: return null
        val m22 = fr.rotationM22 ?: return null
        return floatArrayOf(m00, m01, m02, m10, m11, m12, m20, m21, m22)
    }

    private fun normalizeDeg(deg: Float): Float {
        var v = deg % 360f
        if (v < 0f) v += 360f
        return v
    }
}