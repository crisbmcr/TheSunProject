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
import kotlin.math.sqrt

/**
 * Debug renderer that produces a single PNG with the Z0 cap rendered
 * using multiple candidate yaws. The last H45 of the session is rendered
 * alongside as a reference, so the user can visually identify which
 * candidate aligns with the ring below.
 *
 * Usage: call renderDebugSheet(sessionDir) from AtlasBuildUseCase or
 * CaptureActivity. Output written to atlas/atlas_zenith_yaw_debug.png.
 *
 * The sheet has N+1 rows. Each row is an independent atlas rendered at
 * width × (height/4), showing only the altitude band 0..90 (we crop
 * below the horizon is already excluded by AtlasConfig default).
 *
 * Row 0: only the last H45 and its two neighbors (reference, correct).
 * Rows 1..N: row 0 + the Z0 with candidate yaw N.
 *
 * Candidate yaws tried:
 *   - refYaw (= last H45 measuredAzimuthDeg)
 *   - refYaw + 90
 *   - refYaw + 180
 *   - refYaw + 270
 *   - relYaw (current formula: refYaw + atan2(R[0,1], R[0,0]))
 *   - relYaw with sign flip: refYaw + atan2(R[1,0], R[0,0])
 *   - snapshotYaw (frame.absAzimuthDeg)
 *   - frame absAzimuthDeg unchanged (control)
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

        // Reference frames: last H45 and the two adjacent H45s so the user
        // has context around the seam.
        val h45Frames = frames.filter { it.ringId.equals("H45", ignoreCase = true) }
        val refFrames = h45Frames.takeLast(3)
        if (refFrames.isEmpty()) {
            Log.w("ZenithYawDebug", "No H45 frames in session")
            return
        }

        val refFrame = h45Frames.last()

        // Build yaw candidates.
        val candidates = buildCandidates(z0, refFrame)

        // Render params.
        val rowW = 1800
        val rowH = 300
        val labelH = 30
        val rows = candidates.size + 1 // one extra for reference-only at top
        val totalW = rowW
        val totalH = rows * (rowH + labelH)

        val out = Bitmap.createBitmap(totalW, totalH, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(out)
        canvas.drawColor(Color.BLACK)

        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 22f
        }

        // Row 0: reference only.
        drawLabel(canvas, "REFERENCE: last 3 H45 only (no Z0)",
            0, 0, textPaint)
        renderRow(
            canvas = canvas,
            xOff = 0,
            yOff = labelH,
            rowW = rowW,
            rowH = rowH,
            refFrames = refFrames,
            z0Frame = null,
            z0YawDeg = 0f
        )

        // Subsequent rows: reference + Z0 at candidate yaw.
        for ((i, cand) in candidates.withIndex()) {
            val rowY = (i + 1) * (rowH + labelH)
            drawLabel(canvas,
                "[${i + 1}] ${cand.label}: yaw=${"%.2f".format(cand.yawDeg)}",
                0, rowY, textPaint
            )
            renderRow(
                canvas = canvas,
                xOff = 0,
                yOff = rowY + labelH,
                rowW = rowW,
                rowH = rowH,
                refFrames = refFrames,
                z0Frame = z0,
                z0YawDeg = cand.yawDeg
            )
        }

        val outFile = File(paths.atlasDir, "atlas_zenith_yaw_debug.png")
        FileOutputStream(outFile).use { out.compress(Bitmap.CompressFormat.PNG, 100, it) }
        Log.i("ZenithYawDebug",
            "Wrote ${outFile.absolutePath} with ${candidates.size} candidates")
    }

    private fun buildCandidates(
        z0: FrameRecord,
        refFrame: FrameRecord
    ): List<YawCandidate> {
        val refYaw = refFrame.measuredAzimuthDeg

        // Compute both possible delta-yaw formulations.
        val mRef = storedMatrix(refFrame)
        val mZ0  = storedMatrix(z0)

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

        val candidates = mutableListOf<YawCandidate>()
        candidates += YawCandidate("refYaw (no delta)",      norm(refYaw))
        candidates += YawCandidate("refYaw + 90",            norm(refYaw + 90f))
        candidates += YawCandidate("refYaw + 180",           norm(refYaw + 180f))
        candidates += YawCandidate("refYaw + 270",           norm(refYaw + 270f))
        candidates += YawCandidate("refYaw + delta01",       norm(refYaw + delta01))
        candidates += YawCandidate("refYaw - delta01",       norm(refYaw - delta01))
        candidates += YawCandidate("refYaw + delta10",       norm(refYaw + delta10))
        candidates += YawCandidate("refYaw - delta10",       norm(refYaw - delta10))
        if (snapshotYaw != null) {
            candidates += YawCandidate("snapshotYaw",        norm(snapshotYaw))
            candidates += YawCandidate("snapshotYaw + 90",   norm(snapshotYaw + 90f))
            candidates += YawCandidate("snapshotYaw + 180",  norm(snapshotYaw + 180f))
            candidates += YawCandidate("snapshotYaw + 270",  norm(snapshotYaw + 270f))
        }

        // Log candidates for reference.
        candidates.forEachIndexed { i, c ->
            Log.i("ZenithYawDebug", "  cand[${i+1}] ${c.label}: yaw=${"%.2f".format(c.yawDeg)}")
        }

        return candidates
    }

    private fun renderRow(
        canvas: Canvas,
        xOff: Int,
        yOff: Int,
        rowW: Int,
        rowH: Int,
        refFrames: List<FrameRecord>,
        z0Frame: FrameRecord?,
        z0YawDeg: Float
    ) {
        val atlas = SkyAtlas(
            AtlasConfig(
                widthPx = rowW,
                heightPx = rowH
            )
        )

        // Render H45 reference frames first.
        for (ref in refFrames) {
            val src = BitmapFactory.decodeFile(ref.originalPath) ?: continue
            try {
                AtlasProjector.projectSingleFrameToAtlas(ref, atlas)
            } catch (t: Throwable) {
                Log.e("ZenithYawDebug", "H45 render failed for ${ref.frameId}", t)
            } finally {
                src.recycle()
            }
        }

        // Render Z0 with the forced yaw if requested.
        if (z0Frame != null) {
            val forcedZ0 = z0Frame.copy(
                absAzimuthDeg = z0YawDeg,
                absPitchDeg = z0Frame.absPitchDeg ?: z0Frame.measuredPitchDeg,
                absRollDeg = 0f
            )
            try {
                AtlasProjector.projectSingleFrameToAtlas(forcedZ0, atlas)
            } catch (t: Throwable) {
                Log.e("ZenithYawDebug", "Z0 render failed", t)
            }
        }

        // Blit atlas into canvas.
        val atlasBmp = atlas.toBitmap()
        canvas.drawBitmap(
            atlasBmp,
            Rect(0, 0, atlasBmp.width, atlasBmp.height),
            Rect(xOff, yOff, xOff + rowW, yOff + rowH),
            null
        )
        atlasBmp.recycle()
    }

    private fun drawLabel(canvas: Canvas, text: String, x: Int, y: Int, paint: Paint) {
        canvas.drawText(text, x.toFloat() + 8f, y.toFloat() + 22f, paint)
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

    private fun norm(deg: Float): Float {
        var v = deg % 360f
        if (v < 0f) v += 360f
        return v
    }
}