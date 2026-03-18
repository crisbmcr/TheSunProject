package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.example.sunproject.data.model.FrameRecord
import java.io.File
import java.io.FileOutputStream

object AtlasDebugRenderer {

    fun renderFootprints(
        frames: List<FrameRecord>,
        atlas: SkyAtlas,
        outFile: File
    ) {
        val bmp = Bitmap.createBitmap(atlas.width, atlas.height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bmp)

        canvas.drawColor(Color.rgb(32, 32, 32))

        val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.GRAY
            strokeWidth = 1f
            style = Paint.Style.STROKE
        }

        val centerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = Color.WHITE
        }

        drawGrid(canvas, atlas, gridPaint)

        frames.sortedBy { it.shotIndex }.forEach { frame ->
            val fp = AtlasProjector.approximateFootprint(frame)
            val strokeColor = ringColor(frame.ringId)

            val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = strokeColor
                strokeWidth = 2f
                style = Paint.Style.STROKE
            }

            val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = Color.argb(
                    56,
                    Color.red(strokeColor),
                    Color.green(strokeColor),
                    Color.blue(strokeColor)
                )
                style = Paint.Style.FILL
            }

            val y0 = AtlasMath.altitudeToY(fp.maxAltitudeDeg, atlas.config)
            val y1 = AtlasMath.altitudeToY(fp.minAltitudeDeg, atlas.config)

            AtlasMath.splitAzimuthSpan(fp.minAzimuthDeg, fp.maxAzimuthDeg, atlas.config)
                .forEach { (x0, x1) ->
                    canvas.drawRect(
                        x0.toFloat(),
                        y0.toFloat(),
                        x1.toFloat(),
                        y1.toFloat(),
                        fillPaint
                    )
                    canvas.drawRect(
                        x0.toFloat(),
                        y0.toFloat(),
                        x1.toFloat(),
                        y1.toFloat(),
                        strokePaint
                    )
                }

            val cx = AtlasMath.azimuthToX(frame.measuredAzimuthDeg, atlas.config)
            val cy = AtlasMath.altitudeToY(frame.measuredPitchDeg.coerceIn(0f, 90f), atlas.config)
            canvas.drawCircle(cx.toFloat(), cy.toFloat(), 5f, centerPaint)
        }

        outFile.parentFile?.mkdirs()
        FileOutputStream(outFile).use {
            bmp.compress(Bitmap.CompressFormat.PNG, 100, it)
        }
    }

    private fun drawGrid(canvas: Canvas, atlas: SkyAtlas, gridPaint: Paint) {
        for (az in -180..180 step 30) {
            val x = AtlasMath.azimuthToX(az.toFloat(), atlas.config).toFloat()
            canvas.drawLine(x, 0f, x, atlas.height.toFloat(), gridPaint)
        }

        for (alt in 0..90 step 10) {
            val y = AtlasMath.altitudeToY(alt.toFloat(), atlas.config).toFloat()
            canvas.drawLine(0f, y, atlas.width.toFloat(), y, gridPaint)
        }
    }

    private fun ringColor(ringId: String): Int = when (ringId) {
        "H45" -> Color.CYAN
        "Z0" -> Color.MAGENTA
        else -> Color.GREEN
    }
}