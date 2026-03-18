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
        canvas.drawColor(Color.DKGRAY)

        val gridPaint = Paint().apply {
            color = Color.GRAY
            strokeWidth = 1f
            style = Paint.Style.STROKE
        }

        val rectPaint = Paint().apply {
            color = Color.GREEN
            strokeWidth = 2f
            style = Paint.Style.STROKE
        }

        // grilla simple
        for (i in 0..12) {
            val x = i * atlas.width / 12f
            canvas.drawLine(x, 0f, x, atlas.height.toFloat(), gridPaint)
        }
        for (i in 0..9) {
            val y = i * atlas.height / 9f
            canvas.drawLine(0f, y, atlas.width.toFloat(), y, gridPaint)
        }

        frames.forEach { frame ->
            val fp = AtlasProjector.approximateFootprint(frame)
            val x0 = AtlasMath.azimuthToX(fp.minAzimuthDeg, atlas.config)
            val x1 = AtlasMath.azimuthToX(fp.maxAzimuthDeg, atlas.config)
            val y0 = AtlasMath.altitudeToY(fp.maxAltitudeDeg, atlas.config)
            val y1 = AtlasMath.altitudeToY(fp.minAltitudeDeg, atlas.config)

            canvas.drawRect(
                x0.toFloat(),
                y0.toFloat(),
                x1.toFloat(),
                y1.toFloat(),
                rectPaint
            )
        }

        FileOutputStream(outFile).use {
            bmp.compress(Bitmap.CompressFormat.PNG, 100, it)
        }
    }
}