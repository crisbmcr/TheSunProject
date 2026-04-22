package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import com.example.sunproject.domain.solar.SolarChart
import com.example.sunproject.domain.solar.SolarPath
import com.example.sunproject.domain.solar.SolarPosition
import kotlin.math.abs

/**
 * Superpone un SolarChart sobre el bitmap de un atlas equirectangular.
 * Usa AtlasMath para el mapeo (az, alt) -> (x, y), por lo que los puntos
 * caen en el mismo píxel que cualquier frame proyectado.
 */
object SolarPathOverlay {

    fun drawChart(
        atlasBitmap: Bitmap,
        atlasConfig: AtlasConfig,
        chart: SolarChart
    ) {
        val canvas = Canvas(atlasBitmap)

        chart.dailyPaths.forEach { path ->
            path.monthIndex?.let { month ->
                drawPath(canvas, atlasConfig, path, colorForMonth(month), strokePx = 2.4f)
            }
        }
        chart.hourlyPaths.forEach { path ->
            drawPath(canvas, atlasConfig, path, HOUR_LINE_COLOR, strokePx = 1.6f)
            labelHour(canvas, atlasConfig, path)
        }
        drawMonthLabels(canvas, atlasConfig, chart)
        chart.sunAtCapture?.let { drawSunMarker(canvas, atlasConfig, it) }
    }

    // ---- Paths -----------------------------------------------------------

    private fun drawPath(
        canvas: Canvas,
        cfg: AtlasConfig,
        path: SolarPath,
        color: Int,
        strokePx: Float
    ) {
        val skyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = strokePx
            this.color = color
        }
        val belowPaint = Paint(skyPaint).apply { alpha = 55 }

        val skyPath = Path()
        val belowPath = Path()
        var prev: SolarPosition? = null
        var skyOpen = false
        var belowOpen = false

        for (pos in path.points) {
            val x = azToX(pos.azimuthDeg, cfg).toFloat()
            val y = altToY(pos.altitudeDeg, cfg).toFloat()

            val wrap = prev != null &&
                    abs(shortestAngleDelta(prev!!.azimuthDeg, pos.azimuthDeg)) > 90.0

            if (pos.belowHorizon) {
                if (!belowOpen || wrap) belowPath.moveTo(x, y) else belowPath.lineTo(x, y)
                belowOpen = true; skyOpen = false
            } else {
                if (!skyOpen || wrap) skyPath.moveTo(x, y) else skyPath.lineTo(x, y)
                skyOpen = true; belowOpen = false
            }
            prev = pos
        }

        canvas.drawPath(skyPath, skyPaint)
        canvas.drawPath(belowPath, belowPaint)
    }

    // ---- Labels ----------------------------------------------------------

    private fun labelHour(canvas: Canvas, cfg: AtlasConfig, path: SolarPath) {
        val top = path.points.filter { !it.belowHorizon }
            .maxByOrNull { it.altitudeDeg } ?: return

        val x = azToX(top.azimuthDeg, cfg).toFloat()
        val y = altToY(top.altitudeDeg, cfg).toFloat()

        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = HOUR_LABEL_COLOR
            textSize = 22f
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }
        canvas.drawText("${path.label} solar", x + 6f, y - 6f, paint)
    }

    private fun drawMonthLabels(canvas: Canvas, cfg: AtlasConfig, chart: SolarChart) {
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            textSize = 20f
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }
        val seen = mutableSetOf<Int>()
        chart.dailyPaths.forEach { path ->
            val month = path.monthIndex ?: return@forEach
            if (month !in listOf(6, 12)) return@forEach  // solo solsticios
            if (!seen.add(month)) return@forEach

            val noon = path.points.filter { !it.belowHorizon }
                .maxByOrNull { it.altitudeDeg } ?: return@forEach
            val x = azToX(noon.azimuthDeg, cfg).toFloat()
            val y = altToY(noon.altitudeDeg, cfg).toFloat()

            paint.color = colorForMonth(month)
            canvas.drawText("21 ${path.label}", x + 8f, y - 10f, paint)
        }
    }

    private fun drawSunMarker(canvas: Canvas, cfg: AtlasConfig, pos: SolarPosition) {
        if (pos.belowHorizon) return
        val x = azToX(pos.azimuthDeg, cfg).toFloat()
        val y = altToY(pos.altitudeDeg, cfg).toFloat()

        val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(249, 203, 66); style = Paint.Style.FILL
        }
        val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(133, 79, 11); style = Paint.Style.STROKE; strokeWidth = 3f
        }
        canvas.drawCircle(x, y, 11f, fill)
        canvas.drawCircle(x, y, 11f, ring)
    }

    // ---- Mapeo -----------------------------------------------------------

    /**
     * NOAA devuelve azimut en [0, 360) con 0=Norte. AtlasMath usa [-180, 180].
     * Normalizamos al rango del atlas antes de mapear.
     */
    private fun azToX(azimuthDeg: Double, cfg: AtlasConfig): Int {
        var az = azimuthDeg
        if (az > 180.0) az -= 360.0
        return AtlasMath.azimuthToX(az.toFloat(), cfg)
    }

    private fun altToY(altitudeDeg: Double, cfg: AtlasConfig): Int {
        return AtlasMath.altitudeToY(altitudeDeg.coerceIn(0.0, 90.0).toFloat(), cfg)
    }

    private fun shortestAngleDelta(a: Double, b: Double): Double {
        var d = (b - a) % 360.0
        if (d > 180.0) d -= 360.0
        if (d < -180.0) d += 360.0
        return d
    }

    // ---- Paleta ----------------------------------------------------------
    //
    // Hemisferio Sur: diciembre = verano (rojo/cálido), junio = invierno (azul).

    private fun colorForMonth(month: Int): Int = when (month) {
        12 -> Color.rgb(226, 75, 74)
        1, 11 -> Color.rgb(232, 93, 36)
        2, 10 -> Color.rgb(239, 159, 39)
        3, 9 -> Color.rgb(186, 117, 23)
        4, 8 -> Color.rgb(151, 196, 89)
        5, 7 -> Color.rgb(29, 158, 117)
        6 -> Color.rgb(55, 138, 221)
        else -> Color.GRAY
    }

    private val HOUR_LINE_COLOR = Color.argb(140, 250, 199, 117)
    private val HOUR_LABEL_COLOR = Color.rgb(250, 199, 117)
}