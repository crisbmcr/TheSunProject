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

import android.graphics.BitmapFactory
import android.util.Log
import java.io.File
import java.io.FileOutputStream

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

        // Orden de capas: grilla debajo de todo, marcador arriba de todo.
        drawAxes(canvas, atlasConfig, atlasBitmap.width, atlasBitmap.height)

        chart.dailyPaths.forEach { path ->
            path.monthIndex?.let { month ->
                drawPath(canvas, atlasConfig, path, colorForMonth(month), strokePx = 2.4f)
            }
        }
        chart.hourlyPaths.forEach { path ->
            drawPath(canvas, atlasConfig, path, HOUR_LINE_COLOR, strokePx = 1.4f)
            labelHour(canvas, atlasConfig, path)
        }

        chart.captureDayPath?.let { path ->
            drawPath(canvas, atlasConfig, path, CAPTURE_DAY_COLOR, strokePx = 4.5f)
            labelCaptureDay(canvas, atlasConfig, path)
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

    private fun drawAxes(
        canvas: Canvas,
        cfg: AtlasConfig,
        width: Int,
        height: Int
    ) {
        val minor = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(55, 255, 255, 255)
            style = Paint.Style.STROKE
            strokeWidth = 0.5f
        }
        val major = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(110, 255, 255, 255)
            style = Paint.Style.STROKE
            strokeWidth = 1f
        }
        val horizon = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(200, 255, 255, 255)
            style = Paint.Style.STROKE
            strokeWidth = 2f
        }
        val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 22f
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }
        val cardinalText = Paint(text).apply { textSize = 30f }

        // Líneas horizontales (altitud) cada 10°
        for (alt in 10..80 step 10) {
            val y = AtlasMath.altitudeToY(alt.toFloat(), cfg).toFloat()
            canvas.drawLine(0f, y, width.toFloat(), y, minor)
            canvas.drawText("${alt}°", 8f, y - 4f, text)
        }

        // Horizonte más fuerte
        val yHorizon = AtlasMath.altitudeToY(0f, cfg).toFloat()
        canvas.drawLine(0f, yHorizon, width.toFloat(), yHorizon, horizon)
        canvas.drawText("0°", 8f, yHorizon - 4f, text)

        // Líneas verticales (azimut). Todas cada 10°; cada 30° son mayores con etiqueta.
        for (az in -180..180 step 10) {
            val x = AtlasMath.azimuthToX(az.toFloat(), cfg).toFloat()
            val isMajor = az % 30 == 0
            canvas.drawLine(x, 0f, x, height.toFloat(), if (isMajor) major else minor)

            if (isMajor) {
                val azNorm = ((az % 360) + 360) % 360
                val label = when (azNorm) {
                    0 -> "N"
                    90 -> "E"
                    180 -> "S"
                    270 -> "O"
                    else -> "${azNorm}°"
                }
                val paintToUse = if (azNorm % 90 == 0) cardinalText else text
                canvas.drawText(label, x + 4f, (height - 10).toFloat(), paintToUse)
            }
        }
    }

    private fun labelCaptureDay(canvas: Canvas, cfg: AtlasConfig, path: SolarPath) {
        val top = path.points.filter { !it.belowHorizon }
            .maxByOrNull { it.altitudeDeg } ?: return

        val x = azToX(top.azimuthDeg, cfg).toFloat()
        val y = altToY(top.altitudeDeg, cfg).toFloat()

        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = CAPTURE_DAY_COLOR
            textSize = 26f
            isFakeBoldText = true
            setShadowLayer(4f, 1f, 1f, Color.BLACK)
        }
        canvas.drawText("hoy ${path.label}", x + 8f, y - 14f, paint)
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
    /**
     * Renderiza el chart sobre una copia del atlas y guarda el PNG resultante
     * como "atlas_projected_all_abaco.png" en el mismo directorio que el
     * atlas original. Devuelve el File escrito, o null si falló la escritura.
     *
     * No modifica el bitmap original ni el archivo original.
     */
    fun exportWithChart(
        atlasFile: File,
        atlasConfig: AtlasConfig,
        chart: SolarChart
    ): File? {
        val src = BitmapFactory.decodeFile(atlasFile.absolutePath) ?: return null
        val mutable = src.copy(Bitmap.Config.ARGB_8888, true)
        src.recycle()

        try {
            drawChart(mutable, atlasConfig, chart)

            val outName = atlasFile.nameWithoutExtension + "_abaco." + atlasFile.extension
            val outFile = File(atlasFile.parentFile, outName)

            FileOutputStream(outFile).use { stream ->
                mutable.compress(Bitmap.CompressFormat.PNG, 100, stream)
            }

            Log.i(
                "SolarOverlayExport",
                "Exported: ${outFile.absolutePath} (${outFile.length() / 1024} KB)"
            )
            return outFile
        } catch (t: Throwable) {
            Log.e("SolarOverlayExport", "Export falló", t)
            return null
        } finally {
            mutable.recycle()
        }
    }


    private val HOUR_LINE_COLOR = Color.argb(140, 250, 199, 117)
    private val HOUR_LABEL_COLOR = Color.rgb(250, 199, 117)
    private val CAPTURE_DAY_COLOR = Color.rgb(255, 255, 255)  // blanco, se destaca sobre todo
}