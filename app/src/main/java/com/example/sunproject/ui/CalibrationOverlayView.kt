package com.example.sunproject.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Typeface
import android.util.AttributeSet
import android.view.View
import com.example.sunproject.domain.sensor.AnchorCalibrator

/**
 * Overlay sobre la preview de la cámara que muestra el estado del
 * AnchorCalibrator. Se renderiza encima de la PreviewView y debajo
 * del GuideView.
 *
 * Cuando el anchor está READY o READY_DEGRADED se auto-oculta (visibility
 * = GONE) salvo flash inicial de 2s confirmando el resultado, que se
 * maneja en CaptureActivity vía setStatus(null).
 */
class CalibrationOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var status: AnchorCalibrator.Status? = null

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(180, 0, 0, 0)
    }
    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 48f
        typeface = Typeface.create(Typeface.SANS_SERIF, Typeface.BOLD)
    }
    private val bodyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 36f
        typeface = Typeface.MONOSPACE
    }
    private val hintPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(255, 255, 220, 100)  // amarillo suave
        textSize = 32f
        typeface = Typeface.create(Typeface.SANS_SERIF, Typeface.ITALIC)
    }
    private val progressBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(120, 255, 255, 255)
    }
    private val progressFgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(255, 100, 230, 100)  // verde
    }

    fun setStatus(s: AnchorCalibrator.Status?) {
        status = s
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val s = status ?: return

        // Layout: caja centrada horizontalmente, en el tercio superior.
        val boxWidth = width * 0.88f
        val boxLeft = (width - boxWidth) / 2f
        val boxTop = height * 0.12f
        val boxHeight = 280f
        val box = RectF(boxLeft, boxTop, boxLeft + boxWidth, boxTop + boxHeight)
        canvas.drawRoundRect(box, 24f, 24f, bgPaint)

        val title: String
        val body: String
        val hint: String?
        val progressFrac: Float

        when (s.phase) {
            AnchorCalibrator.Phase.WAITING_FOR_LOCATION -> {
                title = "Esperando GPS…"
                body = "Mag: ${magLabel(s.magAccuracy)}"
                hint = "Salí al aire libre para fix GPS"
                progressFrac = 0f
            }
            AnchorCalibrator.Phase.WAITING_FOR_MAG -> {
                title = "Magnetómetro no calibrado"
                body = "Estado: UNRELIABLE"
                hint = "Mové el celu en figura 8"
                progressFrac = 0f
            }
            AnchorCalibrator.Phase.COLLECTING -> {
                title = "Calibrando norte verdadero"
                body = "Spread: ${"%.1f".format(s.spreadDeg)}° (objetivo: 2,0°)"
                hint = if (s.elapsedMs > 8000L) "Probá figura 8 para acelerar" else null
                progressFrac = (2f / s.spreadDeg.coerceAtLeast(0.5f)).coerceIn(0f, 1f)
            }
            AnchorCalibrator.Phase.STABILIZING -> {
                val secs = s.stableForMs / 1000f
                title = "Estabilizando… (${"%.1f".format(secs)} s / 2,0 s)"
                body = "Spread: ${"%.2f".format(s.spreadDeg)}° (OK)"
                hint = "Mantené el celu lo más quieto posible"
                progressFrac = (s.stableForMs.toFloat() / 2000f).coerceIn(0f, 1f)
            }
            AnchorCalibrator.Phase.READY -> {
                title = "Norte calibrado ✓"
                body = "Precisión estimada: ${"%.2f".format(s.precisionEstimateDeg)}°"
                hint = "Ya podés capturar"
                progressFrac = 1f
            }
            AnchorCalibrator.Phase.READY_DEGRADED -> {
                title = "Norte calibrado (degradado) ⚠"
                body = "Precisión estimada: ${"%.1f".format(s.precisionEstimateDeg)}°"
                hint = "Spread alto en el sitio. Resultado puede tener offset."
                progressFrac = 1f
            }
        }

        var y = boxTop + 56f
        canvas.drawText(title, boxLeft + 32f, y, titlePaint)
        y += 60f
        canvas.drawText(body, boxLeft + 32f, y, bodyPaint)
        y += 48f
        if (hint != null) {
            canvas.drawText(hint, boxLeft + 32f, y, hintPaint)
            y += 40f
        }

        // Barra de progreso.
        val barLeft = boxLeft + 32f
        val barRight = boxLeft + boxWidth - 32f
        val barTop = boxTop + boxHeight - 36f
        val barBottom = barTop + 12f
        canvas.drawRoundRect(
            RectF(barLeft, barTop, barRight, barBottom),
            6f, 6f, progressBgPaint
        )
        canvas.drawRoundRect(
            RectF(barLeft, barTop, barLeft + (barRight - barLeft) * progressFrac, barBottom),
            6f, 6f, progressFgPaint
        )
    }

    private fun magLabel(acc: Int): String = when (acc) {
        3 -> "HIGH"
        2 -> "MEDIUM"
        1 -> "LOW"
        0 -> "UNRELIABLE"
        else -> "UNKNOWN($acc)"
    }
}