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
    // FIX BUG #2 (2026-05-16): textSize se inicializa en init{} usando
    // density para que sea consistente en cualquier densidad de pantalla.
    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        typeface = Typeface.create(Typeface.SANS_SERIF, Typeface.BOLD)
    }
    private val bodyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        typeface = Typeface.MONOSPACE
    }
    private val hintPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(255, 255, 220, 100)  // amarillo suave
        typeface = Typeface.create(Typeface.SANS_SERIF, Typeface.ITALIC)
    }

    init {
        val density = resources.displayMetrics.density
        titlePaint.textSize = 18f * density   // ~48px en xhdpi
        bodyPaint.textSize = 14f * density    // ~36px en xhdpi
        hintPaint.textSize = 12f * density    // ~32px en xhdpi
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

        // FIX BUG #2 (2026-05-16): todas las dimensiones fijas se convierten
        // a dp multiplicando por density. boxHeight pasa de 280px crudo
        // (que en xxxhdpi era una caja chica) a 140dp ≈ proporción consistente
        // en cualquier densidad. Lo mismo para corner radius, padding interno,
        // y la barra de progreso.
        val density = resources.displayMetrics.density

        // Layout: caja centrada horizontalmente, en el tercio superior.
        val boxWidth = width * 0.88f
        val boxLeft = (width - boxWidth) / 2f
        val boxTop = height * 0.12f
        val boxHeight = 140f * density
        val cornerRadius = 12f * density
        val box = RectF(boxLeft, boxTop, boxLeft + boxWidth, boxTop + boxHeight)
        canvas.drawRoundRect(box, cornerRadius, cornerRadius, bgPaint)

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
                // Mapeo logarítmico-like al rango visible: spread=20° → 0.3, spread=10° → 0.5,
                // spread=5° → 0.7, spread=2° → 1.0. Da feedback visual durante toda la
                // fase de calibración, no solo al final.
                progressFrac = (1f - (s.spreadDeg - 2f) / 30f).coerceIn(0f, 1f)
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

        val sidePad = 16f * density
        val titleLineH = 30f * density
        val bodyLineH = 24f * density
        val hintLineH = 20f * density

        var y = boxTop + 28f * density
        canvas.drawText(title, boxLeft + sidePad, y, titlePaint)
        y += titleLineH
        canvas.drawText(body, boxLeft + sidePad, y, bodyPaint)
        y += bodyLineH
        if (hint != null) {
            canvas.drawText(hint, boxLeft + sidePad, y, hintPaint)
            y += hintLineH
        }

        // Barra de progreso.
        val barLeft = boxLeft + sidePad
        val barRight = boxLeft + boxWidth - sidePad
        val barTop = boxTop + boxHeight - 18f * density
        val barBottom = barTop + 6f * density
        val barCorner = 3f * density
        canvas.drawRoundRect(
            RectF(barLeft, barTop, barRight, barBottom),
            barCorner, barCorner, progressBgPaint
        )
        canvas.drawRoundRect(
            RectF(barLeft, barTop, barLeft + (barRight - barLeft) * progressFrac, barBottom),
            barCorner, barCorner, progressFgPaint
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