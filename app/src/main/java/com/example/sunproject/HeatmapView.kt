package com.example.sunproject

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.util.AttributeSet
import android.view.View
import kotlin.math.max
import kotlin.math.min

/**
 * View custom para visualizar una matriz [12 meses][24 horas locales] de
 * pérdidas en %.
 *
 * Convenciones:
 *  - Filas = meses (0 = enero arriba, 11 = diciembre abajo).
 *  - Columnas = horas locales (0 = medianoche izq, 23 = 11pm der).
 *  - Color de cada celda interpolado:
 *      0%  → transparente (no había sol o sin pérdida)
 *      hasta el máximo de la matriz → rojo intenso.
 *  - Celdas con valor 0 quedan en gris muy tenue para distinguirlas
 *    visualmente (significa "no había sol en ese slot" o "el panel
 *    estaba orientado al revés en ese slot").
 *
 * No es interactiva (sin touch). En C.4 se va a poder tocar una celda
 * para ver el detalle.
 */
class HeatmapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var matrix: Array<DoubleArray>? = null
    private var matrixMax: Double = 0.0

    private val cellPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        color = Color.argb(60, 255, 255, 255)
        strokeWidth = 1f
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 22f
        isAntiAlias = true
    }
    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 28f
        isAntiAlias = true
        isFakeBoldText = true
    }
    private val legendPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }

    private val tmpRect = Rect()

    /**
     * Setea la matriz a renderizar. Dispara invalidate().
     * Espera dimensiones [12][24]; otras dimensiones se aceptan y se
     * adaptan en tiempo de render.
     */
    fun setMatrix(matrix: Array<DoubleArray>) {
        this.matrix = matrix
        // Cap inferior del max en 1% para que un perfil sin obstáculos
        // no muestre celdas que ocupan toda la escala por ruido numérico.
        matrixMax = max(1.0, matrix.flatMap { it.toList() }.maxOrNull() ?: 0.0)
        invalidate()
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec)
        // Aspect ratio fijo: queremos que sea más ancho que alto.
        // 24 horas × 12 meses = relación natural ~2:1 + márgenes.
        val width = measuredWidth
        val height = (width * 0.65f).toInt().coerceAtLeast(measuredHeight)
        setMeasuredDimension(width, height)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val m = matrix ?: return

        val rows = m.size
        val cols = if (rows > 0) m[0].size else 0
        if (rows == 0 || cols == 0) return

        // Layout interno:
        //   [marginTop=40] título
        //   [marginTop=40] horas-labels arriba
        //   [grilla...]
        //   [marginBottom=40] meses-labels izq + grilla
        //   [marginBottom=60] leyenda de color
        val padding = 16f
        val titleHeight = 40f
        val hourLabelHeight = 30f
        val monthLabelWidth = 50f
        val legendHeight = 60f

        val gridLeft = padding + monthLabelWidth
        val gridTop = padding + titleHeight + hourLabelHeight
        val gridRight = width - padding
        val gridBottom = height - padding - legendHeight

        val cellW = (gridRight - gridLeft) / cols
        val cellH = (gridBottom - gridTop) / rows

        // Título
        canvas.drawText("Pérdida por mes y hora local", padding, padding + 30f, titlePaint)

        // Labels horas (cada 4 horas)
        labelPaint.textAlign = Paint.Align.CENTER
        for (h in 0 until cols step 4) {
            val x = gridLeft + cellW * (h + 0.5f)
            canvas.drawText(
                "%02d".format(h),
                x,
                gridTop - 8f,
                labelPaint
            )
        }

        // Labels meses (todos los 12)
        labelPaint.textAlign = Paint.Align.RIGHT
        for (mi in 0 until rows) {
            val y = gridTop + cellH * (mi + 0.5f) + 8f
            canvas.drawText(
                MONTH_LABELS_SHORT[mi % MONTH_LABELS_SHORT.size],
                gridLeft - 6f,
                y,
                labelPaint
            )
        }

        // Celdas
        for (mi in 0 until rows) {
            for (hi in 0 until cols) {
                val v = m[mi][hi]
                cellPaint.color = colorForValue(v, matrixMax)

                val l = gridLeft + cellW * hi
                val t = gridTop + cellH * mi
                val r = l + cellW
                val b = t + cellH

                canvas.drawRect(l, t, r, b, cellPaint)
                canvas.drawRect(l, t, r, b, borderPaint)
            }
        }

        // Leyenda: gradient bar de min (verde tenue) a max (rojo intenso)
        val legendLeft = gridLeft
        val legendRight = gridRight
        val legendTop = gridBottom + 16f
        val legendBottom = legendTop + 18f
        val legendSteps = 50
        val legendStepW = (legendRight - legendLeft) / legendSteps
        for (i in 0 until legendSteps) {
            val frac = i.toDouble() / (legendSteps - 1)
            legendPaint.color = colorForValue(frac * matrixMax, matrixMax)
            canvas.drawRect(
                legendLeft + i * legendStepW,
                legendTop,
                legendLeft + (i + 1) * legendStepW,
                legendBottom,
                legendPaint
            )
        }
        // Texto debajo de la leyenda: 0% .. max%
        labelPaint.textAlign = Paint.Align.LEFT
        canvas.drawText("0%", legendLeft, legendBottom + 24f, labelPaint)
        labelPaint.textAlign = Paint.Align.RIGHT
        canvas.drawText("${"%.0f".format(matrixMax)}%", legendRight, legendBottom + 24f, labelPaint)
    }

    /**
     * Mapeo de valor (en %) a color usando una rampa verde-tenue → amarillo
     * → naranja → rojo intenso.
     */
    private fun colorForValue(value: Double, maxValue: Double): Int {
        if (value <= 0.001) {
            return Color.argb(15, 255, 255, 255)  // gris muy tenue
        }
        val frac = (value / maxValue).coerceIn(0.0, 1.0).toFloat()

        // Rampa simple en HSV: hue de 120 (verde) a 0 (rojo).
        // Saturation y alpha crecen con el valor para que pérdidas
        // bajas se vean más sutiles.
        val hue = 120f * (1f - frac)
        val sat = 0.5f + 0.5f * frac
        val value01 = 0.7f + 0.3f * frac
        val alpha = (60 + 195 * frac).toInt().coerceIn(0, 255)

        val rgb = Color.HSVToColor(floatArrayOf(hue, sat, value01))
        return Color.argb(
            alpha,
            Color.red(rgb),
            Color.green(rgb),
            Color.blue(rgb)
        )
    }

    companion object {
        private val MONTH_LABELS_SHORT = listOf(
            "E", "F", "M", "A", "M", "J",
            "J", "A", "S", "O", "N", "D"
        )
    }
}