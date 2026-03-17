package com.example.sunproject

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.math.tan

class HorizonLineView(context: Context, attrs: AttributeSet? = null) : View(context, attrs) {

    // --- Propiedades de Orientación y GPS ---
    private var azimuth: Float = 0f
    private var pitch: Float = 0f
    private var roll: Float = 0f
    private var latitude: String = "Lat: --"
    private var longitude: String = "Lon: --"
    private var altitude: String = "Alt: --"

    // --- Pinceles para dibujar ---
    private val linePaint = Paint().apply {
        color = Color.RED
        strokeWidth = 5f
        style = Paint.Style.STROKE
        isAntiAlias = true
    }
    private val horizonTextPaint = Paint().apply {
        color = Color.WHITE
        textSize = 40f
        textAlign = Paint.Align.CENTER
        isAntiAlias = true
        setShadowLayer(5f, 2f, 2f, Color.BLACK)
    }
    private val topAzimuthPaint = Paint().apply {
        color = Color.YELLOW
        textSize = 80f
        textAlign = Paint.Align.CENTER
        isAntiAlias = true
        setShadowLayer(5f, 2f, 2f, Color.BLACK)
    }
    private val bottomInfoPaint = Paint().apply {
        color = Color.WHITE
        textSize = 35f
        isAntiAlias = true
        setShadowLayer(5f, 2f, 2f, Color.BLACK)
    }

    // --- Constantes de la Cámara ---
    // Campo de visión horizontal y vertical aproximado de la cámara.
    // Estos valores son CLAVE para la precisión. 55-65 grados es un buen
    // punto de partida para el VFOV de la mayoría de los teléfonos.
    private val HORIZONTAL_FOV = 70f
    private val VERTICAL_FOV = 60f // <-- NUEVA CONSTANTE PARA PRECISIÓN VERTICAL

    // Pre-calculamos el valor de tan(VFOV / 2) para eficiencia.
    // Math.toRadians lo convierte a radianes, que es lo que espera la función tan().
    private val tanVfovBy2 = tan(Math.toRadians(VERTICAL_FOV / 2.0)).toFloat()


    fun updateOrientation(azimuth: Float, pitch: Float, roll: Float) {
        this.azimuth = azimuth
        this.pitch = pitch
        this.roll = roll
        invalidate()
    }

    fun updateGpsData(latitude: Double, longitude: Double, altitude: Double) {
        this.latitude = "Lat: %.5f".format(latitude)
        this.longitude = "Lon: %.5f".format(longitude)
        this.altitude = "Alt: %.1fm".format(altitude)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        drawTopAzimuth(canvas)
        drawBottomInfo(canvas)

        // --- CÁLCULO DE PRECISIÓN DEL HORIZONTE ---
        // Convertimos el pitch a radianes para los cálculos trigonométricos.
        val pitchInRadians = Math.toRadians(pitch.toDouble()).toFloat()

        // Esta es la fórmula de proyección físicamente precisa.
        // Mapea el ángulo de pitch a la pantalla usando la óptica de la cámara (VFOV).
        val yOffset = (height / 2f) * (tan(pitchInRadians) / tanVfovBy2)
        val yPosition = (height / 2f) - yOffset

        val rotationToApply = -roll

        canvas.save()
        canvas.rotate(rotationToApply, width / 2f, height / 2f)

        canvas.drawLine(0f, yPosition, width.toFloat(), yPosition, linePaint)
        drawAzimuthMarks(canvas, yPosition)

        canvas.restore()
    }

    private fun drawTopAzimuth(canvas: Canvas) {
        val azimuthText = "${(azimuth + 360).toInt() % 360}°"
        canvas.drawText(azimuthText, width / 2f, topAzimuthPaint.textSize + 40f, topAzimuthPaint)
    }

    private fun drawBottomInfo(canvas: Canvas) {
        val pitchText = "Pitch: ${"%.1f".format(pitch)}°"
        val rollText = "Roll: ${"%.1f".format(roll)}°"
        val bottomMargin = 20f

        bottomInfoPaint.textAlign = Paint.Align.LEFT
        canvas.drawText(latitude, 30f, height - 100f, bottomInfoPaint)
        canvas.drawText(longitude, 30f, height - 60f, bottomInfoPaint)
        canvas.drawText(altitude, 30f, height - bottomMargin, bottomInfoPaint)

        bottomInfoPaint.textAlign = Paint.Align.RIGHT
        canvas.drawText(pitchText, width - 30f, height - 60f, bottomInfoPaint)
        canvas.drawText(rollText, width - 30f, height - bottomMargin, bottomInfoPaint)
    }

    private fun drawAzimuthMarks(canvas: Canvas, yPosition: Float) {
        val pixelsPerDegree = width / HORIZONTAL_FOV
        val startAngle = (azimuth - HORIZONTAL_FOV / 2).toInt()
        val endAngle = (azimuth + HORIZONTAL_FOV / 2).toInt()

        for (angle in startAngle..endAngle) {
            if (angle % 10 == 0) {
                val normalizedAngle = (angle + 360) % 360
                var angleDifference = normalizedAngle - this.azimuth

                if (angleDifference > 180) angleDifference -= 360
                if (angleDifference < -180) angleDifference += 360

                val xPosition = (width / 2) + (angleDifference * pixelsPerDegree)

                canvas.drawLine(xPosition, yPosition - 20, xPosition, yPosition + 20, linePaint)
                canvas.drawText("$normalizedAngle°", xPosition, yPosition - 40, horizonTextPaint)
            }
        }
    }
}
