package com.example.sunproject

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.PorterDuff
import android.graphics.Rect
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.abs
import kotlin.math.tan

class GuideView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    // --- Pinceles y Propiedades ---
    private val horizonPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.CYAN; strokeWidth = 5f; style = Paint.Style.STROKE }
    private val divisionPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.CYAN; strokeWidth = 3f; style = Paint.Style.STROKE }
    private val captureRingPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; strokeWidth = 8f; style = Paint.Style.STROKE }
    private val activeRingPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.YELLOW; strokeWidth = 10f; style = Paint.Style.STROKE }
    private val capturedPointPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.GREEN; style = Paint.Style.FILL }
    private val reticlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; style = Paint.Style.FILL }
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 32f; textAlign = Paint.Align.CENTER; setShadowLayer(5f, 2f, 2f, Color.BLACK) }

    private val horizonPath = android.graphics.Path()
    private val textMeasureRect = Rect()

    private val capturePoints = mutableListOf<CapturePoint>()
    private var cameraAzimuth: Float = 0f
    private var cameraPitch: Float = 0f
    private var cameraRoll: Float = 0f

    private var horizontalFov = 0f
    private var verticalFov = 0f
    private var tanHorizontalFovHalf = 0f
    private var tanVerticalFovHalf = 0f
    private var activePoint: CapturePoint? = null


    init {
        setupCapturePoints()
    }

    fun setCameraFov(fov: Float) {
        this.horizontalFov = fov
        if (width > 0 && height > 0) {
            onSizeChanged(width, height, width, height)
        }
    }

    data class CapturePoint(val azimuth: Float, val pitch: Float, var isCaptured: Boolean = false)

    fun updateOrientation(azimuth: Float, pitch: Float, roll: Float) {
        this.cameraAzimuth = azimuth
        this.cameraPitch = pitch
        this.cameraRoll = roll
        updateActivePoint()
        invalidate()
    }

    fun getActiveCapturePoint(): CapturePoint? = activePoint

    fun resetAllCaptured() {
        capturePoints.forEach { it.isCaptured = false }
        invalidate()
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        if (w > 0 && h > 0 && horizontalFov > 0) {
            verticalFov = horizontalFov * (h.toFloat() / w.toFloat())
            tanHorizontalFovHalf = tan(Math.toRadians(horizontalFov / 2.0)).toFloat()
            tanVerticalFovHalf = tan(Math.toRadians(verticalFov / 2.0)).toFloat()
            invalidate()
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (tanVerticalFovHalf == 0f) return

        val centerX = width / 2f
        val centerY = height / 2f

        canvas.save()
        canvas.rotate(-cameraRoll, centerX, centerY)

        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR)
        drawHorizon(canvas)
        drawAzimuthDivisions(canvas)
        drawAzimuthLabels(canvas)
        drawCapturePoints(canvas)

        canvas.restore()

        drawReticle(canvas)
    }

    private fun projectToScreen(targetAzimuth: Float, targetPitch: Float): Pair<Float, Float>? {
        val deltaAzimuth = (targetAzimuth - cameraAzimuth + 540) % 360 - 180
        val deltaPitch = targetPitch - cameraPitch

        val projectedX = tan(Math.toRadians(deltaAzimuth.toDouble())).toFloat()
        val projectedY = tan(Math.toRadians(deltaPitch.toDouble())).toFloat()

        val screenX = (width / 2f) * (1 + projectedX / tanHorizontalFovHalf)
        val screenY = (height / 2f) * (1 - projectedY / tanVerticalFovHalf)

        if (abs(deltaAzimuth) > 90) return null
        if (screenX.isFinite() && screenY.isFinite()) return Pair(screenX, screenY)

        return null
    }

    private fun setupCapturePoints() {
        capturePoints.clear()
        for (az in 0 until 360 step 30) { capturePoints.add(CapturePoint(az.toFloat(), 0f)) }
        for (az in 0 until 360 step 45) { capturePoints.add(CapturePoint(az.toFloat(), 45f)) }
        capturePoints.add(CapturePoint(0f, 88f))
    }

    private fun drawHorizon(canvas: Canvas) {
        horizonPath.reset()
        var firstPoint = true
        for (az in -180..540 step 5) {
            projectToScreen(az.toFloat(), 0f)?.let { (x, y) ->
                if (firstPoint) horizonPath.moveTo(x, y) else horizonPath.lineTo(x, y)
                firstPoint = false
            }
        }
        canvas.drawPath(horizonPath, horizonPaint)
    }

    // CORREGIDO: Lógica para dibujar las divisiones cada 10°
    private fun drawAzimuthDivisions(canvas: Canvas) {
        for (az in 0 until 360 step 10) {
            projectToScreen(az.toFloat(), 0f)?.let { (x, y) ->
                val deltaAz = abs((az.toFloat() - cameraAzimuth + 540) % 360 - 180)
                val alpha = (1.0f - (deltaAz / 90f)).coerceIn(0f, 1f) * 255
                divisionPaint.alpha = alpha.toInt()

                // Línea larga para múltiplos de 30
                if (az % 30 == 0) {
                    canvas.drawLine(x, y - 20, x, y + 20, divisionPaint)
                } else { // Línea corta para los demás (10, 20, 40, 50...)
                    canvas.drawLine(x, y - 10, x, y + 10, divisionPaint)
                }
            }
        }
    }

    private fun drawAzimuthLabels(canvas: Canvas) {
        val drawnLabels = mutableListOf<Rect>()
        for (az in 0 until 360 step 15) {
            projectToScreen(az.toFloat(), 0f)?.let { (x, y) ->
                val label = when {
                    az == 0 -> "N"
                    az == 90 -> "E"
                    az == 180 -> "S"
                    az == 270 -> "O"
                    az % 30 == 0 -> "$az°"
                    else -> null
                }

                if (label != null) {
                    val deltaAz = abs((az.toFloat() - cameraAzimuth + 540) % 360 - 180)
                    val alpha = (1.0f - (deltaAz / 90f)).coerceIn(0f, 1f) * 255
                    textPaint.alpha = alpha.toInt()

                    textPaint.getTextBounds(label, 0, label.length, textMeasureRect)
                    // CORREGIDO: Ajustamos el espaciado para evitar superposiciones
                    val labelRect = Rect((x - textMeasureRect.width()/2).toInt(), (y + 45).toInt(), (x + textMeasureRect.width()/2).toInt(), (y + 80).toInt())

                    var canDraw = true
                    for (drawnRect in drawnLabels) {
                        if (Rect.intersects(labelRect, drawnRect)) {
                            canDraw = false
                            break
                        }
                    }
                    if (canDraw) {
                        // CORREGIDO: Posición Y ajustada para dibujar DEBAJO de los anillos
                        canvas.drawText(label, x, y + 75f, textPaint)
                        drawnLabels.add(labelRect)
                    }
                }
            }
        }
    }

    private fun drawCapturePoints(canvas: Canvas) {
        val activePoint = activePoint
        val ringRadius = 40f

        capturePoints.forEach { point ->
            projectToScreen(point.azimuth, point.pitch)?.let { (x, y) ->
                if (point.isCaptured) {
                    canvas.drawCircle(x, y, ringRadius, capturedPointPaint)
                } else {
                    val paint = if (point == activePoint) activeRingPaint else captureRingPaint
                    canvas.drawCircle(x, y, ringRadius, paint)
                }
            }
        }
    }

    private fun drawReticle(canvas: Canvas) {
        val centerX = width / 2f
        val centerY = height / 2f
        val reticleRadius = 35f
        canvas.drawCircle(centerX, centerY, reticleRadius, reticlePaint)

        val azimuthText = "${cameraAzimuth.toInt()}°"
        textPaint.alpha = 200
        textPaint.textAlign = Paint.Align.LEFT
        canvas.drawText(azimuthText, centerX + 75, centerY + (textPaint.textSize / 3), textPaint)
        textPaint.textAlign = Paint.Align.CENTER
    }
    private fun absAngleDiff(a: Float, b: Float): Float {
        var d = (a - b) % 360f
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return kotlin.math.abs(d)
    }

    private fun updateActivePoint() {
        val candidates = capturePoints.filter { !it.isCaptured }
        if (candidates.isEmpty()) {
            activePoint = null
            return
        }

        val cx = width / 2f
        val cy = height / 2f

        var best: CapturePoint? = null
        var bestScore = Float.MAX_VALUE

        // 1) Preferimos candidatos VISIBLES (projectToScreen != null) y más cerca del centro
        for (p in candidates) {
            val proj = projectToScreen(p.azimuth, p.pitch) ?: continue
            val dx = proj.first - cx
            val dy = proj.second - cy
            val dist2 = dx * dx + dy * dy
            if (dist2 < bestScore) {
                bestScore = dist2
                best = p
            }
        }

        // 2) Fallback: si ninguno es visible, elegimos por error angular
        if (best == null) {
            bestScore = Float.MAX_VALUE
            for (p in candidates) {
                val dAz = absAngleDiff(p.azimuth, cameraAzimuth)
                val dPi = kotlin.math.abs(p.pitch - cameraPitch)
                val score = dAz + (2f * dPi)
                if (score < bestScore) {
                    bestScore = score
                    best = p
                }
            }
        }

        activePoint = best
    }
}