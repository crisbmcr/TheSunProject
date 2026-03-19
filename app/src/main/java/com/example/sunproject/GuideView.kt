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
import kotlin.math.hypot

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

    private var zenithMode: Boolean = false

    private var pendingActivePoint: CapturePoint? = null
    private var pendingActiveFrames: Int = 0

    private val switchConfirmFrames = 4
    private val switchMarginPx = 28f
    private val centerLockRadiusPx = 72f

    init {
        setupCapturePoints()
    }

    fun setCameraFov(fov: Float) {
        this.horizontalFov = fov
        if (width > 0 && height > 0) {
            onSizeChanged(width, height, width, height)
        }
    }

    fun setZenithMode(enabled: Boolean) {
        if (zenithMode != enabled) {
            zenithMode = enabled
            invalidate()
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
    fun getCapturePlanSize(): Int = capturePoints.size

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
        val appliedRoll = if (zenithMode) cameraRoll * 0.18f else cameraRoll
        canvas.rotate(-appliedRoll, centerX, centerY)

        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR)
        drawHorizon(canvas)
        drawAzimuthDivisions(canvas)
        drawAzimuthLabels(canvas)
        drawCapturePoints(canvas)

        canvas.restore()

        drawReticle(canvas)
    }

    private fun projectToScreen(targetAzimuth: Float, targetPitch: Float): Pair<Float, Float>? {
        val deltaAzimuth = (targetAzimuth - cameraAzimuth + 540f) % 360f - 180f
        val deltaPitch = targetPitch - cameraPitch

        val avgPitch = ((targetPitch + cameraPitch) * 0.5f).coerceIn(0f, 89f)
        val azimuthWeight = if (zenithMode) {
            kotlin.math.cos(Math.toRadians(avgPitch.toDouble())).toFloat().coerceIn(0.08f, 0.35f)
        } else {
            1f
        }

        val effectiveDeltaAzimuth = deltaAzimuth * azimuthWeight

        val projectedX = tan(Math.toRadians(effectiveDeltaAzimuth.toDouble())).toFloat()
        val projectedY = tan(Math.toRadians(deltaPitch.toDouble())).toFloat()

        if (kotlin.math.abs(projectedX) > tanHorizontalFovHalf || kotlin.math.abs(projectedY) > tanVerticalFovHalf) {
            return null
        }

        val centerX = width / 2f
        val centerY = height / 2f

        val screenX = centerX + (projectedX / tanHorizontalFovHalf) * centerX
        val screenY = centerY - (projectedY / tanVerticalFovHalf) * centerY

        return Pair(screenX, screenY)
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

    private fun candidateScore(point: CapturePoint): Float {
        val cx = width / 2f
        val cy = height / 2f

        val proj = projectToScreen(point.azimuth, point.pitch)
        return if (proj != null) {
            val dx = proj.first - cx
            val dy = proj.second - cy
            kotlin.math.hypot(dx.toDouble(), dy.toDouble()).toFloat()
        } else {
            5000f +
                    absAngleDiff(point.azimuth, cameraAzimuth) * 20f +
                    kotlin.math.abs(point.pitch - cameraPitch) * 35f
        }
    }

    private fun isNearCenter(point: CapturePoint, radiusPx: Float): Boolean {
        val cx = width / 2f
        val cy = height / 2f
        val proj = projectToScreen(point.azimuth, point.pitch) ?: return false
        val dx = proj.first - cx
        val dy = proj.second - cy
        val dist = kotlin.math.hypot(dx.toDouble(), dy.toDouble()).toFloat()
        return dist <= radiusPx
    }

    private fun updateActivePoint() {
        val candidates = capturePoints.filter { !it.isCaptured }

        if (candidates.isEmpty()) {
            activePoint = null
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        val best = candidates.minByOrNull { candidateScore(it) }

        if (best == null) {
            activePoint = null
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        val current = activePoint

        if (current == null || current.isCaptured) {
            activePoint = best
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        if (current == best) {
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        if (isNearCenter(current, centerLockRadiusPx)) {
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        val currentScore = candidateScore(current)
        val bestScore = candidateScore(best)
        val margin = if (zenithMode) 18f else switchMarginPx

        if (bestScore + margin < currentScore) {
            if (pendingActivePoint == best) {
                pendingActiveFrames++
            } else {
                pendingActivePoint = best
                pendingActiveFrames = 1
            }

            if (pendingActiveFrames >= switchConfirmFrames) {
                activePoint = best
                pendingActivePoint = null
                pendingActiveFrames = 0
            }
        } else {
            pendingActivePoint = null
            pendingActiveFrames = 0
        }
    }
}