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
import android.os.SystemClock
import android.util.Log

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
    private val textPaint = Paint(
        Paint.ANTI_ALIAS_FLAG or Paint.SUBPIXEL_TEXT_FLAG or Paint.DITHER_FLAG
    ).apply {
        color = Color.WHITE
        textSize = 32f
        textAlign = Paint.Align.CENTER
        isLinearText = true
        setShadowLayer(5f, 2f, 2f, Color.BLACK)
    }

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

    private val switchConfirmFrames = 6
    private val switchMarginPx = 36f
    private val activeKeepRadiusPx = 52f
    private val activeReleaseRadiusPx = 128f
    private val activeSwitchCooldownMs = 250L

    private var lastSwitchAtMs = 0L
    private var lastTargetLogMs = 0L
    private data class Vec3(val x: Float, val y: Float, val z: Float)

    private var camForward = Vec3(0f, 1f, 0f)
    private var camRight = Vec3(1f, 0f, 0f)
    private var camUp = Vec3(0f, 0f, 1f)

    private var latchedProjectionYawDeg = 0f


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

        buildCameraBasis(azimuth, pitch, roll)

        Log.d(
            "SunGuidePose",
            "az=${"%.2f".format(azimuth)} pitch=${"%.2f".format(pitch)} roll=${"%.2f".format(roll)} zenith=$zenithMode"
        )

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

        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR)

        drawHorizon(canvas)
        drawAzimuthDivisions(canvas)
        drawCapturePoints(canvas)
        drawAzimuthLabels(canvas)
        drawReticle(canvas)
    }

    private fun projectToScreen(targetAzimuth: Float, targetPitch: Float): Pair<Float, Float>? {
        if (zenithMode && targetPitch >= 80f) {
            return Pair(width / 2f, height / 2f)
        }

        val dir = worldDir(targetAzimuth, targetPitch)

        val camX = dot(dir, camRight)
        val camY = dot(dir, camUp)
        val camZ = dot(dir, camForward)

        if (camZ <= 0f) return null

        val nx = (camX / camZ) / tanHorizontalFovHalf
        val ny = (camY / camZ) / tanVerticalFovHalf
        if (targetPitch == 45f || targetPitch >= 80f || targetPitch == 0f) {
            Log.d(
                "SunProjectToScreen",
                "target=$targetAzimuth/$targetPitch camZ=${"%.3f".format(camZ)} " +
                        "camX=${"%.3f".format(camX)} camY=${"%.3f".format(camY)} " +
                        "nx=${"%.3f".format(nx)} ny=${"%.3f".format(ny)} zenith=$zenithMode"
            )
        }
        if (abs(nx) > 1f || abs(ny) > 1f) return null

        val cx = width / 2f
        val cy = height / 2f

        val x = cx + nx * cx
        val y = cy - ny * cy

        return Pair(x, y)
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

    private fun drawAzimuthDivisions(canvas: Canvas) {
        for (az in 0 until 360 step 30) {
            projectToScreen(az.toFloat(), 0f)?.let { (x, y) ->
                val deltaAz = abs((az.toFloat() - cameraAzimuth + 540f) % 360f - 180f)
                val alpha = ((1.0f - (deltaAz / 95f)).coerceIn(0f, 1f) * 180f).toInt()

                divisionPaint.alpha = alpha
                val halfLen = if (az % 90 == 0) 14f else 9f
                canvas.drawLine(x, y - halfLen, x, y + halfLen, divisionPaint)
            }
        }
    }

    private fun drawAzimuthLabels(canvas: Canvas) {
        val drawnLabels = mutableListOf<Rect>()

        for (az in 0 until 360 step 30) {
            projectToScreen(az.toFloat(), 0f)?.let { (x, y) ->
                val label = when (az) {
                    0 -> "N"
                    90 -> "E"
                    180 -> "S"
                    270 -> "O"
                    else -> "$az°"
                }

                val deltaAz = abs((az.toFloat() - cameraAzimuth + 540f) % 360f - 180f)
                val alpha = ((1.0f - (deltaAz / 95f)).coerceIn(0f, 1f) * 255f).toInt()
                textPaint.alpha = alpha

                textPaint.getTextBounds(label, 0, label.length, textMeasureRect)

                val rect = Rect(
                    (x - textMeasureRect.width() / 2f - 8f).toInt(),
                    (y + 44f).toInt(),
                    (x + textMeasureRect.width() / 2f + 8f).toInt(),
                    (y + 82f).toInt()
                )

                val overlaps = drawnLabels.any { Rect.intersects(it, rect) }
                if (!overlaps) {
                    canvas.drawText(label, x, y + 72f, textPaint)
                    drawnLabels.add(rect)
                }
            }
        }
    }

    private fun drawCapturePoints(canvas: Canvas) {
        val activePoint = activePoint
        val ringRadius = 40f

        val pointsToDraw = if (zenithMode) {
            capturePoints.filter { it.pitch >= 80f || it.isCaptured }
        } else {
            capturePoints
        }

        pointsToDraw.forEach { point ->
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

    private data class VisibleCandidate(
        val point: CapturePoint,
        val screenX: Float,
        val screenY: Float,
        val distPx: Float
    )

    private fun screenDistancePx(point: CapturePoint): Float? {
        val proj = projectToScreen(point.azimuth, point.pitch) ?: return null
        val cx = width / 2f
        val cy = height / 2f
        val dx = proj.first - cx
        val dy = proj.second - cy
        return hypot(dx.toDouble(), dy.toDouble()).toFloat()
    }

    private fun visibleCandidates(candidates: List<CapturePoint>): List<VisibleCandidate> {
        val cx = width / 2f
        val cy = height / 2f

        return candidates
            .mapNotNull { point ->
                val proj = projectToScreen(point.azimuth, point.pitch) ?: return@mapNotNull null
                val dx = proj.first - cx
                val dy = proj.second - cy
                VisibleCandidate(
                    point = point,
                    screenX = proj.first,
                    screenY = proj.second,
                    distPx = hypot(dx.toDouble(), dy.toDouble()).toFloat()
                )
            }
            .sortedBy { it.distPx }
    }

    private fun logTargetChange(prev: CapturePoint?, next: CapturePoint?, reason: String) {
        val now = SystemClock.elapsedRealtime()
        if (now - lastTargetLogMs < 120L) return
        lastTargetLogMs = now

        Log.d(
            "SunGuideTarget",
            "reason=$reason prev=${prev?.azimuth}/${prev?.pitch} next=${next?.azimuth}/${next?.pitch} " +
                    "camAz=${"%.2f".format(cameraAzimuth)} camPitch=${"%.2f".format(cameraPitch)} camRoll=${"%.2f".format(cameraRoll)}"
        )
    }
    private fun vec3(x: Float, y: Float, z: Float) = Vec3(x, y, z)

    private fun dot(a: Vec3, b: Vec3): Float =
        a.x * b.x + a.y * b.y + a.z * b.z

    private fun cross(a: Vec3, b: Vec3): Vec3 =
        Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        )

    private fun norm(v: Vec3): Float =
        hypot(v.x.toDouble(), hypot(v.y.toDouble(), v.z.toDouble())).toFloat()

    private fun normalize(v: Vec3): Vec3 {
        val n = norm(v)
        if (n < 1e-6f) return Vec3(0f, 0f, 0f)
        return Vec3(v.x / n, v.y / n, v.z / n)
    }

    private fun add(a: Vec3, b: Vec3): Vec3 =
        Vec3(a.x + b.x, a.y + b.y, a.z + b.z)

    private fun scale(v: Vec3, s: Float): Vec3 =
        Vec3(v.x * s, v.y * s, v.z * s)

    private fun worldDir(azimuthDeg: Float, altitudeDeg: Float): Vec3 {
        val az = Math.toRadians(azimuthDeg.toDouble())
        val alt = Math.toRadians(altitudeDeg.toDouble())

        val cosAlt = kotlin.math.cos(alt).toFloat()
        val xEast = (cosAlt * kotlin.math.sin(az)).toFloat()
        val yNorth = (cosAlt * kotlin.math.cos(az)).toFloat()
        val zUp = kotlin.math.sin(alt).toFloat()

        return normalize(Vec3(xEast, yNorth, zUp))
    }

    private fun buildCameraBasis(azimuthDeg: Float, pitchDeg: Float, rollDeg: Float) {
        val projectionYaw = if (zenithMode) {
            latchedProjectionYawDeg
        } else {
            azimuthDeg.also { latchedProjectionYawDeg = it }
        }

        val forward = worldDir(projectionYaw, pitchDeg)

        var right0 = cross(forward, Vec3(0f, 0f, 1f))
        if (norm(right0) < 1e-4f) right0 = Vec3(1f, 0f, 0f)
        right0 = normalize(right0)

        val up0 = normalize(cross(right0, forward))

        val r = Math.toRadians((-rollDeg).toDouble()).toFloat()
        val cosR = kotlin.math.cos(r)
        val sinR = kotlin.math.sin(r)

        camRight = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        camUp = normalize(add(scale(up0, cosR), scale(right0, -sinR)))
        camForward = forward
        Log.d(
            "SunGuideBasis",
            "camForward=(${String.format("%.3f", camForward.x)}, ${String.format("%.3f", camForward.y)}, ${String.format("%.3f", camForward.z)}) " +
                    "camUp=(${String.format("%.3f", camUp.x)}, ${String.format("%.3f", camUp.y)}, ${String.format("%.3f", camUp.z)}) " +
                    "camRight=(${String.format("%.3f", camRight.x)}, ${String.format("%.3f", camRight.y)}, ${String.format("%.3f", camRight.z)})"
        )
    }
    private fun stageCandidates(): List<CapturePoint> {
        val uncaptured = capturePoints.filter { !it.isCaptured }

        return when {
            uncaptured.any { it.pitch == 0f } -> uncaptured.filter { it.pitch == 0f }
            uncaptured.any { it.pitch == 45f } -> uncaptured.filter { it.pitch == 45f }
            else -> uncaptured.filter { it.pitch >= 80f }
        }
    }
    fun getStageDebugString(): String {
        val h0 = capturePoints.count { !it.isCaptured && it.pitch == 0f }
        val h45 = capturePoints.count { !it.isCaptured && it.pitch == 45f }
        val z0 = capturePoints.count { !it.isCaptured && it.pitch >= 80f }
        return "H0=$h0 H45=$h45 Z0=$z0 active=${activePoint?.azimuth}/${activePoint?.pitch} zenithMode=$zenithMode"
    }

    private fun updateActivePoint() {
        val candidates = stageCandidates()

        zenithMode = candidates.isNotEmpty() && candidates.all { it.pitch >= 80f }

        if (candidates.isEmpty()) {
            activePoint = null
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        val now = SystemClock.elapsedRealtime()
        val visible = visibleCandidates(candidates)
        val bestVisible = visible.firstOrNull()

        val current = activePoint?.takeUnless { it.isCaptured }
        if (current == null) {
            val initial = bestVisible?.point ?: candidates.minByOrNull { candidateScore(it) }
            activePoint = initial
            pendingActivePoint = null
            pendingActiveFrames = 0
            lastSwitchAtMs = now
            logTargetChange(null, initial, "init")
            return
        }

        val currentDist = screenDistancePx(current)
        val bestPoint = bestVisible?.point ?: candidates.minByOrNull { candidateScore(it) }
        val bestDist = bestVisible?.distPx ?: candidateScore(bestPoint!!)

        if (bestPoint == null) return

        if (current == bestPoint) {
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        if (currentDist != null && currentDist <= activeKeepRadiusPx) {
            pendingActivePoint = null
            pendingActiveFrames = 0
            return
        }

        val currentScore = currentDist ?: candidateScore(current)
        val bestScore = bestDist
        val margin = if (zenithMode) 14f else switchMarginPx

        val immediateSwitch =
            (currentDist == null || currentScore >= activeReleaseRadiusPx) &&
                    (now - lastSwitchAtMs) >= activeSwitchCooldownMs

        if (immediateSwitch) {
            val prev = activePoint
            activePoint = bestPoint
            pendingActivePoint = null
            pendingActiveFrames = 0
            lastSwitchAtMs = now
            logTargetChange(prev, bestPoint, "release")
            return
        }

        if (bestScore + margin < currentScore && (now - lastSwitchAtMs) >= activeSwitchCooldownMs) {
            if (pendingActivePoint == bestPoint) {
                pendingActiveFrames++
            } else {
                pendingActivePoint = bestPoint
                pendingActiveFrames = 1
            }

            if (pendingActiveFrames >= switchConfirmFrames) {
                val prev = activePoint
                activePoint = bestPoint
                pendingActivePoint = null
                pendingActiveFrames = 0
                lastSwitchAtMs = now
                logTargetChange(prev, bestPoint, "better")
            }
        } else {
            pendingActivePoint = null
            pendingActiveFrames = 0
        }
        Log.d("SunGuideStage", "update ${getStageDebugString()}")
    }
}