package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import com.example.sunproject.data.model.FrameRecord
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan
import kotlin.math.acos

import android.util.Log
import android.graphics.Color
import android.os.SystemClock
import com.example.sunproject.domain.atlas.ZenithTopFaceRefiner
import kotlin.math.atan2
import kotlin.math.asin
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.CameraMountCalibration
import com.example.sunproject.data.storage.CameraMountCalibrationStore

private const val ZENITH_TWIST_FALLBACK_DEG = 90f
private const val ZENITH_PITCH_OFFSET_FALLBACK_DEG = 0f
private const val ZENITH_ROLL_OFFSET_FALLBACK_DEG = 0f

private const val ZENITH_PITCH_OFFSET_MIN_DEG = -5f
private const val ZENITH_PITCH_OFFSET_MAX_DEG = 5f

private const val ZENITH_ROLL_OFFSET_MIN_DEG = -8f
private const val ZENITH_ROLL_OFFSET_MAX_DEG = 8f

private const val ZENITH_OVERLAP_MIN_ALT_DEG = 70f
private const val ZENITH_OVERLAP_MAX_ALT_DEG = 84f

private const val ZENITH_ESTIMATE_MIN_PIXELS = 700
private const val ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE = 480

private const val ZENITH_ESTIMATE_MIN_SCORE = 0.05f
private const val ZENITH_ESTIMATE_SOFT_MIN_SCORE = 0.0f
private const val ZENITH_ESTIMATE_MIN_CONFIDENCE = 0.002f

private const val ZENITH_ESTIMATE_MIN_GRADIENT = 10.0
private const val ZENITH_ESTIMATE_MIN_LUMA = 12.0
private const val ZENITH_ESTIMATE_MAX_LUMA = 245.0

private const val ZENITH_ESTIMATE_MAX_CANDIDATES = 180
private const val ZENITH_ESTIMATE_TIME_BUDGET_MS = 8000L
private const val ZENITH_ESTIMATE_PROGRESS_EVERY = 16

private const val ZENITH_ESTIMATE_PROJ_X_STEP = 4
private const val ZENITH_ESTIMATE_PROJ_Y_STEP = 2
private const val ZENITH_SCORE_X_STEP = 4
private const val ZENITH_SCORE_Y_STEP = 2

private const val ZENITH_ESTIMATE_MIN_IMPROVEMENT_OVER_FALLBACK = 0.015f
private const val ZENITH_ESTIMATE_SOFT_MIN_IMPROVEMENT_OVER_FALLBACK = 0.003f

private const val ZENITH_ESTIMATE_DISTINCT_TWIST_DEG = 20f
private const val ZENITH_ESTIMATE_DISTINCT_PITCH_DEG = 0.75f
private const val ZENITH_ESTIMATE_DISTINCT_ROLL_DEG = 1.5f

private const val ZENITH_MATRIX_SEED_MIN_RAW_PITCH_DEG = 84.5f
private const val ZENITH_MATRIX_SEED_MAX_ABS_ROLL_DEG = 3.0f
private const val ZENITH_MATRIX_SEED_MAX_FORWARD_TILT_DEG = 6.0f

private const val ZENITH_REFINER_MIN_RAW_PITCH_DEG = 88.5f
private const val ZENITH_REFINER_MAX_ABS_ROLL_DEG = 1.0f
private const val ZENITH_REFINER_MAX_ABS_ECC_ROT_DEG = 1.5f
private const val ZENITH_REFINER_MIN_ECC_SCORE = 0.25

private const val PERSISTED_MOUNT_MIN_SAMPLE_COUNT = 12
private const val PERSISTED_MOUNT_MIN_QUALITY_SCORE = 0.996f
private const val PERSISTED_MOUNT_MAX_FORWARD_TILT_DEG = 4.5f

private const val FRAME_RGB_GAIN_MIN = 0.92f
private const val FRAME_RGB_GAIN_MAX = 1.08f
private const val FRAME_GAIN_ESTIMATE_X_STEP = 6
private const val FRAME_GAIN_ESTIMATE_Y_STEP = 3
private const val FRAME_GAIN_ESTIMATE_MIN_SAMPLES = 400

private data class RgbGain(
    val r: Float,
    val g: Float,
    val b: Float
)


data class FrameFootprint(
    val minAzimuthDeg: Float,
    val maxAzimuthDeg: Float,
    val minAltitudeDeg: Float,
    val maxAltitudeDeg: Float
)

data class ZenithPoseEstimate(
    val absoluteYawDeg: Float,
    val absolutePitchDeg: Float,
    val absoluteRollDeg: Float,
    val score: Float,
    val comparedPixels: Int,
    val confidence: Float,
    val legacyTwistDeg: Float,
    val legacyPitchOffsetDeg: Float,
    val legacyRollOffsetDeg: Float
) {
    val twistDeg: Float get() = legacyTwistDeg
    val pitchOffsetDeg: Float get() = legacyPitchOffsetDeg
    val rollOffsetDeg: Float get() = legacyRollOffsetDeg
}
private data class ProjectionBasis(
    val forward: FloatArray,
    val right: FloatArray,
    val up: FloatArray
)

private data class CameraMountBasis(
    val forwardInDevice: FloatArray,
    val rightInDevice: FloatArray,
    val upInDevice: FloatArray,
    val sampleCount: Int
)

object AtlasProjector {

    fun approximateFootprint(frame: FrameRecord): FrameFootprint {
        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val centerAz = AtlasMath.normalizeAzimuthDeg(frame.measuredAzimuthDeg)
        val centerAlt = frame.measuredPitchDeg.coerceIn(0f, 90f)
        val rollAbs = abs(frame.measuredRollDeg)

        val cosAlt = cos(Math.toRadians(centerAlt.toDouble())).toFloat().coerceAtLeast(0.08f)

        val azHalf = when {
            centerAlt >= 80f -> 180f
            centerAlt >= 60f -> (
                    hfov * 0.5f * (1f / cosAlt).coerceAtMost(2.2f) +
                            (rollAbs * 0.15f).coerceAtMost(6f)
                    ).coerceIn(hfov * 0.5f, 180f)
            centerAlt >= 40f -> (
                    hfov * 0.5f * (1f / cosAlt).coerceAtMost(1.6f) +
                            (rollAbs * 0.10f).coerceAtMost(4f)
                    ).coerceIn(hfov * 0.5f, 180f)
            else -> (
                    hfov * 0.5f +
                            (rollAbs * 0.05f).coerceAtMost(2f)
                    ).coerceIn(hfov * 0.5f, 180f)
        }

        val altHalf = (
                vfov * 0.5f +
                        when {
                            centerAlt >= 80f -> 6f
                            centerAlt >= 60f -> 3f
                            else -> 0f
                        } +
                        (rollAbs * 0.08f).coerceAtMost(3f)
                ).coerceIn(vfov * 0.5f, 90f)

        val fullAzimuth = centerAlt >= 80f || azHalf >= 179.5f

        val minAlt = (centerAlt - altHalf).coerceIn(0f, 90f)
        val maxAlt = (centerAlt + altHalf).coerceIn(0f, 90f)

        val constrainedMinAlt = when {
            frame.targetPitchDeg >= 80f || centerAlt >= 80f -> max(minAlt, 68f)
            else -> minAlt
        }

        return FrameFootprint(
            minAzimuthDeg = if (fullAzimuth) -180f else centerAz - azHalf,
            maxAzimuthDeg = if (fullAzimuth) 180f else centerAz + azHalf,
            minAltitudeDeg = constrainedMinAlt,
            maxAltitudeDeg = maxAlt
        )
    }

    fun projectSingleFrameToAtlas(frame: FrameRecord, atlas: SkyAtlas) {
        val src = BitmapFactory.decodeFile(frame.originalPath) ?: return
        try {
            val frameWeight = qualityWeightForFrame(frame)
            projectBitmapToAtlas(
                frame = frame,
                src = src,
                atlas = atlas,
                frameWeight = frameWeight,
                zenithTwistDegOverride = null,
                zenithPitchOffsetDegOverride = null,
                zenithRollOffsetDegOverride = null,
                emitLogs = true
            )
        } finally {
            src.recycle()
        }
    }
    private fun storedDeviceToWorldMatrix(frame: FrameRecord): FloatArray? {
        val m00 = frame.rotationM00 ?: return null
        val m01 = frame.rotationM01 ?: return null
        val m02 = frame.rotationM02 ?: return null
        val m10 = frame.rotationM10 ?: return null
        val m11 = frame.rotationM11 ?: return null
        val m12 = frame.rotationM12 ?: return null
        val m20 = frame.rotationM20 ?: return null
        val m21 = frame.rotationM21 ?: return null
        val m22 = frame.rotationM22 ?: return null

        return floatArrayOf(
            m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22
        )
    }
    private fun baseYawDeg(frame: FrameRecord): Float {
        return frame.absAzimuthDeg ?: frame.measuredAzimuthDeg
    }

    private fun basePitchDeg(frame: FrameRecord): Float {
        return frame.absPitchDeg ?: frame.measuredPitchDeg
    }

    private fun baseRollDeg(frame: FrameRecord): Float {
        return frame.absRollDeg ?: frame.measuredRollDeg
    }
    private const val MOUNT_REF_MAX_YAW_ERR_DEG = 10f
    private const val MOUNT_REF_MAX_PITCH_ERR_DEG = 6f
    private const val MOUNT_REF_MAX_ABS_ROLL_DEG = 8f
    private const val MOUNT_REF_MIN_WEIGHT = 0.20f

    private fun angularDeltaDeg(a: Float, b: Float): Float {
        val d = ((a - b + 540f) % 360f) - 180f
        return abs(d)
    }

    private fun projectOntoPlane(v: FloatArray, normal: FloatArray): FloatArray {
        return add(v, scale(normal, -dot(v, normal)))
    }

    private fun visualFallbackBaseYawDeg(frame: FrameRecord): Float {
        return if (isZenithFrame(frame)) {
            frame.measuredAzimuthDeg
        } else {
            baseYawDeg(frame)
        }
    }

    private fun visualFallbackBasePitchDeg(frame: FrameRecord): Float {
        return if (isZenithFrame(frame)) {
            frame.measuredPitchDeg
        } else {
            basePitchDeg(frame)
        }
    }

    private fun visualFallbackBaseRollDeg(frame: FrameRecord): Float {
        return if (isZenithFrame(frame)) {
            frame.measuredRollDeg
        } else {
            baseRollDeg(frame)
        }
    }

    private fun mountReferenceWeight(frame: FrameRecord): Float {
        if (isZenithFrame(frame)) return 0f

        val yawErr = angularDeltaDeg(frame.measuredAzimuthDeg, frame.targetAzimuthDeg)
        val pitchErr = abs(frame.measuredPitchDeg - frame.targetPitchDeg)
        val rollMag = abs(frame.measuredRollDeg)

        val yawScore = 1f - (yawErr / MOUNT_REF_MAX_YAW_ERR_DEG).coerceIn(0f, 1f)
        val pitchScore = 1f - (pitchErr / MOUNT_REF_MAX_PITCH_ERR_DEG).coerceIn(0f, 1f)
        val rollScore = 1f - (rollMag / MOUNT_REF_MAX_ABS_ROLL_DEG).coerceIn(0f, 1f)

        return (yawScore * pitchScore * rollScore).coerceIn(0f, 1f)
    }
    private fun buildZenithPoseEstimateFromLegacy(
        frame: FrameRecord,
        twistDeg: Float,
        pitchOffsetDeg: Float,
        rollOffsetDeg: Float,
        score: Float,
        comparedPixels: Int,
        confidence: Float
    ): ZenithPoseEstimate {
        val baseYaw = visualFallbackBaseYawDeg(frame)
        val basePitch = visualFallbackBasePitchDeg(frame)
        val baseRoll = visualFallbackBaseRollDeg(frame)

        val normTwist = normalizeTwistDeg(twistDeg)
        val normPitch = clampZenithPitchOffsetDeg(pitchOffsetDeg)
        val normRoll = clampZenithRollOffsetDeg(rollOffsetDeg)

        val absoluteYaw = normalizeTwistDeg(baseYaw + normTwist)
        val absolutePitch = (basePitch + normPitch).coerceIn(84f, 90f)
        val absoluteRoll = baseRoll + normRoll

        return ZenithPoseEstimate(
            absoluteYawDeg = absoluteYaw,
            absolutePitchDeg = absolutePitch,
            absoluteRollDeg = absoluteRoll,
            score = score,
            comparedPixels = comparedPixels,
            confidence = confidence,
            legacyTwistDeg = normTwist,
            legacyPitchOffsetDeg = normPitch,
            legacyRollOffsetDeg = normRoll
        )
    }

    private fun buildZenithPoseEstimateFromAbsolute(
        frame: FrameRecord,
        absoluteYawDeg: Float,
        absolutePitchDeg: Float,
        absoluteRollDeg: Float,
        score: Float,
        comparedPixels: Int,
        confidence: Float
    ): ZenithPoseEstimate {
        val baseYaw = baseYawDeg(frame)
        val basePitch = basePitchDeg(frame)
        val baseRoll = baseRollDeg(frame)

        val yawAbs = normalizeTwistDeg(absoluteYawDeg)
        val rawPitchAbs = absolutePitchDeg
        val projectedPitchAbs = rawPitchAbs.coerceIn(84f, 90f)
        val rollAbs = absoluteRollDeg

        return ZenithPoseEstimate(
            absoluteYawDeg = yawAbs,
            absolutePitchDeg = rawPitchAbs,
            absoluteRollDeg = rollAbs,
            score = score,
            comparedPixels = comparedPixels,
            confidence = confidence,
            legacyTwistDeg = normalizeTwistDeg(yawAbs - baseYaw),
            legacyPitchOffsetDeg = clampZenithPitchOffsetDeg(projectedPitchAbs - basePitch),
            legacyRollOffsetDeg = clampZenithRollOffsetDeg(rollAbs - baseRoll)
        )
    }

    private data class SelectedMountBasis(
        val basis: CameraMountBasis?,
        val source: String
    )

    private fun angleBetweenDeg(a: FloatArray, b: FloatArray): Float {
        val aa = normalize(a)
        val bb = normalize(b)
        val c = dot(aa, bb).coerceIn(-1f, 1f)
        return Math.toDegrees(acos(c.toDouble())).toFloat()
    }

    private fun forwardTiltDeg(basis: CameraMountBasis): Float {
        return angleBetweenDeg(
            basis.forwardInDevice,
            floatArrayOf(0f, 0f, -1f)
        )
    }

    private fun mountQualityScore(basis: CameraMountBasis): Float {
        val idealForward = floatArrayOf(0f, 0f, -1f)
        val idealRight = floatArrayOf(1f, 0f, 0f)
        val idealUp = floatArrayOf(0f, -1f, 0f)

        val f = dot(normalize(basis.forwardInDevice), idealForward).coerceIn(-1f, 1f)
        val r = dot(normalize(basis.rightInDevice), idealRight).coerceIn(-1f, 1f)
        val u = dot(normalize(basis.upInDevice), idealUp).coerceIn(-1f, 1f)

        return ((f + r + u) / 3f).coerceIn(-1f, 1f)
    }

    private fun isGoodPersistentMountCandidate(basis: CameraMountBasis): Boolean {
        val forwardTiltDeg = angleBetweenDeg(
            basis.forwardInDevice,
            floatArrayOf(0f, 0f, -1f)
        )
        val quality = mountQualityScore(basis)

        return basis.sampleCount >= PERSISTED_MOUNT_MIN_SAMPLE_COUNT &&
                quality >= PERSISTED_MOUNT_MIN_QUALITY_SCORE &&
                forwardTiltDeg <= PERSISTED_MOUNT_MAX_FORWARD_TILT_DEG
    }

    private fun toCameraMountCalibration(basis: CameraMountBasis): CameraMountCalibration {
        return CameraMountCalibration(
            forwardInDevice = basis.forwardInDevice.copyOf(),
            rightInDevice = basis.rightInDevice.copyOf(),
            upInDevice = basis.upInDevice.copyOf(),
            sampleCount = basis.sampleCount,
            qualityScore = mountQualityScore(basis),
            updatedAtUtcMs = System.currentTimeMillis()
        )
    }

    private fun fromCameraMountCalibration(calibration: CameraMountCalibration): CameraMountBasis {
        return CameraMountBasis(
            forwardInDevice = calibration.forwardInDevice.copyOf(),
            rightInDevice = calibration.rightInDevice.copyOf(),
            upInDevice = calibration.upInDevice.copyOf(),
            sampleCount = calibration.sampleCount
        )
    }

    private fun loadPersistedMountCalibration(): CameraMountBasis? {
        return try {
            CameraMountCalibrationStore
                .load(SunProjectApp.appContext())
                ?.let(::fromCameraMountCalibration)
        } catch (t: Throwable) {
            Log.w("AtlasMatrixMount", "load persisted calibration failed", t)
            null
        }
    }

    private fun savePersistedMountCalibration(basis: CameraMountBasis) {
        if (!isGoodPersistentMountCandidate(basis)) return

        try {
            val calibration = toCameraMountCalibration(basis)
            CameraMountCalibrationStore.save(SunProjectApp.appContext(), calibration)
            Log.d(
                "AtlasMatrixMount",
                "persisted quality=${"%.4f".format(calibration.qualityScore)} " +
                        "sampleCount=${calibration.sampleCount}"
            )
        } catch (t: Throwable) {
            Log.w("AtlasMatrixMount", "save persisted calibration failed", t)
        }
    }

    private fun selectCameraMountBasis(
        sessionBasis: CameraMountBasis?,
        persistedBasis: CameraMountBasis?
    ): SelectedMountBasis {
        val sessionIsGood = sessionBasis != null && isGoodPersistentMountCandidate(sessionBasis)

        return when {
            sessionIsGood -> SelectedMountBasis(sessionBasis, "session")
            persistedBasis != null -> SelectedMountBasis(persistedBasis, "persisted")
            sessionBasis != null -> SelectedMountBasis(sessionBasis, "session_unstable")
            else -> SelectedMountBasis(null, "none")
        }
    }
    private fun transpose3x3(m: FloatArray): FloatArray {
        return floatArrayOf(
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        )
    }

    private fun mulMat3Vec(m: FloatArray, v: FloatArray): FloatArray {
        return floatArrayOf(
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
        )
    }

    private fun buildProjectionBasisFromAngles(
        yawDeg: Float,
        pitchDeg: Float,
        rollDeg: Float,
        zenithLike: Boolean,
        forceHardZenith: Boolean = false
    ): ProjectionBasis {
        val yawRad = Math.toRadians(yawDeg.toDouble()).toFloat()
        val pitchRad = Math.toRadians(pitchDeg.toDouble()).toFloat()
        val rollRad = Math.toRadians(rollDeg.toDouble()).toFloat()
        val forward = worldDirectionRad(yawRad, pitchRad)

        val right0: FloatArray
        val up0: FloatArray

        val useHardZenithBasis = forceHardZenith || (zenithLike && pitchDeg >= 89.5f)

        if (useHardZenithBasis) {
            right0 = normalize(
                floatArrayOf(
                    cos(yawRad),
                    sin(yawRad),
                    0f
                )
            )
            up0 = normalize(
                floatArrayOf(
                    -sin(yawRad),
                    cos(yawRad),
                    0f
                )
            )
        } else {
            var r = cross(forward, floatArrayOf(0f, 0f, 1f))
            if (length(r) < 1e-4f) {
                r = floatArrayOf(1f, 0f, 0f)
            }
            r = normalize(r)
            val u = normalize(cross(r, forward))
            right0 = r
            up0 = u
        }

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

        return ProjectionBasis(
            forward = forward,
            right = right,
            up = up
        )
    }

    private fun estimateCameraMountBasis(frames: List<FrameRecord>): CameraMountBasis? {
        var count = 0
        var weightSum = 0f

        var sumForwardDev = floatArrayOf(0f, 0f, 0f)
        var sumRightDev = floatArrayOf(0f, 0f, 0f)
        var sumUpDev = floatArrayOf(0f, 0f, 0f)

        frames.forEach { frame ->
            if (isZenithFrame(frame)) return@forEach

            val rWorldDevice = storedDeviceToWorldMatrix(frame) ?: return@forEach
            val rDeviceWorld = transpose3x3(rWorldDevice)

            val refWeight = mountReferenceWeight(frame)
            if (refWeight < MOUNT_REF_MIN_WEIGHT) return@forEach

            val referenceBasis = buildProjectionBasisFromAngles(
                yawDeg = frame.targetAzimuthDeg,
                pitchDeg = frame.targetPitchDeg,
                rollDeg = frame.measuredRollDeg.coerceIn(-8f, 8f),
                zenithLike = false
            )

            val forwardDev = normalize(mulMat3Vec(rDeviceWorld, referenceBasis.forward))
            val rightDev = normalize(mulMat3Vec(rDeviceWorld, referenceBasis.right))
            val upDev = normalize(mulMat3Vec(rDeviceWorld, referenceBasis.up))

            sumForwardDev = add(sumForwardDev, scale(forwardDev, refWeight))
            sumRightDev = add(sumRightDev, scale(rightDev, refWeight))
            sumUpDev = add(sumUpDev, scale(upDev, refWeight))

            weightSum += refWeight
            count++
        }

        if (count <= 0 || weightSum <= 0f) {
            Log.w("AtlasMatrixMount", "insufficientSamples=$count")
            return null
        }

        val forward = normalize(sumForwardDev)

        var right = projectOntoPlane(sumRightDev, forward)
        if (length(right) < 1e-4f) {
            right = cross(sumUpDev, forward)
        }
        if (length(right) < 1e-4f) {
            Log.w("AtlasMatrixMount", "degenerateBasis count=$count weightSum=${"%.3f".format(weightSum)}")
            return null
        }
        right = normalize(right)

        var up = cross(forward, right)
        if (length(up) < 1e-4f) {
            Log.w("AtlasMatrixMount", "degenerateUp count=$count weightSum=${"%.3f".format(weightSum)}")
            return null
        }
        up = normalize(up)

        right = normalize(cross(up, forward))

        Log.d(
            "AtlasMatrixMount",
            "samples=$count weightSum=${"%.3f".format(weightSum)} " +
                    "forwardDev=${forward.joinToString(prefix = "[", postfix = "]") { "%.3f".format(it) }} " +
                    "rightDev=${right.joinToString(prefix = "[", postfix = "]") { "%.3f".format(it) }} " +
                    "upDev=${up.joinToString(prefix = "[", postfix = "]") { "%.3f".format(it) }}"
        )

        return CameraMountBasis(
            forwardInDevice = forward,
            rightInDevice = right,
            upInDevice = up,
            sampleCount = count
        )
    }

    private fun deriveZenithPoseSeedFromMatrixTangent(
        frame: FrameRecord,
        mount: CameraMountBasis
    ): ZenithPoseEstimate? {
        val rWorldDevice = storedDeviceToWorldMatrix(frame) ?: return null

        fun projectToPlane(v: FloatArray, normal: FloatArray): FloatArray {
            return add(v, scale(normal, -dot(v, normal)))
        }

        fun buildLocalTangentBasis(forward: FloatArray): Pair<FloatArray, FloatArray> {
            val worldNorth = floatArrayOf(0f, 1f, 0f)
            val worldEast = floatArrayOf(1f, 0f, 0f)

            var northT = projectToPlane(worldNorth, forward)
            if (length(northT) < 1e-4f) {
                northT = projectToPlane(worldEast, forward)
            }
            northT = normalize(northT)

            var eastT = cross(northT, forward)
            if (length(eastT) < 1e-4f) {
                eastT = projectToPlane(worldEast, forward)
            }
            eastT = normalize(eastT)

            northT = normalize(cross(forward, eastT))
            return northT to eastT
        }

        fun safeNormalizeTangent(v: FloatArray, forward: FloatArray): FloatArray? {
            val projected = projectToPlane(v, forward)
            return if (length(projected) < 1e-4f) null else normalize(projected)
        }

        val forward = normalize(mulMat3Vec(rWorldDevice, mount.forwardInDevice))
        val right = normalize(mulMat3Vec(rWorldDevice, mount.rightInDevice))
        val up = normalize(mulMat3Vec(rWorldDevice, mount.upInDevice))

        val pitchAbsDeg = Math.toDegrees(
            asin(forward[2].coerceIn(-1f, 1f).toDouble())
        ).toFloat()

        val (northT, eastT) = buildLocalTangentBasis(forward)

        val rightT = safeNormalizeTangent(right, forward) ?: return null
        val upT = safeNormalizeTangent(up, forward) ?: return null

        val forwardAzDeg = normalizeTwistDeg(
            Math.toDegrees(
                atan2(forward[0].toDouble(), forward[1].toDouble())
            ).toFloat()
        )

        val useHardZenithBranch = shouldUseHardZenithBasis(
            frame = frame,
            pitchDeg = pitchAbsDeg,
            zenithLike = true
        )

        val tangentAzDeg = if (useHardZenithBranch) {
            normalizeTwistDeg(
                Math.toDegrees(
                    atan2(
                        (-dot(upT, eastT)).toDouble(),
                        dot(upT, northT).toDouble()
                    )
                ).toFloat()
            )
        } else {
            normalizeTwistDeg(
                Math.toDegrees(
                    atan2(
                        (-dot(rightT, northT)).toDouble(),
                        dot(rightT, eastT).toDouble()
                    )
                ).toFloat()
            )
        }

        val zeroRollBasis = buildProjectionBasisFromAngles(
            yawDeg = tangentAzDeg,
            pitchDeg = pitchAbsDeg,
            rollDeg = 0f,
            zenithLike = true,
            forceHardZenith = useHardZenithBranch
        )

        val zeroRightT = safeNormalizeTangent(zeroRollBasis.right, forward) ?: return null
        val zeroUpT = safeNormalizeTangent(zeroRollBasis.up, forward) ?: return null

        val cosR = dot(rightT, zeroRightT).coerceIn(-1f, 1f)
        val sinR = dot(rightT, zeroUpT).coerceIn(-1f, 1f)
        val residualRollDeg = Math.toDegrees(
            atan2(sinR.toDouble(), cosR.toDouble())
        ).toFloat()

        val yawAbsDeg = normalizeTwistDeg(tangentAzDeg)
        val rawPitchAbsDeg = pitchAbsDeg
        val pitchUsedDeg = rawPitchAbsDeg.coerceIn(84f, 90f)
        val rollAbsDeg = residualRollDeg

        val baseYaw = baseYawDeg(frame)
        val basePitch = basePitchDeg(frame)
        val baseRoll = baseRollDeg(frame)

        Log.d(
            "AtlasZenithMatrixSeed",
            "frame=${frame.frameId} " +
                    "yawForward=${"%.2f".format(forwardAzDeg)} " +
                    "yawAbs=${"%.2f".format(yawAbsDeg)} " +
                    "rawPitchAbs=${"%.2f".format(rawPitchAbsDeg)} " +
                    "pitchAbsClamped=${"%.2f".format(pitchUsedDeg)} " +
                    "rollAbs=${"%.2f".format(rollAbsDeg)} " +
                    "baseYaw=${"%.2f".format(baseYaw)} " +
                    "basePitch=${"%.2f".format(basePitch)} " +
                    "baseRoll=${"%.2f".format(baseRoll)} " +
                    "legacyTwist=${"%.2f".format(normalizeTwistDeg(yawAbsDeg - baseYaw))} " +
                    "legacyPitchOffset=${"%.2f".format(clampZenithPitchOffsetDeg(pitchUsedDeg - basePitch))} " +
                    "legacyRollOffset=${"%.2f".format(clampZenithRollOffsetDeg(rollAbsDeg - baseRoll))} " +
                    "seedHard=$useHardZenithBranch"
        )

        return buildZenithPoseEstimateFromAbsolute(
            frame = frame,
            absoluteYawDeg = yawAbsDeg,
            absolutePitchDeg = rawPitchAbsDeg,
            absoluteRollDeg = rollAbsDeg,
            score = Float.POSITIVE_INFINITY,
            comparedPixels = 0,
            confidence = 1f
        )
    }
    fun projectFramesToAtlas(frames: List<FrameRecord>, atlas: SkyAtlas) {
        val ordered = frames.sortedBy { it.shotIndex }
        val nonZenithFrames = ordered.filterNot { isZenithFrame(it) }
        val zenithFrames = ordered.filter { isZenithFrame(it) }
        val sessionMountBasis = estimateCameraMountBasis(nonZenithFrames)
        val persistedMountBasis = loadPersistedMountCalibration()
        val selectedMount = selectCameraMountBasis(
            sessionBasis = sessionMountBasis,
            persistedBasis = persistedMountBasis
        )
        val cameraMountBasis = selectedMount.basis

        if (sessionMountBasis != null && selectedMount.source == "session") {
            savePersistedMountCalibration(sessionMountBasis)
        }

        Log.d(
            "AtlasMatrixMount",
            "selectedSource=${selectedMount.source} " +
                    "hasSession=${sessionMountBasis != null} " +
                    "hasPersisted=${persistedMountBasis != null}"
        )
        nonZenithFrames.forEach { frame ->
            val src = BitmapFactory.decodeFile(frame.originalPath) ?: run {
                Log.w("AtlasProjector", "No se pudo decodificar ${frame.originalPath}")
                return@forEach
            }

            try {
                val frameWeight = qualityWeightForFrame(frame)

                Log.d(
                    "AtlasProjector",
                    "frame=${frame.frameId} ring=${frame.ringId} " +
                            "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                            "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                            "weight=${"%.3f".format(frameWeight)}"
                )

                projectBitmapToAtlas(
                    frame = frame,
                    src = src,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    zenithTwistDegOverride = null,
                    zenithPitchOffsetDegOverride = null,
                    zenithRollOffsetDegOverride = null,
                    emitLogs = true
                )
            } finally {
                src.recycle()
            }
        }

        zenithFrames.forEach { frame ->
            val src = BitmapFactory.decodeFile(frame.originalPath) ?: run {
                Log.w("AtlasProjector", "No se pudo decodificar ${frame.originalPath}")
                return@forEach
            }

            try {
                val frameWeight = qualityWeightForFrame(frame)

                val matrixSeed = cameraMountBasis
                    ?.let { deriveZenithPoseSeedFromMatrixTangent(frame, it) }

                val useMatrixSeed = matrixSeed != null &&
                        cameraMountBasis != null &&
                        isReliableMatrixZenithSeed(matrixSeed, cameraMountBasis)

                if (matrixSeed != null && !useMatrixSeed) {
                    val forwardTilt = if (cameraMountBasis != null) {
                        forwardTiltDeg(cameraMountBasis)
                    } else {
                        Float.NaN
                    }

                    Log.w(
                        "AtlasZenithMatrixSeed",
                        "framee=${frame.frameId} rejected " +
                                "yawAbs=${"%.2f".format(matrixSeed.absoluteYawDeg)} " +
                                "rawPitchAbs=${"%.2f".format(matrixSeed.absolutePitchDeg)} " +
                                "rollAbs=${"%.2f".format(matrixSeed.absoluteRollDeg)} " +
                                "forwardTilt=${"%.2f".format(forwardTilt)}"
                    )
                }

                val zenithPose = if (useMatrixSeed) {
                    matrixSeed!!
                } else {
                    estimateZenithPoseDeg(
                        frame = frame,
                        src = src,
                        baseAtlas = atlas,
                        frameWeight = frameWeight
                    )
                }

                Log.d(
                    "AtlasZenithPoseSeed",
                    "frame=${frame.frameId} " +
                            "source=${if (useMatrixSeed) "matrix" else "image"} " +
                            "yawAbs=${"%.2f".format(zenithPose.absoluteYawDeg)} " +
                            "pitchAbs=${"%.2f".format(zenithPose.absolutePitchDeg)} " +
                            "rollAbs=${"%.2f".format(zenithPose.absoluteRollDeg)} " +
                            "legacyTwist=${"%.2f".format(zenithPose.legacyTwistDeg)} " +
                            "legacyPitchOffset=${"%.2f".format(zenithPose.legacyPitchOffsetDeg)} " +
                            "legacyRollOffset=${"%.2f".format(zenithPose.legacyRollOffsetDeg)} " +
                            "visualBaseYaw=${"%.2f".format(visualFallbackBaseYawDeg(frame))} " +
                            "visualBasePitch=${"%.2f".format(visualFallbackBasePitchDeg(frame))} " +
                            "visualBaseRoll=${"%.2f".format(visualFallbackBaseRollDeg(frame))} " +
                            "score=${"%.5f".format(zenithPose.score)} " +
                            "compared=${zenithPose.comparedPixels} " +
                            "confidence=${"%.3f".format(zenithPose.confidence)}"
                )

                try {
                    val shouldRefine = useMatrixSeed && shouldAttemptZenithRefiner(zenithPose)

                    if (!shouldRefine) {
                        Log.w(
                            "AtlasZenithTopRefine",
                            "skip frame=${frame.frameId} " +
                                    "source=${if (useMatrixSeed) "matrix" else "image"} " +
                                    "rawPitchAbs=${"%.2f".format(zenithPose.absolutePitchDeg)} " +
                                    "rollAbs=${"%.2f".format(zenithPose.absoluteRollDeg)}"
                        )

                        projectBitmapToAtlas(
                            frame = frame,
                            src = src,
                            atlas = atlas,
                            frameWeight = frameWeight,
                            zenithAbsoluteYawDegOverride = zenithPose.absoluteYawDeg,
                            zenithAbsolutePitchDegOverride = zenithPose.absolutePitchDeg,
                            zenithAbsoluteRollDegOverride = zenithPose.absoluteRollDeg,
                            emitLogs = true
                        )
                    } else {
                        val refined = ZenithTopFaceRefiner.refineAndBlendZenithIntoAtlas(
                            atlas = atlas,
                            frame = frame,
                            srcBitmap = src,
                            frameWeight = frameWeight,
                            seedTwistDeg = zenithPose.legacyTwistDeg,
                            seedPitchOffsetDeg = zenithPose.legacyPitchOffsetDeg,
                            seedRollOffsetDeg = zenithPose.legacyRollOffsetDeg,
                            seedAbsoluteYawDeg = zenithPose.absoluteYawDeg,
                            seedAbsolutePitchDeg = zenithPose.absolutePitchDeg,
                            seedAbsoluteRollDeg = zenithPose.absoluteRollDeg
                        )

                        if (refined == null) {
                            Log.w(
                                "AtlasZenithTopRefine",
                                "frame=${frame.frameId} no refinement result; fallback to ERP zenith"
                            )

                            projectBitmapToAtlas(
                                frame = frame,
                                src = src,
                                atlas = atlas,
                                frameWeight = frameWeight,
                                zenithAbsoluteYawDegOverride = zenithPose.absoluteYawDeg,
                                zenithAbsolutePitchDegOverride = zenithPose.absolutePitchDeg,
                                zenithAbsoluteRollDegOverride = zenithPose.absoluteRollDeg,
                                emitLogs = true
                            )
                        } else {
                            Log.d(
                                "AtlasZenithTopRefine",
                                "frame=${frame.frameId} " +
                                        "initialTwist=${"%.2f".format(refined.initialTwistDeg)} " +
                                        "finalTwist=${"%.2f".format(refined.finalTwistDeg)} " +
                                        "eccRot=${"%.2f".format(refined.eccRotationDeg)} " +
                                        "eccTx=${"%.2f".format(refined.eccTxPx)} " +
                                        "eccTy=${"%.2f".format(refined.eccTyPx)} " +
                                        "eccScore=${"%.5f".format(refined.eccScore)}"
                            )
                            ZenithTopFaceRefiner.releaseTopFace(refined.alignedTopFace)
                        }
                    }
                } catch (t: Throwable) {
                    Log.e(
                        "AtlasZenithTopRefine",
                        "Fallo en refineAndBlendZenithIntoAtlas, usando fallback ERP",
                        t
                    )

                    projectBitmapToAtlas(
                        frame = frame,
                        src = src,
                        atlas = atlas,
                        frameWeight = frameWeight,
                        zenithAbsoluteYawDegOverride = zenithPose.absoluteYawDeg,
                        zenithAbsolutePitchDegOverride = zenithPose.absolutePitchDeg,
                        zenithAbsoluteRollDegOverride = zenithPose.absoluteRollDeg,
                        emitLogs = true
                    )
                }
            } finally {
                src.recycle()
            }
        }
    }

    private fun projectBitmapToAtlas(
        frame: FrameRecord,
        src: Bitmap,
        atlas: SkyAtlas,
        frameWeight: Float,
        zenithAbsoluteYawDegOverride: Float? = null,
        zenithAbsolutePitchDegOverride: Float? = null,
        zenithAbsoluteRollDegOverride: Float? = null,
        zenithTwistDegOverride: Float? = null,
        zenithPitchOffsetDegOverride: Float? = null,
        zenithRollOffsetDegOverride: Float? = null,
        emitLogs: Boolean = true,
        atlasXStep: Int = 1,
        atlasYStep: Int = 1
    ) {
        if (frameWeight <= 0f) return

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        val zenithLike = frame.targetPitchDeg >= 80f || frame.measuredPitchDeg >= 80f

        val hasAbsoluteZenithOverride =
            zenithLike &&
                    zenithAbsoluteYawDegOverride != null &&
                    zenithAbsolutePitchDegOverride != null &&
                    zenithAbsoluteRollDegOverride != null

        val effectiveZenithTwistDeg = zenithTwistDegOverride ?: ZENITH_TWIST_FALLBACK_DEG
        val effectiveZenithPitchOffsetDeg =
            zenithPitchOffsetDegOverride ?: ZENITH_PITCH_OFFSET_FALLBACK_DEG
        val effectiveZenithRollOffsetDeg =
            zenithRollOffsetDegOverride ?: ZENITH_ROLL_OFFSET_FALLBACK_DEG

        val projectionAzimuthDeg = if (zenithLike) {
            if (hasAbsoluteZenithOverride) {
                normalizeTwistDeg(zenithAbsoluteYawDegOverride!!)
            } else {
                frame.measuredAzimuthDeg + effectiveZenithTwistDeg
            }
        } else {
            frame.measuredAzimuthDeg
        }

        val projectionPitchDeg = if (zenithLike) {
            if (hasAbsoluteZenithOverride) {
                zenithAbsolutePitchDegOverride!!.coerceIn(84f, 90f)
            } else {
                (frame.measuredPitchDeg + effectiveZenithPitchOffsetDeg).coerceIn(84f, 90f)
            }
        } else {
            frame.measuredPitchDeg
        }

        val projectionRollDeg = if (zenithLike) {
            if (hasAbsoluteZenithOverride) {
                zenithAbsoluteRollDegOverride!!
            } else {
                (frame.measuredRollDeg + effectiveZenithRollOffsetDeg).coerceIn(
                    ZENITH_ROLL_OFFSET_MIN_DEG,
                    ZENITH_ROLL_OFFSET_MAX_DEG
                )
            }
        } else {
            frame.measuredRollDeg
        }

        val yawRad = Math.toRadians(
            AtlasMath.normalizeAzimuthDeg(projectionAzimuthDeg).toDouble()
        ).toFloat()

        val pitchRad = Math.toRadians(projectionPitchDeg.toDouble()).toFloat()
        val rollRad = Math.toRadians((-projectionRollDeg).toDouble()).toFloat()

        if (emitLogs) {
            Log.d(
                "AtlasPose",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "zenithLike=$zenithLike " +
                        "absOverride=$hasAbsoluteZenithOverride " +
                        "yawUsed=${"%.2f".format(projectionAzimuthDeg)} " +
                        "yawMeasured=${"%.2f".format(frame.measuredAzimuthDeg)} " +
                        "yawAbsBase=${"%.2f".format(frame.absAzimuthDeg ?: Float.NaN)} " +
                        "pitchUsed=${"%.2f".format(projectionPitchDeg)} " +
                        "pitchMeasured=${"%.2f".format(frame.measuredPitchDeg)} " +
                        "pitchAbsBase=${"%.2f".format(frame.absPitchDeg ?: Float.NaN)} " +
                        "rollUsed=${"%.2f".format(projectionRollDeg)} " +
                        "rollMeasured=${"%.2f".format(frame.measuredRollDeg)} " +
                        "rollAbsBase=${"%.2f".format(frame.absRollDeg ?: Float.NaN)} " +
                        "legacyTwist=${"%.2f".format(effectiveZenithTwistDeg)} " +
                        "legacyPitchOffset=${"%.2f".format(effectiveZenithPitchOffsetDeg)} " +
                        "legacyRollOffset=${"%.2f".format(effectiveZenithRollOffsetDeg)}"
            )
        }

        val forward = worldDirectionRad(yawRad, pitchRad)

        val right0: FloatArray
        val up0: FloatArray

        val useHardZenithBasis = shouldUseHardZenithBasis(
            frame = frame,
            pitchDeg = projectionPitchDeg,
            zenithLike = zenithLike
        )

        if (useHardZenithBasis) {
            right0 = normalize(
                floatArrayOf(
                    cos(yawRad),
                    sin(yawRad),
                    0f
                )
            )

            up0 = normalize(
                floatArrayOf(
                    -sin(yawRad),
                    cos(yawRad),
                    0f
                )
            )
        } else {
            var r = cross(forward, floatArrayOf(0f, 0f, 1f))
            if (length(r) < 1e-4f) {
                r = floatArrayOf(1f, 0f, 0f)
            }
            r = normalize(r)

            var u = cross(r, forward)
            u = normalize(u)

            right0 = r
            up0 = u
        }

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

        if (emitLogs) {
            Log.d(
                "AtlasPole",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "zenithLike=$zenithLike " +
                        "pitchUsed=${"%.2f".format(projectionPitchDeg)} " +
                        "rollUsed=${"%.2f".format(projectionRollDeg)} " +
                        "twist=${"%.2f".format(effectiveZenithTwistDeg)} " +
                        "pitchOffset=${"%.2f".format(effectiveZenithPitchOffsetDeg)} " +
                        "rollOffset=${"%.2f".format(effectiveZenithRollOffsetDeg)}"
            )
        }

        val fp = approximateFootprint(frame)
        val yTop = AtlasMath.altitudeToY(fp.maxAltitudeDeg, atlas.config)
        val yBottom = AtlasMath.altitudeToY(fp.minAltitudeDeg, atlas.config)

        val srcW = src.width
        val srcH = src.height
        val srcPixels = IntArray(srcW * srcH)
        src.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val coversAllAzimuth =
            frame.targetPitchDeg >= 80f ||
                    frame.measuredPitchDeg >= 80f ||
                    (fp.maxAzimuthDeg - fp.minAzimuthDeg) >= 359f

        val xSpans = if (coversAllAzimuth) {
            listOf(0 to (atlas.width - 1))
        } else {
            AtlasMath.splitAzimuthSpan(
                fp.minAzimuthDeg,
                fp.maxAzimuthDeg,
                atlas.config
            )
        }
        val frameRgbGain = estimateFrameRgbGain(
            atlas = atlas,
            frame = frame,
            srcPixels = srcPixels,
            srcW = srcW,
            srcH = srcH,
            forward = forward,
            right = right,
            up = up,
            tanHalfH = tanHalfH,
            tanHalfV = tanHalfV,
            xSpans = xSpans,
            yTop = yTop,
            yBottom = yBottom
        )

        if (emitLogs) {
            Log.d(
                "AtlasFrameGain",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "gainR=${"%.3f".format(frameRgbGain.r)} " +
                        "gainG=${"%.3f".format(frameRgbGain.g)} " +
                        "gainB=${"%.3f".format(frameRgbGain.b)}"
            )
        }
        if (emitLogs) {
            Log.d(
                "AtlasFootprint",
                "frame=${frame.frameId} ring=${frame.ringId} " +
                        "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                        "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                        "fpAz=${"%.2f".format(fp.minAzimuthDeg)}..${"%.2f".format(fp.maxAzimuthDeg)} " +
                        "fpAlt=${"%.2f".format(fp.minAltitudeDeg)}..${"%.2f".format(fp.maxAltitudeDeg)} " +
                        "coversAll=$coversAllAzimuth spans=${xSpans.joinToString()}"
            )
        }

        val safeXStep = atlasXStep.coerceAtLeast(1)
        val safeYStep = atlasYStep.coerceAtLeast(1)

        for ((x0, x1) in xSpans) {
            for (y in yTop..yBottom step safeYStep) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

                for (x in x0..x1 step safeXStep) {
                    val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                    val azRad = Math.toRadians(azDeg.toDouble()).toFloat()

                    val dir = worldDirectionRad(azRad, altRad)

                    val camX = dot(dir, right)
                    val camY = dot(dir, up)
                    val camZ = dot(dir, forward)

                    if (camZ <= 0f) continue

                    val nx = (camX / camZ) / tanHalfH
                    val ny = (camY / camZ) / tanHalfV

                    if (abs(nx) > 1f || abs(ny) > 1f) continue

                    val u = (((nx + 1f) * 0.5f) * (srcW - 1)).coerceIn(0f, (srcW - 1).toFloat())
                    val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).coerceIn(0f, (srcH - 1).toFloat())

                    val sampled = bilinearSampleArgb(
                        pixels = srcPixels,
                        width = srcW,
                        height = srcH,
                        fx = u,
                        fy = v
                    )

                    val color = applyRgbGain(sampled, frameRgbGain)

                    val wx = (1f - abs(nx)).coerceIn(0f, 1f)
                    val wy = (1f - abs(ny)).coerceIn(0f, 1f)

                    val localWeight = max(
                        0.001f,
                        (wx * wx) * (wy * wy)
                    )

                    val finalWeight = frameWeight * localWeight
                    atlas.blendPixel(x, y, color, finalWeight)
                }
            }
        }
    }
    private fun isZenithFrame(frame: FrameRecord): Boolean {
        return frame.ringId.equals("Z0", ignoreCase = true) ||
                frame.targetPitchDeg >= 80f ||
                frame.measuredPitchDeg >= 80f
    }

    private fun estimateZenithPoseDeg(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float
    ): ZenithPoseEstimate {
        if (frameWeight <= 0f) {
            return buildZenithPoseEstimateFromLegacy(
                frame = frame,
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG,
                score = Float.NEGATIVE_INFINITY,
                comparedPixels = 0,
                confidence = 0f
            )
        }

        val estimationSrc = downscaleForZenithEstimation(src)
        val startedAt = SystemClock.elapsedRealtime()
        val deadlineAt = startedAt + ZENITH_ESTIMATE_TIME_BUDGET_MS

        try {
            var best: ZenithPoseEstimate? = null
            val allCandidates = mutableListOf<ZenithPoseEstimate>()
            val visited = HashSet<String>()
            var evaluatedCount = 0
            var stoppedByBudget = false

            fun considerCandidate(
                twistDeg: Float,
                pitchOffsetDeg: Float,
                rollOffsetDeg: Float
            ) {
                if (stoppedByBudget) return

                if (SystemClock.elapsedRealtime() >= deadlineAt ||
                    evaluatedCount >= ZENITH_ESTIMATE_MAX_CANDIDATES
                ) {
                    stoppedByBudget = true
                    return
                }

                val normTwist = normalizeTwistDeg(twistDeg)
                val normPitch = clampZenithPitchOffsetDeg(pitchOffsetDeg)
                val normRoll = clampZenithRollOffsetDeg(rollOffsetDeg)

                val key = buildString {
                    append((normTwist * 10f).roundToInt())
                    append(':')
                    append((normPitch * 10f).roundToInt())
                    append(':')
                    append((normRoll * 10f).roundToInt())
                }

                if (!visited.add(key)) return

                val candidate = evaluateZenithPoseCandidate(
                    frame = frame,
                    src = estimationSrc,
                    baseAtlas = baseAtlas,
                    frameWeight = frameWeight,
                    twistDeg = normTwist,
                    pitchOffsetDeg = normPitch,
                    rollOffsetDeg = normRoll
                )

                evaluatedCount++
                allCandidates.add(candidate)

                if (evaluatedCount % ZENITH_ESTIMATE_PROGRESS_EVERY == 0) {
                    Log.d(
                        "AtlasZenithPoseProgress",
                        "frame=${frame.frameId} evaluated=$evaluatedCount " +
                                "bestTwist=${"%.2f".format(best?.twistDeg ?: ZENITH_TWIST_FALLBACK_DEG)} " +
                                "bestPitchOffset=${"%.2f".format(best?.pitchOffsetDeg ?: 0f)} " +
                                "bestRollOffset=${"%.2f".format(best?.rollOffsetDeg ?: 0f)} " +
                                "bestScore=${"%.4f".format(best?.score ?: Float.NEGATIVE_INFINITY)}"
                    )
                }

                if (best == null || candidate.score > best!!.score) {
                    best = candidate
                }
            }

            val fallbackCandidate = evaluateZenithPoseCandidate(
                frame = frame,
                src = estimationSrc,
                baseAtlas = baseAtlas,
                frameWeight = frameWeight,
                twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                pitchOffsetDeg = ZENITH_PITCH_OFFSET_FALLBACK_DEG,
                rollOffsetDeg = ZENITH_ROLL_OFFSET_FALLBACK_DEG
            )

            // Stage 1: coarse pitch/roll with fixed fallback twist.
            val stage1PitchOffsets = floatArrayOf(-4f, 0f, 4f)
            val stage1RollOffsets = floatArrayOf(-6f, -3f, 0f, 3f, 6f)

            loop@ for (pitchOffset in stage1PitchOffsets) {
                for (rollOffset in stage1RollOffsets) {
                    considerCandidate(
                        twistDeg = ZENITH_TWIST_FALLBACK_DEG,
                        pitchOffsetDeg = pitchOffset,
                        rollOffsetDeg = rollOffset
                    )
                    if (stoppedByBudget) break@loop
                }
            }

            val stage1Best = best ?: fallbackCandidate

            Log.d(
                "AtlasZenithPoseStage1",
                "frame=${frame.frameId} " +
                        "pitchOffset=${"%.2f".format(stage1Best.pitchOffsetDeg)} " +
                        "rollOffset=${"%.2f".format(stage1Best.rollOffsetDeg)} " +
                        "score=${"%.4f".format(stage1Best.score)} " +
                        "evaluated=$evaluatedCount stoppedByBudget=$stoppedByBudget"
            )

            // Stage 2: twist sweep on the best coarse pitch/roll.
            if (!stoppedByBudget) {
                var twist = 0f
                while (twist < 360f - 1e-3f) {
                    considerCandidate(
                        twistDeg = twist,
                        pitchOffsetDeg = stage1Best.pitchOffsetDeg,
                        rollOffsetDeg = stage1Best.rollOffsetDeg
                    )
                    if (stoppedByBudget) break
                    twist += 30f
                }
            }

            val stage2Best = best ?: stage1Best

            Log.d(
                "AtlasZenithPoseStage2",
                "frame=${frame.frameId} " +
                        "twist=${"%.2f".format(stage2Best.twistDeg)} " +
                        "pitchOffset=${"%.2f".format(stage2Best.pitchOffsetDeg)} " +
                        "rollOffset=${"%.2f".format(stage2Best.rollOffsetDeg)} " +
                        "score=${"%.4f".format(stage2Best.score)} " +
                        "evaluated=$evaluatedCount stoppedByBudget=$stoppedByBudget"
            )

            // Stage 3: small local refinement around the current best.
            val finalBest = if (!stoppedByBudget) {
                refineZenithPoseLocally(
                    seed = stage2Best,
                    considerCandidate = { twistDeg, pitchOffsetDeg, rollOffsetDeg ->
                        considerCandidate(
                            twistDeg = twistDeg,
                            pitchOffsetDeg = pitchOffsetDeg,
                            rollOffsetDeg = rollOffsetDeg
                        )
                    },
                    currentBestProvider = { best },
                    stopRequested = { stoppedByBudget }
                )
            } else {
                stage2Best
            }

            val distinctRunnerUpScore = allCandidates
                .asSequence()
                .filter { isDistinctZenithPoseCandidate(it, finalBest) }
                .maxOfOrNull { it.score }
                ?: Float.NEGATIVE_INFINITY

            val confidence = if (distinctRunnerUpScore.isFinite()) {
                finalBest.score - distinctRunnerUpScore
            } else {
                0f
            }

            val improvementOverFallback = finalBest.score - fallbackCandidate.score

            val lowPixels = finalBest.comparedPixels < ZENITH_ESTIMATE_MIN_PIXELS
            val lowScore = !finalBest.score.isFinite() || finalBest.score < ZENITH_ESTIMATE_MIN_SCORE
            val lowConfidence = confidence < ZENITH_ESTIMATE_MIN_CONFIDENCE

            val softLowScore =
                !finalBest.score.isFinite() || finalBest.score < ZENITH_ESTIMATE_SOFT_MIN_SCORE
            val softBeatsFallback =
                improvementOverFallback >= ZENITH_ESTIMATE_SOFT_MIN_IMPROVEMENT_OVER_FALLBACK

            val onSearchEdge = isZenithPoseOnSearchEdge(finalBest)
            val edgeBlocked = onSearchEdge && stoppedByBudget

            val acceptByAbsoluteGate = !lowPixels && !lowScore && !lowConfidence && !edgeBlocked
            val acceptBySoftGate = !lowPixels && !softLowScore && softBeatsFallback && !edgeBlocked
            val acceptByRelativeGain = !lowPixels &&
                    improvementOverFallback >= ZENITH_ESTIMATE_MIN_IMPROVEMENT_OVER_FALLBACK &&
                    !edgeBlocked

            Log.d(
                "AtlasZenithPoseSummary",
                "frame=${frame.frameId} evaluated=$evaluatedCount " +
                        "stoppedByBudget=$stoppedByBudget " +
                        "bestTwist=${"%.2f".format(finalBest.twistDeg)} " +
                        "bestPitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                        "bestRollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                        "score=${"%.4f".format(finalBest.score)} " +
                        "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                        "improvement=${"%.4f".format(improvementOverFallback)} " +
                        "pixels=${finalBest.comparedPixels} " +
                        "confidence=${"%.4f".format(confidence)} " +
                        "onSearchEdge=$onSearchEdge edgeBlocked=$edgeBlocked " +
                        "lowPixels=$lowPixels lowScore=$lowScore lowConfidence=$lowConfidence " +
                        "acceptAbs=$acceptByAbsoluteGate acceptSoft=$acceptBySoftGate " +
                        "acceptRel=$acceptByRelativeGain"
            )

            if (acceptByAbsoluteGate || acceptBySoftGate || acceptByRelativeGain) {
                Log.d(
                    "AtlasZenithPose",
                    "accept frame=${frame.frameId} " +
                            "twist=${"%.2f".format(finalBest.twistDeg)} " +
                            "pitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                            "rollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                            "score=${"%.4f".format(finalBest.score)} " +
                            "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                            "improvement=${"%.4f".format(improvementOverFallback)} " +
                            "confidence=${"%.4f".format(confidence)} " +
                            "acceptByAbsoluteGate=$acceptByAbsoluteGate " +
                            "acceptBySoftGate=$acceptBySoftGate " +
                            "acceptByRelativeGain=$acceptByRelativeGain"
                )
                return finalBest.copy(confidence = confidence)
            }

            Log.w(
                "AtlasZenithPose",
                "fallback frame=${frame.frameId} " +
                        "bestTwist=${"%.2f".format(finalBest.twistDeg)} " +
                        "bestPitchOffset=${"%.2f".format(finalBest.pitchOffsetDeg)} " +
                        "bestRollOffset=${"%.2f".format(finalBest.rollOffsetDeg)} " +
                        "score=${"%.4f".format(finalBest.score)} " +
                        "fallbackScore=${"%.4f".format(fallbackCandidate.score)} " +
                        "improvement=${"%.4f".format(improvementOverFallback)} " +
                        "pixels=${finalBest.comparedPixels} " +
                        "confidence=${"%.4f".format(confidence)} " +
                        "onSearchEdge=$onSearchEdge edgeBlocked=$edgeBlocked " +
                        "lowPixels=$lowPixels lowScore=$lowScore lowConfidence=$lowConfidence"
            )

            return fallbackCandidate
        } finally {
            if (estimationSrc !== src) {
                estimationSrc.recycle()
            }
        }
    }

    private fun refineZenithPoseLocally(
        seed: ZenithPoseEstimate,
        considerCandidate: (Float, Float, Float) -> Unit,
        currentBestProvider: () -> ZenithPoseEstimate?,
        stopRequested: () -> Boolean
    ): ZenithPoseEstimate {
        var current = seed

        val stepSchedule = listOf(
            Triple(6f, 1f, 1.5f),
            Triple(3f, 0.5f, 0.75f)
        )

        for ((twistStep, pitchStep, rollStep) in stepSchedule) {
            if (stopRequested()) break

            val anchor = currentBestProvider() ?: current

            val neighborhood = listOf(
                Triple(anchor.twistDeg - twistStep, anchor.pitchOffsetDeg, anchor.rollOffsetDeg),
                Triple(anchor.twistDeg + twistStep, anchor.pitchOffsetDeg, anchor.rollOffsetDeg),
                Triple(anchor.twistDeg, anchor.pitchOffsetDeg - pitchStep, anchor.rollOffsetDeg),
                Triple(anchor.twistDeg, anchor.pitchOffsetDeg + pitchStep, anchor.rollOffsetDeg),
                Triple(anchor.twistDeg, anchor.pitchOffsetDeg, anchor.rollOffsetDeg - rollStep),
                Triple(anchor.twistDeg, anchor.pitchOffsetDeg, anchor.rollOffsetDeg + rollStep)
            )

            for ((twistDeg, pitchOffsetDeg, rollOffsetDeg) in neighborhood) {
                considerCandidate(twistDeg, pitchOffsetDeg, rollOffsetDeg)
                if (stopRequested()) {
                    return currentBestProvider() ?: anchor
                }
            }

            current = currentBestProvider() ?: anchor
        }

        return currentBestProvider() ?: current
    }

    private fun isZenithPoseOnSearchEdge(candidate: ZenithPoseEstimate): Boolean {
        val eps = 1e-3f

        val pitchAtEdge =
            abs(candidate.pitchOffsetDeg - ZENITH_PITCH_OFFSET_MIN_DEG) <= eps ||
                    abs(candidate.pitchOffsetDeg - ZENITH_PITCH_OFFSET_MAX_DEG) <= eps

        val rollAtEdge =
            abs(candidate.rollOffsetDeg - ZENITH_ROLL_OFFSET_MIN_DEG) <= eps ||
                    abs(candidate.rollOffsetDeg - ZENITH_ROLL_OFFSET_MAX_DEG) <= eps

        return pitchAtEdge || rollAtEdge
    }

    private fun evaluateZenithPoseCandidate(
        frame: FrameRecord,
        src: Bitmap,
        baseAtlas: SkyAtlas,
        frameWeight: Float,
        twistDeg: Float,
        pitchOffsetDeg: Float,
        rollOffsetDeg: Float
    ): ZenithPoseEstimate {
        val candidateAtlas = SkyAtlas(baseAtlas.config)

        projectBitmapToAtlas(
            frame = frame,
            src = src,
            atlas = candidateAtlas,
            frameWeight = frameWeight,
            zenithTwistDegOverride = twistDeg,
            zenithPitchOffsetDegOverride = pitchOffsetDeg,
            zenithRollOffsetDegOverride = rollOffsetDeg,
            emitLogs = false,
            atlasXStep = ZENITH_ESTIMATE_PROJ_X_STEP,
            atlasYStep = ZENITH_ESTIMATE_PROJ_Y_STEP
        )

        val (score, comparedPixels) = scoreZenithOverlap(
            baseAtlas = baseAtlas,
            candidateAtlas = candidateAtlas,
            minAltitudeDeg = ZENITH_OVERLAP_MIN_ALT_DEG,
            maxAltitudeDeg = ZENITH_OVERLAP_MAX_ALT_DEG,
            xStep = ZENITH_SCORE_X_STEP,
            yStep = ZENITH_SCORE_Y_STEP
        )

        return buildZenithPoseEstimateFromLegacy(
            frame = frame,
            twistDeg = twistDeg,
            pitchOffsetDeg = pitchOffsetDeg,
            rollOffsetDeg = rollOffsetDeg,
            score = score,
            comparedPixels = comparedPixels,
            confidence = 0f
        )
    }
    private fun scoreZenithOverlap(
        baseAtlas: SkyAtlas,
        candidateAtlas: SkyAtlas,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        xStep: Int = 1,
        yStep: Int = 1
    ): Pair<Float, Int> {
        val yTop = AtlasMath.altitudeToY(maxAltitudeDeg, baseAtlas.config)
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, baseAtlas.config)

        val safeXStep = xStep.coerceAtLeast(1)
        val safeYStep = yStep.coerceAtLeast(1)

        var sumW = 0.0
        var sumLumA = 0.0
        var sumLumB = 0.0
        var sumGradA = 0.0
        var sumGradB = 0.0
        var count = 0

        for (y in yTop..yBottom step safeYStep) {
            for (x in 0 until baseAtlas.width step safeXStep) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val idxA = baseAtlas.index(x, y)
                val idxB = candidateAtlas.index(x, y)

                val colorA = baseAtlas.pixels[idxA]
                val colorB = candidateAtlas.pixels[idxB]

                val lumA = luma(colorA)
                val lumB = luma(colorB)
                val gradA = gradientMag(baseAtlas, x, y)
                val gradB = gradientMag(candidateAtlas, x, y)

                if (!isInformativeZenithSample(lumA, lumB, gradA, gradB)) continue

                val w = zenithSampleWeight(
                    baseWeight = baseAtlas.weightAt(x, y).toDouble(),
                    candidateWeight = candidateAtlas.weightAt(x, y).toDouble(),
                    gradA = gradA,
                    gradB = gradB
                )

                sumW += w
                sumLumA += w * lumA
                sumLumB += w * lumB
                sumGradA += w * gradA
                sumGradB += w * gradB
                count++
            }
        }

        if (count < ZENITH_ESTIMATE_MIN_PIXELS || sumW <= 0.0) {
            return Float.NEGATIVE_INFINITY to count
        }

        val meanLumA = sumLumA / sumW
        val meanLumB = sumLumB / sumW
        val meanGradA = sumGradA / sumW
        val meanGradB = sumGradB / sumW

        var numLum = 0.0
        var denLumA = 0.0
        var denLumB = 0.0

        var numGrad = 0.0
        var denGradA = 0.0
        var denGradB = 0.0

        for (y in yTop..yBottom step safeYStep) {
            for (x in 0 until baseAtlas.width step safeXStep) {
                if (!baseAtlas.hasCoverageAt(x, y) || !candidateAtlas.hasCoverageAt(x, y)) continue

                val idxA = baseAtlas.index(x, y)
                val idxB = candidateAtlas.index(x, y)

                val colorA = baseAtlas.pixels[idxA]
                val colorB = candidateAtlas.pixels[idxB]

                val lumA = luma(colorA)
                val lumB = luma(colorB)
                val gradA = gradientMag(baseAtlas, x, y)
                val gradB = gradientMag(candidateAtlas, x, y)

                if (!isInformativeZenithSample(lumA, lumB, gradA, gradB)) continue

                val w = zenithSampleWeight(
                    baseWeight = baseAtlas.weightAt(x, y).toDouble(),
                    candidateWeight = candidateAtlas.weightAt(x, y).toDouble(),
                    gradA = gradA,
                    gradB = gradB
                )

                val daLum = lumA - meanLumA
                val dbLum = lumB - meanLumB
                numLum += w * daLum * dbLum
                denLumA += w * daLum * daLum
                denLumB += w * dbLum * dbLum

                val daGrad = gradA - meanGradA
                val dbGrad = gradB - meanGradB
                numGrad += w * daGrad * dbGrad
                denGradA += w * daGrad * daGrad
                denGradB += w * dbGrad * dbGrad
            }
        }

        val lumScore =
            if (denLumA > 1e-9 && denLumB > 1e-9) {
                (numLum / sqrt(denLumA * denLumB)).toFloat()
            } else {
                -1f
            }

        val gradScore =
            if (denGradA > 1e-9 && denGradB > 1e-9) {
                (numGrad / sqrt(denGradA * denGradB)).toFloat()
            } else {
                -1f
            }

        val finalScore = 0.20f * lumScore + 0.80f * gradScore
        return finalScore to count
    }
    private fun shouldAttemptZenithRefiner(seed: ZenithPoseEstimate): Boolean {
        return seed.absolutePitchDeg >= ZENITH_REFINER_MIN_RAW_PITCH_DEG &&
                abs(seed.absoluteRollDeg) <= ZENITH_REFINER_MAX_ABS_ROLL_DEG
    }
    private fun isInformativeZenithSample(
        lumA: Double,
        lumB: Double,
        gradA: Double,
        gradB: Double
    ): Boolean {
        if (lumA <= ZENITH_ESTIMATE_MIN_LUMA || lumA >= ZENITH_ESTIMATE_MAX_LUMA) return false
        if (lumB <= ZENITH_ESTIMATE_MIN_LUMA || lumB >= ZENITH_ESTIMATE_MAX_LUMA) return false

        val maxGrad = if (gradA > gradB) gradA else gradB
        return maxGrad >= ZENITH_ESTIMATE_MIN_GRADIENT
    }

    private fun zenithSampleWeight(
        baseWeight: Double,
        candidateWeight: Double,
        gradA: Double,
        gradB: Double
    ): Double {
        val minCoverage =
            if (baseWeight < candidateWeight) baseWeight else candidateWeight

        val maxGrad =
            if (gradA > gradB) gradA else gradB

        val structureBoost = 1.0 + (maxGrad * 0.02).coerceIn(0.0, 6.0)
        val coverageBoost = 1.0 + (minCoverage * 0.15).coerceIn(0.0, 1.5)

        return structureBoost * coverageBoost
    }

    private fun downscaleForZenithEstimation(src: Bitmap): Bitmap {
        val longSide = max(src.width, src.height)
        if (longSide <= ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE) return src

        val scale = ZENITH_ESTIMATE_MAX_SRC_LONG_SIDE.toFloat() / longSide.toFloat()
        val dstW = (src.width * scale).roundToInt().coerceAtLeast(1)
        val dstH = (src.height * scale).roundToInt().coerceAtLeast(1)

        return Bitmap.createScaledBitmap(src, dstW, dstH, true)
    }

    private fun isReliableMatrixZenithSeed(
        seed: ZenithPoseEstimate,
        basis: CameraMountBasis
    ): Boolean {
        val rawPitchOk = seed.absolutePitchDeg >= ZENITH_MATRIX_SEED_MIN_RAW_PITCH_DEG
        val rollOk = abs(seed.absoluteRollDeg) <= ZENITH_MATRIX_SEED_MAX_ABS_ROLL_DEG
        val forwardTiltOk = forwardTiltDeg(basis) <= ZENITH_MATRIX_SEED_MAX_FORWARD_TILT_DEG

        return rawPitchOk && rollOk && forwardTiltOk
    }

    private fun normalizeTwistDeg(value: Float): Float {
        var v = value % 360f
        if (v < 0f) v += 360f
        return v
    }
    private fun isDistinctZenithPoseCandidate(
        a: ZenithPoseEstimate,
        b: ZenithPoseEstimate
    ): Boolean {
        val twistDiff = angleDiffDeg(a.twistDeg, b.twistDeg)
        val pitchDiff = abs(a.pitchOffsetDeg - b.pitchOffsetDeg)
        val rollDiff = abs(a.rollOffsetDeg - b.rollOffsetDeg)

        return twistDiff >= ZENITH_ESTIMATE_DISTINCT_TWIST_DEG ||
                pitchDiff >= ZENITH_ESTIMATE_DISTINCT_PITCH_DEG ||
                rollDiff >= ZENITH_ESTIMATE_DISTINCT_ROLL_DEG
    }
    private fun luma(color: Int): Double {
        return 0.299 * Color.red(color) +
                0.587 * Color.green(color) +
                0.114 * Color.blue(color)
    }

    private fun clampZenithPitchOffsetDeg(value: Float): Float {
        return value.coerceIn(
            ZENITH_PITCH_OFFSET_MIN_DEG,
            ZENITH_PITCH_OFFSET_MAX_DEG
        )
    }

    private fun clampZenithRollOffsetDeg(value: Float): Float {
        return value.coerceIn(
            ZENITH_ROLL_OFFSET_MIN_DEG,
            ZENITH_ROLL_OFFSET_MAX_DEG
        )
    }

    private fun gradientMag(atlas: SkyAtlas, x: Int, y: Int): Double {
        val xl = if (x > 0) x - 1 else x
        val xr = if (x < atlas.width - 1) x + 1 else x
        val yt = if (y > 0) y - 1 else y
        val yb = if (y < atlas.height - 1) y + 1 else y

        val gx = luma(atlas.pixels[atlas.index(xr, y)]) - luma(atlas.pixels[atlas.index(xl, y)])
        val gy = luma(atlas.pixels[atlas.index(x, yb)]) - luma(atlas.pixels[atlas.index(x, yt)])

        return kotlin.math.abs(gx) + kotlin.math.abs(gy)
    }


    private fun estimateFrameRgbGain(
        atlas: SkyAtlas,
        frame: FrameRecord,
        srcPixels: IntArray,
        srcW: Int,
        srcH: Int,
        forward: FloatArray,
        right: FloatArray,
        up: FloatArray,
        tanHalfH: Float,
        tanHalfV: Float,
        xSpans: List<Pair<Int, Int>>,
        yTop: Int,
        yBottom: Int
    ): RgbGain {
        var refRSum = 0f
        var refGSum = 0f
        var refBSum = 0f

        var movRSum = 0f
        var movGSum = 0f
        var movBSum = 0f

        var count = 0

        for ((x0, x1) in xSpans) {
            for (y in yTop..yBottom step FRAME_GAIN_ESTIMATE_Y_STEP) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altRad = Math.toRadians(altDeg.toDouble()).toFloat()

                for (x in x0..x1 step FRAME_GAIN_ESTIMATE_X_STEP) {
                    if (!atlas.hasCoverageAt(x, y)) continue

                    val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                    val azRad = Math.toRadians(azDeg.toDouble()).toFloat()

                    val dir = worldDirectionRad(azRad, altRad)

                    val camX = dot(dir, right)
                    val camY = dot(dir, up)
                    val camZ = dot(dir, forward)

                    if (camZ <= 0f) continue

                    val nx = (camX / camZ) / tanHalfH
                    val ny = (camY / camZ) / tanHalfV

                    if (abs(nx) > 1f || abs(ny) > 1f) continue

                    val u = (((nx + 1f) * 0.5f) * (srcW - 1)).coerceIn(0f, (srcW - 1).toFloat())
                    val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).coerceIn(0f, (srcH - 1).toFloat())

                    val movColor = bilinearSampleArgb(
                        pixels = srcPixels,
                        width = srcW,
                        height = srcH,
                        fx = u,
                        fy = v
                    )

                    val refColor = atlas.pixels[atlas.index(x, y)]

                    refRSum += Color.red(refColor)
                    refGSum += Color.green(refColor)
                    refBSum += Color.blue(refColor)

                    movRSum += Color.red(movColor)
                    movGSum += Color.green(movColor)
                    movBSum += Color.blue(movColor)

                    count++
                }
            }
        }

        if (count < FRAME_GAIN_ESTIMATE_MIN_SAMPLES) {
            return RgbGain(1f, 1f, 1f)
        }

        val rGain = if (movRSum > 1e-3f) {
            (refRSum / movRSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        val gGain = if (movGSum > 1e-3f) {
            (refGSum / movGSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        val bGain = if (movBSum > 1e-3f) {
            (refBSum / movBSum).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
        } else {
            1f
        }

        return RgbGain(rGain, gGain, bGain)
    }

    private fun bilinearSampleArgb(
        pixels: IntArray,
        width: Int,
        height: Int,
        fx: Float,
        fy: Float
    ): Int {
        val x0 = fx.toInt().coerceIn(0, width - 1)
        val y0 = fy.toInt().coerceIn(0, height - 1)
        val x1 = (x0 + 1).coerceIn(0, width - 1)
        val y1 = (y0 + 1).coerceIn(0, height - 1)

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val c00 = pixels[y0 * width + x0]
        val c10 = pixels[y0 * width + x1]
        val c01 = pixels[y1 * width + x0]
        val c11 = pixels[y1 * width + x1]

        val a = bilerp(
            Color.alpha(c00).toFloat(),
            Color.alpha(c10).toFloat(),
            Color.alpha(c01).toFloat(),
            Color.alpha(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val r = bilerp(
            Color.red(c00).toFloat(),
            Color.red(c10).toFloat(),
            Color.red(c01).toFloat(),
            Color.red(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val g = bilerp(
            Color.green(c00).toFloat(),
            Color.green(c10).toFloat(),
            Color.green(c01).toFloat(),
            Color.green(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        val b = bilerp(
            Color.blue(c00).toFloat(),
            Color.blue(c10).toFloat(),
            Color.blue(c01).toFloat(),
            Color.blue(c11).toFloat(),
            dx,
            dy
        ).roundToInt().coerceIn(0, 255)

        return Color.argb(a, r, g, b)
    }

    private fun bilerp(
        v00: Float,
        v10: Float,
        v01: Float,
        v11: Float,
        dx: Float,
        dy: Float
    ): Float {
        val top = v00 * (1f - dx) + v10 * dx
        val bottom = v01 * (1f - dx) + v11 * dx
        return top * (1f - dy) + bottom * dy
    }

    private fun applyRgbGain(color: Int, gain: RgbGain): Int {
        val a = Color.alpha(color)
        val r = (Color.red(color) * gain.r).roundToInt().coerceIn(0, 255)
        val g = (Color.green(color) * gain.g).roundToInt().coerceIn(0, 255)
        val b = (Color.blue(color) * gain.b).roundToInt().coerceIn(0, 255)
        return Color.argb(a, r, g, b)
    }
    private fun qualityWeightForFrame(frame: FrameRecord): Float {
        val zenithLike = frame.targetPitchDeg >= 80f || frame.measuredPitchDeg >= 80f

        val azErr = if (zenithLike) 0f
        else angleDiffDeg(frame.targetAzimuthDeg, frame.measuredAzimuthDeg)

        val pitchErr = abs(frame.targetPitchDeg - frame.measuredPitchDeg)
        val rollAbs = abs(frame.measuredRollDeg)

        val maxAzErr = when {
            zenithLike -> Float.MAX_VALUE
            frame.targetPitchDeg >= 40f -> 4.5f
            else -> 4.0f
        }
        val maxPitchErr = if (zenithLike) 2.0f else 2.75f
        val maxRollErr = if (zenithLike) 4.5f else 3.5f

        if (!zenithLike && azErr > maxAzErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=azErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "azErr=${"%.2f".format(azErr)} max=${"%.2f".format(maxAzErr)}"
            )
            return 0f
        }

        if (pitchErr > maxPitchErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=pitchErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "pitchErr=${"%.2f".format(pitchErr)} max=${"%.2f".format(maxPitchErr)}"
            )
            return 0f
        }

        if (rollAbs > maxRollErr) {
            Log.d(
                "AtlasWeight",
                "reject frame=${frame.frameId} reason=rollErr " +
                        "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                        "measured=${frame.measuredAzimuthDeg}/${frame.measuredPitchDeg}/${frame.measuredRollDeg} " +
                        "roll=${"%.2f".format(rollAbs)} max=${"%.2f".format(maxRollErr)}"
            )
            return 0f
        }

        val azScore = if (zenithLike) 1f
        else (1f - azErr / maxAzErr).coerceIn(0f, 1f)

        val pitchScore = (1f - pitchErr / maxPitchErr).coerceIn(0f, 1f)
        val rollScore = (1f - rollAbs / maxRollErr).coerceIn(0f, 1f)

        val weight = 0.15f + 0.85f * azScore * pitchScore * rollScore

        Log.d(
            "AtlasWeight",
            "accept frame=${frame.frameId} ring=${frame.ringId} " +
                    "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                    "measured=${"%.2f".format(frame.measuredAzimuthDeg)}/${"%.2f".format(frame.measuredPitchDeg)}/${"%.2f".format(frame.measuredRollDeg)} " +
                    "azErr=${"%.2f".format(azErr)} pitchErr=${"%.2f".format(pitchErr)} roll=${"%.2f".format(rollAbs)} " +
                    "weight=${"%.3f".format(weight)}"
        )

        return weight
    }
    private fun shouldUseHardZenithBasis(
        frame: FrameRecord,
        pitchDeg: Float,
        zenithLike: Boolean
    ): Boolean {
        return zenithLike && (
                frame.ringId.equals("Z0", ignoreCase = true) ||
                        frame.targetPitchDeg >= 88f ||
                        pitchDeg >= 89.5f
                )
    }
    private fun angleDiffDeg(a: Float, b: Float): Float {
        var d = AtlasMath.normalizeAzimuthDeg(a) - AtlasMath.normalizeAzimuthDeg(b)
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return abs(d)
    }

    private fun worldDirectionRad(azimuthRad: Float, altitudeRad: Float): FloatArray {
        val cosAlt = cos(altitudeRad)
        val xEast = cosAlt * sin(azimuthRad)
        val yNorth = cosAlt * cos(azimuthRad)
        val zUp = sin(altitudeRad)
        return floatArrayOf(xEast, yNorth, zUp)
    }

    private fun dot(a: FloatArray, b: FloatArray): Float =
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun cross(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        )

    private fun scale(v: FloatArray, s: Float): FloatArray =
        floatArrayOf(v[0] * s, v[1] * s, v[2] * s)

    private fun add(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])

    private fun length(v: FloatArray): Float =
        sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    private fun normalize(v: FloatArray): FloatArray {
        val len = length(v)
        if (len < 1e-6f) return floatArrayOf(0f, 0f, 0f)
        return floatArrayOf(v[0] / len, v[1] / len, v[2] / len)
    }
}