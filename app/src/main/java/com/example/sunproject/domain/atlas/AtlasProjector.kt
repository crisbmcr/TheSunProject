package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import com.example.sunproject.data.model.FrameRecord
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
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

private const val ZENITH_REFINER_MIN_RAW_PITCH_DEG = 85.5f
private const val ZENITH_REFINER_MAX_ABS_ROLL_DEG = 1.0f
private const val ZENITH_REFINER_MAX_ABS_ECC_ROT_DEG = 1.5f
private const val ZENITH_REFINER_MIN_ECC_SCORE = 0.25
private const val ENABLE_ZENITH_TOPFACE_REFINER = false

// Which zenith refiner to use when ENABLE_ZENITH_TOPFACE_REFINER is true.
// - "feature" → ORB + RANSAC 1-DoF (new, feature-based, immune to IMU seed bias)
// - "legacy"  → photometric annulus correlation (old, kept for A/B comparison)
// Change to "legacy" to revert behaviour without editing call sites.
private const val ZENITH_REFINER_IMPL = "feature"

private const val PERSISTED_MOUNT_MIN_SAMPLE_COUNT = 12

private const val PERSISTED_MOUNT_MIN_QUALITY_SCORE = 0.996f
private const val PERSISTED_MOUNT_MAX_FORWARD_TILT_DEG = 4.5f


private const val FRAME_RGB_GAIN_MIN = 0.92f

private const val FRAME_RGB_GAIN_MAX = 1.08f

private const val FRAME_GAIN_ESTIMATE_X_STEP = 6

private const val FRAME_GAIN_ESTIMATE_Y_STEP = 3

private const val FRAME_GAIN_ESTIMATE_MIN_SAMPLES = 400

private const val ZENITH_ERP_MAX_WEIGHT = 0.40f
private const val ZENITH_ERP_FADE_START_ALT_DEG = 74f
private const val ZENITH_ERP_FADE_FULL_ALT_DEG = 82f
private const val ZENITH_ERP_BASE_WEIGHT_DAMP = 1.25f

// Cambio A (anti-blanqueamiento polo): fade-out de H0/H45 hacia el polo.
// Razón: en altitudes >80° el remap inverso replica el top-edge de cada H45
// sobre muchos pixels del atlas (zona equirectangular comprimida en y).
// Esos pixels deformados, sumados entre 2-3 H45 vecinos, contaminan la cap
// polar donde el Z0 debería dominar. Apagamos H45 a partir de 80°
// (Z0 ya tiene ~84% de peso ahí por su smoothstep 74→82) y llegamos a 0
// a 88°, dejando el polo limpio para el Z0.
private const val H45_POLAR_FADE_START_ALT_DEG = 80f
private const val H45_POLAR_FADE_FULL_ALT_DEG = 88f


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
    private var cachedGyroToTrueNorthCorrectionDeg: Float? = null

    // FIX TRUE-NORTH (2026-05-13): declinación magnética de la sesión,
    // computada con GeomagneticField al iniciar la captura y persistida
    // en session.json. AtlasBuildUseCase la inyecta acá antes de projectar
    // para que esté disponible cuando ZenithMatrixProjector la consulte.
    // PanoramaViewActivity también la setea al abrir un atlas, leyendo
    // directo del session.json. Es la fuente única de verdad.
    private var cachedSessionDeclinationDeg: Float? = null

    fun setSessionDeclinationDeg(deg: Float?) {
        cachedSessionDeclinationDeg = deg
        Log.i(
            "AtlasDeclCorrection",
            "setSessionDeclinationDeg=${deg?.let { "%.2f".format(it) } ?: "null"}°"
        )
    }

    /**
     * Corrección angular para llevar yaw mag-N → true-N (declinación
     * magnética local de la sesión).
     *
     * Convención del atlas: vive en true-N (ENU geográfico). Los H0/H45
     * caen ahí naturalmente porque measuredAzimuthDeg hereda del anchor
     * inicial que tiene applyDeclination aplicado. El Z0 NO: usa la
     * matriz rotationM** cruda, que está en mag-N. Por eso
     * ZenithMatrixProjector le suma este valor al rayo mundo.
     *
     * La vista 3D consume el mismo valor: el rotvec sensor en runtime
     * también está en mag-N, GyroCameraController lo lleva a true-N
     * pre-multiplicando por esta rotación.
     *
     * Devuelve 0 si no hay GPS — el atlas queda en mag-N puro y el ábaco
     * cae desplazado por declinación, pero internamente consistente
     * entre H0/H45/Z0 y vista 3D (todos quedan en mag-N).
     */
    fun gyroToTrueNorthCorrectionDeg(): Float =
        cachedSessionDeclinationDeg ?: cachedGyroToTrueNorthCorrectionDeg ?: 0f

    // ============================================================
    // BIAS ROTVEC ↔ GRAV+MAG (Fase 6)
    // ============================================================
    // Matriz 3x3 row-major que lleva una matriz expresada en frame
    // grav+mag al frame rotvec (TYPE_ROTATION_VECTOR de Android):
    //   R_rotvec = R_bias · R_gravmag
    //
    // Se computa una vez por sesión en projectFramesToAtlas, promediando
    // R_rotvec_i · R_gravmag_i^T sobre los frames H0/H45 que tengan AMBAS
    // matrices presentes y consistentes. SVD para obtener un promedio
    // que sea matriz de rotación válida.
    //
    // Si por cualquier motivo el bias no se puede estimar (faltan frames,
    // spread alto, rotación implícita anómala) → null. ZenithMatrixProjector
    // entonces se comporta como antes (sin corrección).
    //
    // Como se setea solo durante projectFramesToAtlas, queda disponible
    // por toda la sesión hasta que se proyecte otra. Concurrent reads ok
    // porque es un assignment atómico de referencia.
    private var cachedRotvecGravMagBiasMatrix: FloatArray? = null

    fun rotvecGravMagBiasMatrix(): FloatArray? = cachedRotvecGravMagBiasMatrix

    fun approximateFootprint(frame: FrameRecord): FrameFootprint {
        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val pose = resolveZenithProjectionPose(
            frame = frame,
            absoluteYawDegOverride = frame.absAzimuthDeg,
            absolutePitchDegOverride = frame.absPitchDeg,
            absoluteRollDegOverride = frame.absRollDeg
        )

        val centerYawDeg = pose.yawDeg
        val centerPitchDeg = pose.pitchDeg

        val basis = buildProjectionBasisFromAngles(
            yawDeg = pose.yawDeg,
            pitchDeg = pose.pitchDeg,
            rollDeg = pose.rollDeg,
            zenithLike = isZenithFrame(frame) || frame.targetPitchDeg >= 80f || pose.pitchDeg >= 80f,
            forceHardZenith = pose.hardZenith
        )

        val tanHalfH = tan(Math.toRadians(hfov / 2.0)).toFloat()
        val tanHalfV = tan(Math.toRadians(vfov / 2.0)).toFloat()

        var minYawDelta = Float.POSITIVE_INFINITY
        var maxYawDelta = Float.NEGATIVE_INFINITY
        var minAlt = Float.POSITIVE_INFINITY
        var maxAlt = Float.NEGATIVE_INFINITY

        fun sample(nx: Float, ny: Float) {
            val camX = nx * tanHalfH
            val camY = -ny * tanHalfV

            val rayWorld = normalize(
                add(
                    add(scale(basis.right, camX), scale(basis.up, camY)),
                    basis.forward
                )
            )

            val azDeg = normalizeTwistDeg(
                Math.toDegrees(
                    atan2(rayWorld[0].toDouble(), rayWorld[1].toDouble())
                ).toFloat()
            )

            val altDeg = Math.toDegrees(
                asin(rayWorld[2].coerceIn(-1f, 1f).toDouble())
            ).toFloat()

            val yawDelta = shortestAngleDeltaDeg(centerYawDeg, azDeg)
            minYawDelta = minOf(minYawDelta, yawDelta)
            maxYawDelta = maxOf(maxYawDelta, yawDelta)
            minAlt = minOf(minAlt, altDeg)
            maxAlt = maxOf(maxAlt, altDeg)
        }

        val steps = 24
        for (i in 0..steps) {
            val t = i / steps.toFloat()
            val v = -1f + 2f * t
            sample(v, -1f)
            sample(1f, v)
            sample(-v, 1f)
            sample(-1f, -v)
        }
        val zenithCapLike =
            isZenithFrame(frame) ||
                    pose.hardZenith ||
                    pose.pitchDeg >= 84f

        if (zenithCapLike) {
            return FrameFootprint(
                minAzimuthDeg = 0f,
                maxAzimuthDeg = 360f,
                minAltitudeDeg = minAlt.coerceIn(0f, 90f),
                maxAltitudeDeg = 90f
            )
        }
        return FrameFootprint(
            minAzimuthDeg = centerYawDeg + minYawDelta,
            maxAzimuthDeg = centerYawDeg + maxYawDelta,
            minAltitudeDeg = minAlt.coerceIn(0f, 90f),
            maxAltitudeDeg = maxAlt.coerceIn(0f, 90f)
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
        // Atlas vive en mag-N puro (estado documentado de Fase 7).
        // La corrección mag→true-N (cachedGyroToTrueNorthCorrectionDeg)
        // se sigue computando pero NO se aplica acá — la consume solo
        // GyroCameraController para rotar la viewMatrix de la vista 3D.
        // De esa forma atlas y cámara virtual quedan en frames consistentes
        // (ambos mag-N a nivel del atlas; la cámara aplica la corrección
        // adicional para que los overlays solares calculados en true-N
        // caigan donde corresponde).
        return normalizeTwistDeg(frame.measuredAzimuthDeg)
    }


    private fun basePitchDeg(frame: FrameRecord): Float {
        return frame.absPitchDeg ?: frame.measuredPitchDeg
    }

    private fun baseRollDeg(frame: FrameRecord): Float {
        return frame.absRollDeg ?: frame.measuredRollDeg
    }

    private data class ZenithProjectionPose(
        val yawDeg: Float,
        val pitchDeg: Float,
        val rollDeg: Float,
        val hardZenith: Boolean
    )

    private fun resolveZenithProjectionPose(
        frame: FrameRecord,
        absoluteYawDegOverride: Float?,
        absolutePitchDegOverride: Float?,
        absoluteRollDegOverride: Float?
    ): ZenithProjectionPose {
        val pitchDeg = (absolutePitchDegOverride ?: basePitchDeg(frame)).coerceIn(0f, 90f)
        val zenithLike = isZenithFrame(frame) || frame.targetPitchDeg >= 80f || pitchDeg >= 80f
        val yawDeg = normalizeTwistDeg(absoluteYawDegOverride ?: baseYawDeg(frame))
        val rollDeg = if (zenithLike && pitchDeg >= 84f) {
            0f
        } else {
            absoluteRollDegOverride ?: baseRollDeg(frame)
        }

        return ZenithProjectionPose(
            yawDeg = yawDeg,
            pitchDeg = pitchDeg,
            rollDeg = rollDeg,
            hardZenith = zenithLike && pitchDeg >= 84f
        )
    }
    private const val MOUNT_REF_MAX_YAW_ERR_DEG = 10f
    private const val MOUNT_REF_MAX_PITCH_ERR_DEG = 6f
    private const val MOUNT_REF_MAX_ABS_ROLL_DEG = 8f
    private const val MOUNT_REF_MIN_WEIGHT = 0.20f

    private fun angularDeltaDeg(a: Float, b: Float): Float {
        val d = ((a - b + 540f) % 360f) - 180f
        return abs(d)
    }

    private fun smoothstep(edge0: Float, edge1: Float, x: Float): Float {
        if (edge1 <= edge0) return if (x >= edge1) 1f else 0f
        val t = ((x - edge0) / (edge1 - edge0)).coerceIn(0f, 1f)
        return t * t * (3f - 2f * t)
    }

    private fun shortestAngleDeltaDeg(fromDeg: Float, toDeg: Float): Float {
        var d = (toDeg - fromDeg) % 360f
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return d
    }

    private fun lerpAngleDeg(aDeg: Float, bDeg: Float, t: Float): Float {
        return normalizeTwistDeg(aDeg + shortestAngleDeltaDeg(aDeg, bDeg) * t)
    }

    private fun projectOntoPlane(v: FloatArray, normal: FloatArray): FloatArray {
        return add(v, scale(normal, -dot(v, normal)))
    }

    private fun visualFallbackBaseYawDeg(frame: FrameRecord): Float {
        return baseYawDeg(frame)
    }

    private fun visualFallbackBasePitchDeg(frame: FrameRecord): Float {
        return basePitchDeg(frame)
    }

    private fun visualFallbackBaseRollDeg(frame: FrameRecord): Float {
        return baseRollDeg(frame)
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

        val useHardZenithBasis = forceHardZenith || (zenithLike && pitchDeg >= 84f)

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

        val baseYaw = baseYawDeg(frame)
        val basePitch = basePitchDeg(frame)
        val baseRoll = baseRollDeg(frame)

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

        val rawPitchAbsDeg = maxOf(pitchAbsDeg, basePitch)

        val pitchUsedDeg = rawPitchAbsDeg.coerceIn(84f, 90f)

        val preferredMatrixYawDeg = if (useHardZenithBranch || rawPitchAbsDeg >= 84f) {
            tangentAzDeg
        } else {
            forwardAzDeg
        }

        val rawYawDelta = shortestAngleDeltaDeg(baseYaw, preferredMatrixYawDeg)

        val yawTrust = smoothstep(84f, 87.5f, rawPitchAbsDeg)

        val interpolatedYawDeg = if (useHardZenithBranch && rawPitchAbsDeg >= 87f) {
            preferredMatrixYawDeg
        } else {
            val maxYawDelta = when {
                rawPitchAbsDeg >= 88.5f -> 18f
                rawPitchAbsDeg >= 87f -> 22f
                rawPitchAbsDeg >= 85f -> 28f
                else -> 30f
            }

            val clampedMatrixYaw = normalizeTwistDeg(
                baseYaw + rawYawDelta.coerceIn(-maxYawDelta, maxYawDelta)
            )

            lerpAngleDeg(baseYaw, clampedMatrixYaw, yawTrust)
        }

        val hardZenithSeed = useHardZenithBranch || rawPitchAbsDeg >= 87f

        val forcePreferredYaw =
            hardZenithSeed &&
                    abs(shortestAngleDeltaDeg(interpolatedYawDeg, preferredMatrixYawDeg)) > 2f

        val centerYawDeg = if (forcePreferredYaw) {
            preferredMatrixYawDeg
        } else {
            interpolatedYawDeg
        }

        val zeroRollBasis = buildProjectionBasisFromAngles(
            yawDeg = centerYawDeg,
            pitchDeg = pitchUsedDeg,
            rollDeg = 0f,
            zenithLike = true,
            forceHardZenith = true
        )

        val zeroRightT = safeNormalizeTangent(zeroRollBasis.right, forward) ?: return null
        val zeroUpT = safeNormalizeTangent(zeroRollBasis.up, forward) ?: return null

        val cosR = dot(rightT, zeroRightT).coerceIn(-1f, 1f)
        val sinR = dot(rightT, zeroUpT).coerceIn(-1f, 1f)
        val residualRollDeg = Math.toDegrees(
            atan2(sinR.toDouble(), cosR.toDouble())
        ).toFloat()

        val rollAbsDeg = 0f

        Log.d(
            "AtlasZenithMatrixSeed",
            "frame=${frame.frameId} " +
                    "yawForward=${"%.2f".format(forwardAzDeg)} " +
                    "yawTangent=${"%.2f".format(tangentAzDeg)} " +
                    "yawPreferred=${"%.2f".format(preferredMatrixYawDeg)} " +
                    "yawUsed=${"%.2f".format(centerYawDeg)} " +
                    "yawErrorToPreferred=${"%.2f".format(shortestAngleDeltaDeg(centerYawDeg, preferredMatrixYawDeg))} " +
                    "yawTrust=${"%.3f".format(yawTrust)} " +
                    "hardZenithSeed=$hardZenithSeed " +
                    "forcedToPreferred=$forcePreferredYaw " +
                    "rawPitchAbs=${"%.2f".format(rawPitchAbsDeg)} " +
                    "pitchAbsClamped=${"%.2f".format(pitchUsedDeg)} " +
                    "residualRoll=${"%.2f".format(residualRollDeg)} " +
                    "rollAbs=${"%.2f".format(rollAbsDeg)} " +
                    "baseYaw=${"%.2f".format(baseYaw)} " +
                    "basePitch=${"%.2f".format(basePitch)} " +
                    "baseRoll=${"%.2f".format(baseRoll)} " +
                    "legacyTwist=${"%.2f".format(normalizeTwistDeg(centerYawDeg - baseYaw))} " +
                    "legacyPitchOffset=${"%.2f".format(clampZenithPitchOffsetDeg(pitchUsedDeg - basePitch))} " +
                    "legacyRollOffset=${"%.2f".format(clampZenithRollOffsetDeg(rollAbsDeg - baseRoll))} " +
                    "seedHard=true"
        )

        return buildZenithPoseEstimateFromAbsolute(
            frame = frame,
            absoluteYawDeg = centerYawDeg,
            absolutePitchDeg = pitchUsedDeg,
            absoluteRollDeg = rollAbsDeg,
            score = Float.POSITIVE_INFINITY,
            comparedPixels = 0,
            confidence = 1f
        )
    }
    fun projectFramesToAtlas(frames: List<FrameRecord>, atlas: SkyAtlas) {
        val ordered = frames.sortedBy { it.shotIndex }

        // Note: the session yaw anchor was removed. It existed to
        // compensate for magnetometer drift in absAzimuthDeg — but
        // baseYawDeg now reads measuredAzimuthDeg (gyro-only) for
        // non-Z0 frames, which is already drift-free within a session
        // (measured drift over 4 real sessions: max 1.6° from target).
        // With no drift to correct, the anchor only introduced extra
        // bias when the first frame had a small alignment error.
        //
        // Z0 frames still read absAzimuthDeg, but their value was
        // captured via the pre-zenith snapshot in CaptureActivity,
        // which reads the gyro-fused yaw while pitch<65° (well below
        // the Euler singularity) and persists it as absAzimuthDeg.
        val nonZenithFrames = ordered.filterNot { isZenithFrame(it) }
        val zenithFrames = ordered.filter { isZenithFrame(it) }

// FIX DECLINACIÓN: calcular corrección gyro→true-N para esta sesión
        cachedGyroToTrueNorthCorrectionDeg = computeGyroToTrueNorthCorrection(nonZenithFrames)
        Log.i(
            "AtlasDeclCorrection",
            "correction=${cachedGyroToTrueNorthCorrectionDeg?.let { "%.2f".format(it) } ?: "null"}° " +
                    "(positivo = rotar atlas al este para llegar a true-N)"
        )

        // FIX BIAS ROTVEC↔GRAV+MAG (Fase 6): estimar la rotación que lleva
        // matrices grav+mag al frame rotvec. Solo usa H0/H45 (frames con
        // pose estable, lejos del cenit). Resultado se cachea pero NO se
        // aplica todavía — la edición 5 lo consume desde ZenithMatrixProjector.
        cachedRotvecGravMagBiasMatrix = computeRotvecToGravMagBias(nonZenithFrames)

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

        // NOTA: Fase 2 (multi-band blending via Burt-Adelson manual) fue
        // implementada y descartada en sesión 2026-05-11 tras A/B contra
        // este path. Veredicto: el multi-band convierte el misalignment
        // estructural del rotvec (~1-2°) en ghost extendido en lugar de
        // esconder seams. Ver MultiBandAtlasBlender.kt en commit anterior
        // si se necesita reactivar para escenarios distintos (ej. exterior
        // con gain compensation, que sería un módulo distinto, no este).
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

                Log.d(
                    "AtlasProjector",
                    "frame=${frame.frameId} ring=${frame.ringId} " +
                            "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                            "weight=${"%.3f".format(frameWeight)}"
                )

                if (frameWeight <= 0f) return@forEach

                ZenithMatrixProjector.projectZ0ToAtlas(
                    frame = frame,
                    src = src,
                    atlas = atlas,
                    frameWeight = frameWeight,
                    displayRotation = 0 // TODO: persistir display.rotation por frame
                )
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

        val effectiveZenithTwistDeg = zenithTwistDegOverride ?: ZENITH_TWIST_FALLBACK_DEG
        val effectiveZenithPitchOffsetDeg =
            zenithPitchOffsetDegOverride ?: ZENITH_PITCH_OFFSET_FALLBACK_DEG
        val effectiveZenithRollOffsetDeg =
            zenithRollOffsetDegOverride ?: ZENITH_ROLL_OFFSET_FALLBACK_DEG

        val zenithPose = if (isZenithFrame(frame)) {
            resolveZenithProjectionPose(
                frame = frame,
                absoluteYawDegOverride = zenithAbsoluteYawDegOverride,
                absolutePitchDegOverride = zenithAbsolutePitchDegOverride,
                absoluteRollDegOverride = zenithAbsoluteRollDegOverride
            )
        } else {
            null
        }

        val hasAbsoluteZenithOverride =
            zenithPose != null &&
                    zenithAbsoluteYawDegOverride != null &&
                    zenithAbsolutePitchDegOverride != null &&
                    zenithAbsoluteRollDegOverride != null

        val projectionAzimuthDeg = if (zenithPose != null) {
            zenithPose.yawDeg
        } else {
            normalizeTwistDeg(baseYawDeg(frame))
        }

        val projectionPitchDeg = if (zenithPose != null) {
            zenithPose.pitchDeg
        } else {
            basePitchDeg(frame).coerceIn(0f, 90f)
        }

        val projectionRollDeg = if (zenithPose != null) {
            zenithPose.rollDeg
        } else {
            baseRollDeg(frame)
        }

        val zenithLike =
            zenithPose != null ||
                    frame.targetPitchDeg >= 80f ||
                    projectionPitchDeg >= 80f

        val basis = buildProjectionBasisFromAngles(
            yawDeg = projectionAzimuthDeg,
            pitchDeg = projectionPitchDeg,
            rollDeg = projectionRollDeg,
            zenithLike = zenithLike,
            forceHardZenith = zenithPose?.hardZenith == true
        )

        val forward = basis.forward
        val right = basis.right
        val up = basis.up

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

        val zenithCapLike = zenithPose != null && projectionPitchDeg >= 84f

        val yTop = if (zenithCapLike) {
            0
        } else {
            AtlasMath.altitudeToY(fp.maxAltitudeDeg, atlas.config)
        }

        val yBottom = AtlasMath.altitudeToY(fp.minAltitudeDeg, atlas.config)

        val srcW = src.width
        val srcH = src.height
        val srcPixels = IntArray(srcW * srcH)
        src.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val coversAllAzimuth =
            zenithCapLike ||
                    zenithPose != null ||
                    frame.targetPitchDeg >= 80f ||
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

                    var rawWeight = frameWeight * localWeight

                    if (zenithPose != null) {
                        rawWeight = min(rawWeight, ZENITH_ERP_MAX_WEIGHT)
                    }

                    val zenithAltitudeAlpha = if (zenithPose != null) {
                        smoothstep(
                            ZENITH_ERP_FADE_START_ALT_DEG,
                            ZENITH_ERP_FADE_FULL_ALT_DEG,
                            altDeg
                        )
                    } else {
                        1f
                    }

                    val baseSuppression = if (zenithPose != null && atlas.hasCoverageAt(x, y)) {
                        1f / (1f + atlas.weightAt(x, y) * ZENITH_ERP_BASE_WEIGHT_DAMP)
                    } else {
                        1f
                    }

                    // Cambio A: para frames no-zenithLike (H0/H45), atenuar al
                    // acercarse al polo. Z0 va por ZenithMatrixProjector con su
                    // propio fade, así que zenithPose==null marca exactamente
                    // los frames que tenemos que apagar acá.
                    val polarFadeForNonZenith = if (zenithPose == null) {
                        1f - smoothstep(
                            H45_POLAR_FADE_START_ALT_DEG,
                            H45_POLAR_FADE_FULL_ALT_DEG,
                            altDeg
                        )
                    } else {
                        1f
                    }

                    val finalWeight = rawWeight * zenithAltitudeAlpha * baseSuppression * polarFadeForNonZenith
                    if (finalWeight <= 0f) continue

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
    private fun safeOverlapChannelGain(atlasMean: Float, zenithMean: Float): Float {
        if (!atlasMean.isFinite() || !zenithMean.isFinite() || zenithMean <= 1f) return 1f
        return (atlasMean / zenithMean).coerceIn(FRAME_RGB_GAIN_MIN, FRAME_RGB_GAIN_MAX)
    }

    private fun applyRgbGainToBitmap(src: Bitmap, gain: RgbGain): Bitmap {
        val identity =
            abs(gain.r - 1f) < 1e-3f &&
                    abs(gain.g - 1f) < 1e-3f &&
                    abs(gain.b - 1f) < 1e-3f

        if (identity) return src

        val out = src.copy(Bitmap.Config.ARGB_8888, true)
        val w = out.width
        val h = out.height
        val pixels = IntArray(w * h)

        out.getPixels(pixels, 0, w, 0, 0, w, h)

        for (i in pixels.indices) {
            val c = pixels[i]
            val a = Color.alpha(c)
            val r = (Color.red(c).toFloat() * gain.r).roundToInt().coerceIn(0, 255)
            val g = (Color.green(c).toFloat() * gain.g).roundToInt().coerceIn(0, 255)
            val b = (Color.blue(c).toFloat() * gain.b).roundToInt().coerceIn(0, 255)
            pixels[i] = Color.argb(a, r, g, b)
        }

        out.setPixels(pixels, 0, w, 0, 0, w, h)
        return out
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
        val forwardTiltOk = forwardTiltDeg(basis) <= ZENITH_MATRIX_SEED_MAX_FORWARD_TILT_DEG

        return rawPitchOk && forwardTiltOk
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
    private data class BitmapRgbStats(
        val meanR: Float,
        val meanG: Float,
        val meanB: Float,
        val meanLuma: Float
    )

    private fun computeBitmapRgbStats(bitmap: Bitmap): BitmapRgbStats {
        val w = bitmap.width
        val h = bitmap.height
        val pixels = IntArray(w * h)
        bitmap.getPixels(pixels, 0, w, 0, 0, w, h)

        var sumR = 0.0
        var sumG = 0.0
        var sumB = 0.0
        var count = 0

        for (c in pixels) {
            val a = (c ushr 24) and 0xFF
            if (a == 0) continue
            val r = (c ushr 16) and 0xFF
            val g = (c ushr 8) and 0xFF
            val b = c and 0xFF
            sumR += r.toDouble()
            sumG += g.toDouble()
            sumB += b.toDouble()
            count++
        }

        if (count == 0) {
            return BitmapRgbStats(0f, 0f, 0f, 0f)
        }

        val meanR = (sumR / count).toFloat()
        val meanG = (sumG / count).toFloat()
        val meanB = (sumB / count).toFloat()
        val meanLuma = (0.2126f * meanR + 0.7152f * meanG + 0.0722f * meanB)

        return BitmapRgbStats(meanR, meanG, meanB, meanLuma)
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
        val zenithLike = isZenithFrame(frame)

        if (zenithLike) {
            val pitchBase = frame.absPitchDeg ?: frame.measuredPitchDeg
            val pitchErr = abs(frame.targetPitchDeg - pitchBase)

            // Para Z0 no usar measuredRollDeg como criterio de descarte:
            // cerca del zenith el roll visual/sensorial es inestable y ya vimos
            // que la autocaptura correcta quedó gobernada por absolutePitchDeg.
            val maxZenithPitchErr = if (frame.absPitchDeg != null) 4.0f else 2.0f

            if (pitchErr > maxZenithPitchErr) {
                Log.d(
                    "AtlasWeight",
                    "reject frame=${frame.frameId} ring=${frame.ringId} reason=zenithPitchErr " +
                            "target=${frame.targetAzimuthDeg}/${frame.targetPitchDeg} " +
                            "pitchBase=${"%.2f".format(pitchBase)} " +
                            "pitchMeasured=${"%.2f".format(frame.measuredPitchDeg)} " +
                            "pitchAbs=${"%.2f".format(frame.absPitchDeg ?: Float.NaN)} " +
                            "rollMeasured=${"%.2f".format(frame.measuredRollDeg)} " +
                            "rollAbs=${"%.2f".format(frame.absRollDeg ?: Float.NaN)} " +
                            "pitchErr=${"%.2f".format(pitchErr)} max=${"%.2f".format(maxZenithPitchErr)}"
                )
                return 0f
            }

            val pitchScore = (1f - pitchErr / maxZenithPitchErr).coerceIn(0f, 1f)

            // Darle un piso al Z0 para que no vuelva a quedar anulado por inestabilidad
            // residual. Luego AtlasZenithBlend ya lo capará con ZENITH_ERP_MAX_WEIGHT.
            val weight = (0.45f + 0.55f * pitchScore).coerceIn(0.45f, 1f)

            Log.d(
                "AtlasWeight",
                "accept frame=${frame.frameId} ring=${frame.ringId} zenith=true " +
                        "target=${"%.2f".format(frame.targetAzimuthDeg)}/${"%.2f".format(frame.targetPitchDeg)} " +
                        "pitchBase=${"%.2f".format(pitchBase)} " +
                        "pitchMeasured=${"%.2f".format(frame.measuredPitchDeg)} " +
                        "pitchAbs=${"%.2f".format(frame.absPitchDeg ?: Float.NaN)} " +
                        "rollMeasured=${"%.2f".format(frame.measuredRollDeg)} " +
                        "rollAbs=${"%.2f".format(frame.absRollDeg ?: Float.NaN)} " +
                        "pitchErr=${"%.2f".format(pitchErr)} " +
                        "weight=${"%.3f".format(weight)}"
            )

            return weight
        }

        val azErr = angleDiffDeg(frame.targetAzimuthDeg, frame.measuredAzimuthDeg)
        val pitchErr = abs(frame.targetPitchDeg - frame.measuredPitchDeg)
        val rollAbs = abs(frame.measuredRollDeg)

        val maxAzErr = when {
            frame.targetPitchDeg >= 40f -> 4.5f
            else -> 4.0f
        }
        val maxPitchErr = 2.75f
        val maxRollErr = 3.5f

        if (azErr > maxAzErr) {
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

        val azScore = (1f - azErr / maxAzErr).coerceIn(0f, 1f)
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

    /**
     * Calcula el offset gyro-N → true-N promediando TODOS los frames no-zenitales
     * con absAzimuthDeg disponible. Usa mean angular con sin/cos para manejar el
     * wrap-around en 360°. Descarta la corrección si la dispersión entre frames
     * es alta (indicador de magnetómetro mal calibrado).
     *
     * Retorna null si:
     *   - no hay ningún frame con absAzimuthDeg (GPS nunca disponible), o
     *   - la dispersión angular entre frames supera MAX_SPREAD_DEG (mag errático)
     *
     * En esos casos el atlas queda en gyro-N (comportamiento pre-fix).
     */
    /**
     * Computa la matriz de bias R_bias tal que R_rotvec = R_bias · R_gravmag,
     * promediando sobre los frames H0/H45 que tienen AMBAS matrices presentes.
     *
     * Pipeline:
     *   1. Filtrar frames con rotationM** (rotvec) Y rotationGravMagM** (grav+mag).
     *   2. Para cada frame: R_bias_i = R_rotvec · R_gravmag^T.
     *   3. Sumar todas las R_bias_i en una matriz M.
     *   4. SVD(M) = U · Σ · V^T → R_avg = U · V^T (corregido el signo si det<0).
     *      Esto da la matriz de rotación más cercana al promedio aritmético
     *      en el sentido de Frobenius — solución estándar de Wahba para
     *      promediar rotaciones.
     *   5. Validar:
     *      - Al menos 10 frames usados.
     *      - Spread (rotación máxima entre R_bias_i y R_avg) < 3°.
     *      - Rotación implícita de R_avg < 15° (es un bias, no un giro grande).
     *      Si alguna falla → return null.
     *
     * Retorna FloatArray(9) row-major o null.
     */
    private fun computeRotvecToGravMagBias(frames: List<FrameRecord>): FloatArray? {
        val MIN_FRAMES = 10
        val MAX_SPREAD_DEG = 3f
        val MAX_BIAS_ROTATION_DEG = 15f

        // Step 1: filtrar frames con ambas matrices presentes.
        val pairs = mutableListOf<Pair<FloatArray, FloatArray>>()
        for (f in frames) {
            val rotvec = storedDeviceToWorldMatrix(f) ?: continue
            val gravMag = storedGravMagMatrix(f) ?: continue
            pairs.add(rotvec to gravMag)
        }

        Log.i(
            "Z0BiasCorrection",
            "estimateBias: framesWithBoth=${pairs.size} (need ≥ $MIN_FRAMES)"
        )

        if (pairs.size < MIN_FRAMES) {
            Log.w(
                "Z0BiasCorrection",
                "REJECT: not enough frames with both matrices " +
                        "(have=${pairs.size}, need=$MIN_FRAMES)"
            )
            return null
        }

        // Step 2-3: computar R_bias_i = R_rotvec · R_gravmag^T, sumar.
        val biasList = mutableListOf<FloatArray>()
        val M = DoubleArray(9) // accumulator (Double para precisión en suma)
        for ((rotvec, gravMag) in pairs) {
            val gravMagT = transpose3x3(gravMag)
            val biasI = matMul3x3(rotvec, gravMagT) // 3×3 que debería ser ~identidad
            biasList.add(biasI)
            for (k in 0 until 9) M[k] += biasI[k].toDouble()
        }

        // Step 4: SVD del promedio. M es la suma; M/N es el promedio aritmético.
        // SVD de M/N o de M da el mismo U·V^T (la escala no afecta).
        val Mfloat = FloatArray(9) { M[it].toFloat() }
        val Rsvd = projectToRotationViaSVD(Mfloat) ?: run {
            Log.w("Z0BiasCorrection", "REJECT: SVD failed (matrix degenerate)")
            return null
        }

        // Step 5a: validar rotación implícita del bias.
        val biasAngleDeg = rotationAngleOfMatrixDeg(Rsvd)
        if (biasAngleDeg > MAX_BIAS_ROTATION_DEG) {
            Log.w(
                "Z0BiasCorrection",
                "REJECT: bias rotation too large " +
                        "(${"%.2f".format(biasAngleDeg)}° > $MAX_BIAS_ROTATION_DEG°). " +
                        "Probable bug or sensor problem, not a real bias."
            )
            return null
        }

        // Step 5b: validar spread — la rotación máxima entre cada R_bias_i
        // y el promedio R_svd. Si las muestras individuales están muy
        // dispersas el promedio no es confiable.
        var maxSpreadDeg = 0f
        for (biasI in biasList) {
            val biasIRsvdT = matMul3x3(biasI, transpose3x3(Rsvd))
            val spreadDeg = rotationAngleOfMatrixDeg(biasIRsvdT)
            if (spreadDeg > maxSpreadDeg) maxSpreadDeg = spreadDeg
        }
        if (maxSpreadDeg > MAX_SPREAD_DEG) {
            Log.w(
                "Z0BiasCorrection",
                "REJECT: spread too high " +
                        "(${"%.2f".format(maxSpreadDeg)}° > $MAX_SPREAD_DEG°). " +
                        "Magnetometer or rotvec inconsistent across frames."
            )
            return null
        }

        Log.i(
            "Z0BiasCorrection",
            "ACCEPT: framesUsed=${pairs.size} " +
                    "biasAngle=${"%.3f".format(biasAngleDeg)}° " +
                    "spread=${"%.3f".format(maxSpreadDeg)}° " +
                    "R=[${"%.4f".format(Rsvd[0])},${"%.4f".format(Rsvd[1])},${"%.4f".format(Rsvd[2])}; " +
                    "${"%.4f".format(Rsvd[3])},${"%.4f".format(Rsvd[4])},${"%.4f".format(Rsvd[5])}; " +
                    "${"%.4f".format(Rsvd[6])},${"%.4f".format(Rsvd[7])},${"%.4f".format(Rsvd[8])}]"
        )

        return Rsvd
    }

    /**
     * Lee la matriz grav+mag persistida en FrameRecord.
     * Igual que storedDeviceToWorldMatrix pero sobre los 9 campos rotationGravMagM**.
     */
    private fun storedGravMagMatrix(frame: FrameRecord): FloatArray? {
        val m00 = frame.rotationGravMagM00 ?: return null
        val m01 = frame.rotationGravMagM01 ?: return null
        val m02 = frame.rotationGravMagM02 ?: return null
        val m10 = frame.rotationGravMagM10 ?: return null
        val m11 = frame.rotationGravMagM11 ?: return null
        val m12 = frame.rotationGravMagM12 ?: return null
        val m20 = frame.rotationGravMagM20 ?: return null
        val m21 = frame.rotationGravMagM21 ?: return null
        val m22 = frame.rotationGravMagM22 ?: return null

        return floatArrayOf(
            m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22
        )
    }

    // transpose3x3 ya existe más arriba en el archivo (junto a mulMat3Vec) —
    // se reusa esa. Solo agregamos matMul3x3 aquí, que es nueva.

    private fun matMul3x3(a: FloatArray, b: FloatArray): FloatArray = floatArrayOf(
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8]
    )

    /**
     * Ángulo de rotación implícito en una matriz de rotación 3×3, en grados.
     * Para R, traza = 1 + 2·cos(θ), entonces θ = acos((traza − 1) / 2).
     */
    private fun rotationAngleOfMatrixDeg(R: FloatArray): Float {
        val trace = R[0] + R[4] + R[8]
        val cosTheta = ((trace - 1f) / 2f).coerceIn(-1f, 1f)
        return Math.toDegrees(acos(cosTheta.toDouble())).toFloat()
    }

    /**
     * Proyecta una matriz 3×3 cualquiera a la matriz de rotación más cercana
     * usando SVD via OpenCV. Esto resuelve el problema de Wahba: dado
     * M = sum(R_i), encontrar el R que mejor representa el promedio.
     *
     * R_avg = U · diag(1, 1, det(U·V^T)) · V^T
     *
     * El factor diag(1, 1, det) corrige el signo si SVD devuelve una reflexión
     * en vez de rotación (sucede cuando det(M) < 0, raro pero posible).
     *
     * Retorna null si OpenCV no puede hacer SVD (matriz degenerada).
     */
    private fun projectToRotationViaSVD(M: FloatArray): FloatArray? {
        return try {
            val Mmat = org.opencv.core.Mat(3, 3, org.opencv.core.CvType.CV_32F)
            Mmat.put(0, 0, M)

            val w = org.opencv.core.Mat()
            val u = org.opencv.core.Mat()
            val vt = org.opencv.core.Mat()
            org.opencv.core.Core.SVDecomp(Mmat, w, u, vt)

            // R = U · V^T, con corrección de signo si det negativo.
            val ut = org.opencv.core.Mat()
            org.opencv.core.Core.transpose(vt, ut) // V (vt es V^T → transpose para V; pero ya tenemos VT)
            // Wait: vt es V^T directamente. Queremos U · V^T = U · vt.
            val uvt = org.opencv.core.Mat()
            org.opencv.core.Core.gemm(u, vt, 1.0, org.opencv.core.Mat(), 0.0, uvt)

            // Si det(uvt) < 0, es reflexión. Negar la última columna de U y rehacer.
            val det = org.opencv.core.Core.determinant(uvt)
            val Rmat = if (det < 0) {
                val uFixed = u.clone()
                // Negar columna 2 de U
                for (row in 0 until 3) {
                    val v = uFixed.get(row, 2)
                    uFixed.put(row, 2, -v[0])
                }
                val uvtFixed = org.opencv.core.Mat()
                org.opencv.core.Core.gemm(uFixed, vt, 1.0, org.opencv.core.Mat(), 0.0, uvtFixed)
                uFixed.release()
                uvtFixed
            } else {
                uvt
            }

            // Extraer FloatArray
            val out = FloatArray(9)
            val tmp = FloatArray(1)
            for (r in 0 until 3) {
                for (c in 0 until 3) {
                    Rmat.get(r, c, tmp)
                    out[r * 3 + c] = tmp[0]
                }
            }

            Mmat.release()
            w.release()
            u.release()
            vt.release()
            ut.release()
            if (Rmat !== uvt) uvt.release()
            Rmat.release()

            out
        } catch (t: Throwable) {
            Log.e("Z0BiasCorrection", "SVD failed", t)
            null
        }
    }

    private fun computeGyroToTrueNorthCorrection(frames: List<FrameRecord>): Float? {
        // Umbral máximo de desviación angular aceptable entre frames.
        // Cauchari típico: deltas -6° a -11° (spread ~5°).
        // Magnetómetro errático: deltas -5° a -47° (spread ~40°).
        // Un umbral de 15° separa claramente los dos casos.
        val MAX_SPREAD_DEG = 15f

        val validFrames = frames.filter { it.absAzimuthDeg != null }
        if (validFrames.isEmpty()) {
            Log.w("AtlasDeclCorrection", "skip: sin frames con absAzimuthDeg (GPS no disponible)")
            return null
        }

        // Calcular delta por frame, wrap a [-180°, +180°].
        val deltas = validFrames.map { f ->
            var d = (f.absAzimuthDeg ?: 0f) - f.measuredAzimuthDeg
            while (d > 180f) d -= 360f
            while (d <= -180f) d += 360f
            d
        }

        // Promedio angular via vector unitario (correcto cerca del wrap).
        var sumX = 0.0
        var sumY = 0.0
        deltas.forEach { d ->
            val r = Math.toRadians(d.toDouble())
            sumX += kotlin.math.cos(r)
            sumY += kotlin.math.sin(r)
        }
        val meanRad = kotlin.math.atan2(sumY, sumX)
        val meanDeg = Math.toDegrees(meanRad).toFloat()

        // Dispersión: distancia angular máxima al promedio.
        val spreadDeg = deltas.maxOf { d ->
            var diff = d - meanDeg
            while (diff > 180f) diff -= 360f
            while (diff <= -180f) diff += 360f
            kotlin.math.abs(diff)
        }

        Log.d(
            "AtlasDeclCorrection",
            "samples=${validFrames.size} mean=${"%.2f".format(meanDeg)}° " +
                    "spread=${"%.2f".format(spreadDeg)}° deltas=" +
                    deltas.joinToString(prefix = "[", postfix = "]") { "%.1f".format(it) }
        )

        if (spreadDeg > MAX_SPREAD_DEG) {
            Log.w(
                "AtlasDeclCorrection",
                "REJECT correction: spread=${"%.2f".format(spreadDeg)}° " +
                        "> threshold=${MAX_SPREAD_DEG}°. Magnetómetro inestable. " +
                        "Atlas quedará en gyro-N (sin fix). Recalibrar magnetómetro antes de próxima captura."
            )
            return null
        }

        return meanDeg
    }
}