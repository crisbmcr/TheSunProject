package com.example.sunproject

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.*
import android.hardware.camera2.CameraCharacteristics
import android.os.Bundle
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.util.SizeF
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.android.gms.location.*
import com.google.common.util.concurrent.ListenableFuture
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.atan
import kotlin.math.roundToInt
import android.view.WindowManager
import com.example.sunproject.data.model.FrameRecord
import com.example.sunproject.data.model.SessionRecord
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.data.storage.SessionPaths
import android.content.Intent
import com.example.sunproject.domain.atlas.AtlasBuildUseCase
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class CaptureActivity : AppCompatActivity(), SensorEventListener {

    // UI
    private lateinit var cameraPreview: PreviewView
    private lateinit var guideView: GuideView
    private lateinit var debugAzimuth: TextView
    private lateinit var debugPitch: TextView
    private lateinit var debugRoll: TextView
    private lateinit var debugLocation: TextView
    private lateinit var debugCaptureCount: TextView
    private lateinit var btnCapture: Button
    private lateinit var btnAuto: Button

    // CameraX
    private lateinit var cameraProviderFuture: ListenableFuture<ProcessCameraProvider>
    private lateinit var cameraExecutor: ExecutorService
    private var imageCapture: ImageCapture? = null

    // Sensores
    private lateinit var sensorManager: SensorManager
    private var rotationVectorSensor: Sensor? = null
    private val rotationMatrix = FloatArray(9)
    private val remappedRotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)
    private val lastAbsRotationMatrix = FloatArray(9)
    private var hasLastAbsRotationMatrix = false

    //private val alpha = 0.08f
    //private var isFirstReading = true
    private data class Vec3(var x: Float, var y: Float, var z: Float)
    private var gameRotationVectorSensor: Sensor? = null
    private var magneticFieldSensor: Sensor? = null

    private var magneticAccuracy: Int = SensorManager.SENSOR_STATUS_UNRELIABLE
    private var magneticFieldNormUt: Float = 0f

    private var gameYawDeg = 0f

    private var gamePitchDeg = 0f

    private var gameRollDeg = 0f

    private var absoluteYawDeg = 0f

    private var absolutePitchDeg = 0f

    private var absoluteRollDeg = 0f
    private var displayNorthOffsetDeg = 0f
    private var displayOffsetInitialized = false

    private var lastAbsTsNs: Long = 0L
    private var lastGameTsNs: Long = 0L

    private val gameDisplayFilter = AngleFilterState()

    private var guidePoseInitialized = false
    private var guideForward = Vec3(0f, 1f, 0f)
    private var guideUp = Vec3(0f, 0f, 1f)

    private var guideAzimuthDeg = 0f
    private var guidePitchDeg = 0f
    private var guideRollDeg = 0f

    private var lastGuideLogMs = 0L

    // Ubicación
    private lateinit var fusedLocationClient: FusedLocationProviderClient
    private lateinit var locationCallback: LocationCallback
    private var lastLocation: android.location.Location? = null

    // --- Stitching Self-test ---
    private var stitchSelfTestRan = false

    // --- Auto-captura ---
    private var autoCaptureEnabled = true
    private var maxShots = 0
    private val atlasBuildUseCase = AtlasBuildUseCase()
    private val azTolDeg = 4f
    private val pitchTolDeg = 2.5f
    private val rollTolDeg = 3f

    private val zenithPitchTolDeg = 1.4f
    private val zenithRollTolDeg = 2.2f

    private val holdMs = 800L
    private val zenithHoldMs = 1300L
    private val zenithSettleMs = 450L
    private val zenithAbsPitchDriftTolDeg = 0.8f
    private val zenithAbsRollDriftTolDeg = 1.2f

    private val cooldownMs = 1200L
    private var alignmentStartMs = 0L
    private var zenithLockEnteredMs = 0L
    private var zenithAbsPitchAnchorDeg = 0f
    private var lastShotMs = 0L

    private val capturedFiles = mutableListOf<File>()
    private var sessionDir: File? = null

    private var lastAzimuth = 0f
    private var lastPitch = 0f
    private var lastRoll = 0f

    private var stitchInProgress = false
    private var stitchDone = false

    private val sessionStore = JsonSessionStore()
    private var sessionPaths: SessionPaths? = null
    private var currentSessionId: String? = null

    private var currentCameraId: String? = null
    private var currentHfovDeg: Float? = null
    private var currentVfovDeg: Float? = null

    private var sessionYawAnchorDeg: Float? = null
    private var sessionPitchAnchorDeg: Float? = null
    private var sessionRollAnchorDeg: Float? = null

    private data class AngleFilterState(
        var yawX: Float = 1f,
        var yawY: Float = 0f,
        var pitch: Float = 0f,
        var roll: Float = 0f,
        var initialized: Boolean = false
    )

    private data class CapturedPose(

        val azimuthDeg: Float,

        val pitchDeg: Float,
        val rollDeg: Float,

        val absAzimuthDeg: Float?,
        val absPitchDeg: Float?,
        val absRollDeg: Float?,

        val rotationM00: Float?,

        val rotationM01: Float?,

        val rotationM02: Float?,

        val rotationM10: Float?,

        val rotationM11: Float?,

        val rotationM12: Float?,

        val rotationM20: Float?,

        val rotationM21: Float?,

        val rotationM22: Float?,

        val capturedAtUtcMs: Long
    )

    private val displayFilter = AngleFilterState()
    private val captureFilter = AngleFilterState()

    private var lastSensorTsNs: Long = 0L

    private var displayAzimuth = 0f
    private var displayPitchDeg = 0f
    private var displayRollDeg = 0f

    private var zenithPitchSmoothedDeg = 0f
    private var zenithRollSmoothedDeg = 0f
    private var zenithDisplayInitialized = false
    private var zenithYawLockedDeg = 0f
    private var zenithYawLocked = false

    private var zenithAssistLatched = false

    // Deben coincidir con GuideView para que el target visual y la lógica de captura
// entren/salgan del modo zenital en el mismo momento.
    private val zenithAssistEnterDeg = 70f

    private val zenithAssistExitDeg = 64f
    // -------------------- ordenar por azimuth --------------------
    private val azRegex = Regex("""_az(\d{3})_""")

    private fun azFromName(f: File): Int =
        azRegex.find(f.name)?.groupValues?.get(1)?.toIntOrNull() ?: Int.MAX_VALUE

    /**
     * Ordena ascendente, pero “rota” el array para evitar que el salto grande (p.ej 330->0)
     * quede en el medio. (Útil SOLO cuando NO es loop completo)
     */
    private fun orderByAzimuthContinuous(files: List<File>): List<File> {
        val pairs = files.map { it to azFromName(it) }
            .filter { it.second != Int.MAX_VALUE }
            .sortedBy { it.second }

        if (pairs.size <= 2) return files

        var bestIdx = 0
        var maxGap = -1

        for (i in pairs.indices) {
            val a = pairs[i].second
            val b = pairs[(i + 1) % pairs.size].second
            val gap = if (i == pairs.lastIndex) (pairs.first().second + 360 - a) else (b - a)
            if (gap > maxGap) {
                maxGap = gap
                bestIdx = (i + 1) % pairs.size
            }
        }

        return (pairs.drop(bestIdx) + pairs.take(bestIdx)).map { it.first }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_capture)
        if (!SunProjectApp.ensureOpenCvReady()) {
            Log.e("OpenCV", "OpenCV initialization failed")
            Toast.makeText(this, "OpenCV initialization failed", Toast.LENGTH_LONG).show()
            finish()
            return
        } else {
            Log.i("OpenCV", "OpenCV loaded successfully")
        }
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        cameraPreview = findViewById(R.id.cameraPreview)
        guideView = findViewById(R.id.guideView)
        debugAzimuth = findViewById(R.id.debugAzimuth)
        debugPitch = findViewById(R.id.debugPitch)
        debugRoll = findViewById(R.id.debugRoll)
        debugLocation = findViewById(R.id.debugLocation)
        debugCaptureCount = findViewById(R.id.debugCaptureCount)
        btnCapture = findViewById(R.id.btnCapture)
        btnAuto = findViewById(R.id.btnAuto)
        maxShots = guideView.getCapturePlanSize()
        updateCaptureCountUi()

        cameraExecutor = Executors.newSingleThreadExecutor()
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)
        setupLocationCallback()

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        gameRotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        magneticFieldSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        btnCapture.setOnClickListener { takePhotoManual() }

        // Long-press para FORZAR stitch con 2+ fotos (debug)
        btnCapture.setOnLongClickListener {
            Toast.makeText(this, "Forzando stitch…", Toast.LENGTH_SHORT).show()
            stitchCurrentSessionIfReady(force = true)
            true
        }

        btnAuto.setOnClickListener {
            autoCaptureEnabled = !autoCaptureEnabled
            updateAutoButton()
            Toast.makeText(
                this,
                "Auto capture: ${if (autoCaptureEnabled) "ON" else "OFF"}",
                Toast.LENGTH_SHORT
            ).show()
        }

        btnAuto.setOnLongClickListener {
            if (capturedFiles.isEmpty()) {
                Toast.makeText(this, "Captura al menos una foto primero", Toast.LENGTH_SHORT).show()
            } else {
                openProjectedAtlasAll()
            }
            true
        }

        updateAutoButton()
        updateCaptureCountUi()

        if (allPermissionsGranted()) {
            startCamera()
            startLocationUpdates()
            runStitchSelfTestOnce()
        } else {
            ActivityCompat.requestPermissions(this, REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS)
        }
    }

    override fun onResume() {
        super.onResume()
        if (allPermissionsGranted()) startLocationUpdates()

        rotationVectorSensor?.also { sensor ->
            sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME)
        }

        rotationVectorSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }

        gameRotationVectorSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }

        magneticFieldSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
        fusedLocationClient.removeLocationUpdates(locationCallback)
    }

    override fun onDestroy() {
        super.onDestroy()
        try { cameraExecutor.shutdown() } catch (_: Throwable) {}
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        if (sensor?.type == Sensor.TYPE_MAGNETIC_FIELD) {
            magneticAccuracy = accuracy
            Log.d(
                "SunSensors",
                "magAccuracy=$accuracy fieldNorm=${"%.1f".format(magneticFieldNormUt)}uT"
            )
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_MAGNETIC_FIELD -> {
                val x = event.values[0]
                val y = event.values[1]
                val z = event.values[2]
                magneticFieldNormUt = sqrt(x * x + y * y + z * z)
            }

            Sensor.TYPE_ROTATION_VECTOR -> {
                updateLastAbsRotationMatrix(event.values)
                val raw = extractAnglesFromRotationVector(event.values)

                val rawAz = applyDeclination(raw[0])
                val rawPitch = raw[1]
                val rawRoll = raw[2]

                val dtSec = sensorDtSec(event.timestamp, lastAbsTsNs)
                lastAbsTsNs = event.timestamp

                val alpha = adaptiveAlpha(dtSec, tauSec = 0.10f)
                val captureAngles = filterAngles(captureFilter, rawAz, rawPitch, rawRoll, alpha)

                lastAzimuth = captureAngles[0]

                lastPitch = captureAngles[1]

                lastRoll = captureAngles[2]

                absoluteYawDeg = lastAzimuth
                absolutePitchDeg = lastPitch
                absoluteRollDeg = lastRoll

                if (!displayOffsetInitialized && gameRotationVectorSensor == null) {
                    displayAzimuth = lastAzimuth
                    displayPitchDeg = lastPitch
                    displayRollDeg = lastRoll
                    Log.d(
                        "SunSensorFusion",
                        "gameYaw=${"%.2f".format(gameYawDeg)} " +
                                "gamePitch=${"%.2f".format(gamePitchDeg)} " +
                                "gameRoll=${"%.2f".format(gameRollDeg)} " +
                                "dispYaw=${"%.2f".format(displayAzimuth)} " +
                                "dispPitch=${"%.2f".format(displayPitchDeg)} " +
                                "dispRoll=${"%.2f".format(displayRollDeg)} " +
                                "zenith="
                    )
                    guideView.updateOrientation(
                        displayAzimuth,
                        displayPitchDeg,
                        displayRollDeg,
                        absolutePitchDeg
                    )
                    checkAutoCapture()

                }
            }

            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                val raw = extractAnglesFromRotationVector(event.values)

                val dtSec = sensorDtSec(event.timestamp, lastGameTsNs)
                lastGameTsNs = event.timestamp

                val alpha = adaptiveAlpha(dtSec, tauSec = 0.22f)
                val displayAngles = filterAngles(gameDisplayFilter, raw[0], raw[1], raw[2], alpha)

                gameYawDeg = displayAngles[0]
                gamePitchDeg = displayAngles[1]
                gameRollDeg = displayAngles[2]

                if (!displayOffsetInitialized && absoluteYawDeg != 0f) {
                    displayNorthOffsetDeg = normalize360(absoluteYawDeg - gameYawDeg)
                    displayOffsetInitialized = true
                }

                val zenithTargetActive =
                    guideView.isZenithStageActive() || isZenithTarget(guideView.getActiveCapturePoint())

                val nearZenith = updateZenithAssistLatch(zenithTargetActive, absolutePitchDeg)

                val baseDisplayYaw = if (displayOffsetInitialized) {
                    normalize360(gameYawDeg + displayNorthOffsetDeg)
                } else {
                    absoluteYawDeg
                }

                if (nearZenith) {
                    val zenithAbsPitch = absolutePitchDeg
                    val zenithAbsRoll = absoluteRollDeg

                    if (!zenithYawLocked && zenithAbsPitch >= 82f) {
                        zenithYawLockedDeg = baseDisplayYaw
                        zenithYawLocked = true
                        zenithLockEnteredMs = SystemClock.elapsedRealtime()
                    }

                    if (!zenithDisplayInitialized) {
                        zenithPitchSmoothedDeg = zenithAbsPitch
                        zenithRollSmoothedDeg = zenithAbsRoll
                        zenithDisplayInitialized = true
                    }

                    zenithPitchSmoothedDeg =
                        0.90f * zenithPitchSmoothedDeg + 0.10f * zenithAbsPitch

                    zenithRollSmoothedDeg =
                        0.92f * zenithRollSmoothedDeg + 0.08f * zenithAbsRoll

                    displayAzimuth = normalizeDeg(
                        if (zenithYawLocked) zenithYawLockedDeg else baseDisplayYaw
                    )

                    // En zenith ya no remapeamos el pitch a una escala artificial.
                    // GuideView ya centra el target zenital por su cuenta.
                    displayPitchDeg = zenithPitchSmoothedDeg

                    displayRollDeg = when {
                        zenithAbsPitch >= 86f -> 0f
                        zenithAbsPitch >= 82f -> zenithRollSmoothedDeg * 0.02f
                        zenithAbsPitch >= 78f -> zenithRollSmoothedDeg * 0.05f
                        else -> zenithRollSmoothedDeg * 0.10f
                    }

                } else {
                    zenithDisplayInitialized = false
                    zenithYawLocked = false
                    zenithLockEnteredMs = 0L
                    zenithPitchSmoothedDeg = displayPitchDeg
                    zenithRollSmoothedDeg = displayRollDeg

                    displayAzimuth = normalizeDeg(gameYawDeg + displayNorthOffsetDeg)
                    displayPitchDeg = gamePitchDeg
                    displayRollDeg = gameRollDeg
                }

                guideView.updateOrientation(
                    displayAzimuth,
                    displayPitchDeg,
                    displayRollDeg,
                    absolutePitchDeg
                )

                Log.d(
                    "SunSensorFusion",
                    "absYaw=${"%.2f".format(absoluteYawDeg)} gameYaw=${"%.2f".format(gameYawDeg)} " +
                            "dispYaw=${"%.2f".format(displayAzimuth)} dispPitch=${"%.2f".format(displayPitchDeg)} " +
                            "dispRoll=${"%.2f".format(displayRollDeg)} nearZenith=$nearZenith"
                )

                debugAzimuth.text = "Azimuth: ${displayAzimuth.toInt()}°"
                debugPitch.text = "Pitch: ${displayPitchDeg.toInt()}°"
                debugRoll.text = "Roll: ${displayRollDeg.toInt()}°"

                Log.d(
                    "SunSensors",
                    "absYaw=${"%.2f".format(absoluteYawDeg)} " +
                            "gameYaw=${"%.2f".format(gameYawDeg)} " +
                            "offset=${"%.2f".format(displayNorthOffsetDeg)} " +
                            "dispYaw=${"%.2f".format(displayAzimuth)} " +
                            "magAcc=$magneticAccuracy " +
                            "field=${"%.1f".format(magneticFieldNormUt)}uT"
                )

                checkAutoCapture()
            }
        }
    }

    private fun checkAutoCapture() {
        if (!autoCaptureEnabled) {
            alignmentStartMs = 0L
            return
        }

        if (capturedFiles.size >= maxShots) {
            autoCaptureEnabled = false
            updateAutoButton()
            return
        }

        if (imageCapture == null) return

        val target = guideView.getActiveCapturePoint() ?: return
        val zenithMode = isZenithTarget(target)
        val now = SystemClock.elapsedRealtime()

        if (zenithMode) {
            val capturePitchDeg = absolutePitchDeg
            val pitchErr = abs(capturePitchDeg - target.pitch)
            val aligned = pitchErr <= 2.0f

            Log.d(
                "SunGuideZenithAuto",
                "targetPitch=${"%.2f".format(target.pitch)} " +
                        "capturePitch=${"%.2f".format(capturePitchDeg)} " +
                        "pitchErr=${"%.2f".format(pitchErr)} " +
                        "aligned=$aligned"
            )

            if (!aligned) {
                alignmentStartMs = 0L
                return
            }

            if (!hasLastAbsRotationMatrix) {
                alignmentStartMs = 0L
                return
            }

            val settled = zenithLockEnteredMs == 0L ||
                    (now - zenithLockEnteredMs) >= 180L

            if (!settled) {
                alignmentStartMs = 0L
                return
            }

            if (alignmentStartMs == 0L) {
                alignmentStartMs = now
                zenithAbsPitchAnchorDeg = absolutePitchDeg
                return
            }

            val absPitchDrift = abs(absolutePitchDeg - zenithAbsPitchAnchorDeg)
            val grosslyUnstable = absPitchDrift > 1.2f

            Log.d(
                "SunGuideZenithHold",
                "pitchDrift=${"%.2f".format(absPitchDrift)} " +
                        "settled=$settled grosslyUnstable=$grosslyUnstable"
            )

            if (grosslyUnstable) {
                alignmentStartMs = 0L
                return
            }

            val held = (now - alignmentStartMs) >= 650L
            val cooled = (now - lastShotMs) >= cooldownMs

            if (held && cooled) {
                takePhotoForTarget(target, reason = "AUTO")
                alignmentStartMs = 0L
            }
            return
        }

        val azErr = absAngleDiff(displayAzimuth, target.azimuth)
        val pitchErr = abs(displayPitchDeg - target.pitch)
        val rollErr = abs(displayRollDeg)

        val aligned =
            azErr <= azTolDeg &&
                    pitchErr <= pitchTolDeg &&
                    rollErr <= rollTolDeg

        if (!aligned) {
            alignmentStartMs = 0L
            return
        }

        if (alignmentStartMs == 0L) alignmentStartMs = now

        val held = (now - alignmentStartMs) >= holdMs
        val cooled = (now - lastShotMs) >= cooldownMs

        if (held && cooled) {
            takePhotoForTarget(target, reason = "AUTO")
            alignmentStartMs = 0L
        }
    }
    private fun absAngleDiff(a: Float, b: Float): Float {
        var d = (a - b) % 360f
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return abs(d)
    }

    private fun updateAutoButton() {
        btnAuto.text = if (autoCaptureEnabled) "Auto: ON" else "Auto: OFF"
    }

    private fun updateCaptureCountUi() {
        debugCaptureCount.text = "Capturas: ${capturedFiles.size}/$maxShots"
    }

    private fun ensureSessionDir(): File {
        sessionDir?.let { return it }

        val base = File(getExternalFilesDir(null), "sessions")
        if (!base.exists()) base.mkdirs()

        val ts = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val dir = File(base, "session_$ts")
        dir.mkdirs()

        sessionDir = dir
        currentSessionId = dir.name
        sessionPaths = sessionStore.createSessionPaths(dir)

        Log.i("PanoramaCAP", "Session dir: ${dir.absolutePath}")
        runOnUiThread { Toast.makeText(this, "Sesión: ${dir.name}", Toast.LENGTH_SHORT).show() }

        val loc = lastLocation

        if (sessionYawAnchorDeg == null) sessionYawAnchorDeg = displayAzimuth
        if (sessionPitchAnchorDeg == null) sessionPitchAnchorDeg = displayPitchDeg
        if (sessionRollAnchorDeg == null) sessionRollAnchorDeg = displayRollDeg

        val session = SessionRecord(
            sessionId = dir.name,
            startedAtUtcMs = System.currentTimeMillis(),
            latitudeDeg = loc?.latitude,
            longitudeDeg = loc?.longitude,
            altitudeM = loc?.altitude,
            declinationDeg = null,
            cameraId = currentCameraId ?: "widest_back_camera",
            sensorMode = "rotation_vector",
            sessionYawAnchorDeg = sessionYawAnchorDeg,
            sessionPitchAnchorDeg = sessionPitchAnchorDeg,
            sessionRollAnchorDeg = sessionRollAnchorDeg,
            notes = null
        )
        sessionStore.saveSession(session, sessionPaths!!)

        val meta = File(dir, "metadata.csv")
        if (!meta.exists()) meta.writeText("filename,timestamp_ms,azimuth,pitch,roll,lat,lon,alt\n")
        return dir
    }


    private fun appendMetadata(fileName: String, pose: CapturedPose) {
        val dir = ensureSessionDir()
        val meta = File(dir, "metadata.csv")

        val loc = lastLocation
        val lat = loc?.latitude ?: Double.NaN
        val lon = loc?.longitude ?: Double.NaN
        val alt = loc?.altitude ?: Double.NaN

        val line =
            "${fileName},${pose.capturedAtUtcMs},${pose.azimuthDeg},${pose.pitchDeg},${pose.rollDeg},${lat},${lon},${alt}\n"

        meta.appendText(line)
    }

    private fun takePhotoManual() {
        if (capturedFiles.size >= maxShots) {
            Toast.makeText(this, "Ya se capturaron $maxShots fotos.", Toast.LENGTH_SHORT).show()
            return
        }

        val target = guideView.getActiveCapturePoint()
        if (target != null) takePhotoForTarget(target, reason = "MANUAL")
        else Toast.makeText(this, "No hay punto activo.", Toast.LENGTH_SHORT).show()
    }

    private fun takePhotoForTarget(target: GuideView.CapturePoint, reason: String) {
        val cap = imageCapture ?: return
        val dir = ensureSessionDir()

        val index = capturedFiles.size + 1
        val safeAz = ((target.azimuth.roundToInt() % 360) + 360) % 360
        val safePi = target.pitch.roundToInt()
        val fileName = "img_%02d_az%03d_p%+03d_%s.jpg".format(index, safeAz, safePi, reason)
        val file = File(dir, fileName)

        val frozenPose = CapturedPose(

            azimuthDeg = displayAzimuth,

            pitchDeg = displayPitchDeg,

            rollDeg = displayRollDeg,

            absAzimuthDeg = if (hasLastAbsRotationMatrix) absoluteYawDeg else null,
            absPitchDeg = if (hasLastAbsRotationMatrix) absolutePitchDeg else null,
            absRollDeg = if (hasLastAbsRotationMatrix) absoluteRollDeg else null,

            rotationM00 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[0] else null,

            rotationM01 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[1] else null,

            rotationM02 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[2] else null,

            rotationM10 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[3] else null,

            rotationM11 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[4] else null,

            rotationM12 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[5] else null,

            rotationM20 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[6] else null,

            rotationM21 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[7] else null,

            rotationM22 = if (hasLastAbsRotationMatrix) lastAbsRotationMatrix[8] else null,

            capturedAtUtcMs = System.currentTimeMillis()
        )

        Log.i(
            "PanoramaCAP",
            "Capturing -> ${file.name} (az=$safeAz p=$safePi) " +
                    "pose=${"%.2f".format(frozenPose.azimuthDeg)}/${"%.2f".format(frozenPose.pitchDeg)}/${"%.2f".format(frozenPose.rollDeg)}"
        )

        val outputOptions = ImageCapture.OutputFileOptions.Builder(file).build()
        lastShotMs = SystemClock.elapsedRealtime()

        cap.takePicture(
            outputOptions,
            cameraExecutor,
            object : ImageCapture.OnImageSavedCallback {
                override fun onImageSaved(outputFileResults: ImageCapture.OutputFileResults) {
                    capturedFiles.add(file)

                    appendMetadata(file.name, frozenPose)
                    appendFrameRecord(file, target, capturedFiles.size, frozenPose)

                    target.isCaptured = true
                    Log.d("SunGuideStage", "after_save ${guideView.getStageDebugString()}")

                    runOnUiThread {
                        guideView.updateOrientation(
                            displayAzimuth,
                            displayPitchDeg,
                            displayRollDeg,
                            absolutePitchDeg
                        )
                        guideView.invalidate()
                    }

                    Log.i("PanoramaCAP", "Saved: ${file.absolutePath} (${capturedFiles.size}/$maxShots)")

                    runOnUiThread {
                        updateCaptureCountUi()

                        Toast.makeText(
                            this@CaptureActivity,
                            "Foto ${capturedFiles.size}/$maxShots guardada",
                            Toast.LENGTH_SHORT
                        ).show()

                        if (capturedFiles.size >= maxShots) {
                            autoCaptureEnabled = false
                            updateAutoButton()

                            Toast.makeText(
                                this@CaptureActivity,
                                "Listo: ${capturedFiles.size} fotos.\nGenerando atlas angular...",
                                Toast.LENGTH_LONG
                            ).show()

                            openAngularAtlasDebug()
                        }
                    }
                }

                override fun onError(exception: ImageCaptureException) {
                    Log.e("PanoramaCAP", "Capture error", exception)
                    runOnUiThread {
                        Toast.makeText(
                            this@CaptureActivity,
                            "Error captura: ${exception.message}",
                            Toast.LENGTH_LONG
                        ).show()
                    }
                }
            }
        )
    }

    private fun allPermissionsGranted() = REQUIRED_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }

    private fun startCamera() {
        cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            val cameraProvider = cameraProviderFuture.get()
            bindPreview(cameraProvider)
        }, ContextCompat.getMainExecutor(this))
    }

    @OptIn(ExperimentalCamera2Interop::class)
    private fun selectWidestBackCameraSelector(cameraProvider: ProcessCameraProvider): CameraSelector {
        val backInfos = cameraProvider.availableCameraInfos.filter { info ->
            val c = info.getCameraCharacteristics()
            c.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK
        }
        if (backInfos.isEmpty()) return CameraSelector.DEFAULT_BACK_CAMERA

        fun fovDeg(info: CameraInfo): Float {
            val c = info.getCameraCharacteristics()
            val sensorSize: SizeF? = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val focals: FloatArray? = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val orientation: Int = c.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 0
            if (sensorSize == null || focals == null || focals.isEmpty()) return 0f

            var minFocal = focals[0]
            for (v in focals) if (v < minFocal) minFocal = v

            val sensorWidthMm =
                if (orientation == 90 || orientation == 270) sensorSize.height else sensorSize.width

            val fov = 2.0 * atan((sensorWidthMm / (2.0f * minFocal)).toDouble())
            return Math.toDegrees(fov).toFloat()
        }

        val best = backInfos.maxByOrNull { fovDeg(it) } ?: backInfos.first()
        val bestFov = fovDeg(best)
        Log.i("PanoramaCAP", "Selected widest BACK camera, approx HFOV=$bestFov deg")

        return CameraSelector.Builder()
            .addCameraFilter { infos -> infos.filter { it === best } }
            .build()
    }

    @OptIn(ExperimentalCamera2Interop::class)
    private fun bindPreview(cameraProvider: ProcessCameraProvider) {
        cameraPreview.scaleType = PreviewView.ScaleType.FIT_CENTER
        // cameraPreview.scaleType = PreviewView.ScaleType.FILL_CENTER

        val rotation = cameraPreview.display.rotation

        val preview = Preview.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setTargetRotation(rotation)
            .build()

        val cameraSelector = selectWidestBackCameraSelector(cameraProvider)

        val cap = ImageCapture.Builder()
            .setCaptureMode(ImageCapture.CAPTURE_MODE_MINIMIZE_LATENCY)
            .setTargetRotation(rotation)
            .build()
        imageCapture = cap

        preview.setSurfaceProvider(cameraPreview.surfaceProvider)

        cameraProvider.unbindAll()
        val camera = cameraProvider.bindToLifecycle(this, cameraSelector, preview, cap)

        try { camera.cameraControl.setLinearZoom(0f) } catch (_: Throwable) {}

        val characteristics = camera.cameraInfo.getCameraCharacteristics()

        currentCameraId = try {
            Camera2CameraInfo.from(camera.cameraInfo).cameraId
        } catch (_: Throwable) {
            "widest_back_camera"
        }

        val sensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 0
        val sensorSize: SizeF? = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
        val focalLengths: FloatArray? = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)

        if (sensorSize != null && focalLengths != null && focalLengths.isNotEmpty()) {
            var minFocal = focalLengths[0]
            for (v in focalLengths) if (v < minFocal) minFocal = v

            val sensorWidthMm =
                if (sensorOrientation == 90 || sensorOrientation == 270) sensorSize.height else sensorSize.width
            val sensorHeightMm =
                if (sensorOrientation == 90 || sensorOrientation == 270) sensorSize.width else sensorSize.height

            val hfov = 2.0 * atan((sensorWidthMm / (2.0 * minFocal)).toDouble())
            val vfov = 2.0 * atan((sensorHeightMm / (2.0 * minFocal)).toDouble())

            currentHfovDeg = Math.toDegrees(hfov).toFloat()
            currentVfovDeg = Math.toDegrees(vfov).toFloat()

            guideView.setCameraFov(currentHfovDeg ?: 0f)

            Log.i(
                "PanoramaCAP",
                "CameraId=$currentCameraId HFOV=${currentHfovDeg} VFOV=${currentVfovDeg}"
            )
        }
    }

    private fun setupLocationCallback() {
        locationCallback = object : LocationCallback() {
            override fun onLocationResult(locationResult: LocationResult) {
                locationResult.lastLocation?.let { location ->
                    lastLocation = location
                    val locationText =
                        "Lat: ${"%.4f".format(location.latitude)}\nLon: ${"%.4f".format(location.longitude)}\nAlt: ${location.altitude.toInt()}m"
                    debugLocation.text = locationText
                }
            }
        }
    }

    private fun startLocationUpdates() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) return

        val locationRequest = LocationRequest.Builder(
            Priority.PRIORITY_HIGH_ACCURACY,
            TimeUnit.SECONDS.toMillis(5)
        ).build()

        fusedLocationClient.requestLocationUpdates(locationRequest, locationCallback, Looper.getMainLooper())
    }

    private fun runStitchSelfTestOnce() {
        if (stitchSelfTestRan) return
        stitchSelfTestRan = true

        cameraExecutor.execute {
            try {
                val ok = NativeStitcher.checkStitcherAvailable()
                val jpeg = NativeStitcher.selfTestStitch()

                runOnUiThread {
                    Toast.makeText(
                        this,
                        "Stitcher=$ok  selfTest=${jpeg?.size ?: 0} bytes",
                        Toast.LENGTH_LONG
                    ).show()
                }

                if (jpeg != null) {
                    val f = File(cacheDir, "pano_selftest.jpg")
                    f.writeBytes(jpeg)
                    Log.i("PanoramaJNI", "SelfTest saved: ${f.absolutePath}")
                }
            } catch (t: Throwable) {
                Log.e("PanoramaJNI", "SelfTest failed", t)
                runOnUiThread {
                    Toast.makeText(this, "SelfTest failed: ${t.message}", Toast.LENGTH_LONG).show()
                }
            }
        }
    }

    private fun stitchCurrentSessionIfReady(force: Boolean) {
        val need = if (force) 2 else maxShots

        if (stitchDone || stitchInProgress) return
        if (capturedFiles.size < need) {
            Log.i("PanoramaJNI", "not enough images (${capturedFiles.size}/$need)")
            return
        }

        val dir = sessionDir ?: ensureSessionDir()
        val outFile = File(dir, "pano_real.jpg")

        Log.i("PanoramaJNI", "capturedFiles.size=${capturedFiles.size}")
        capturedFiles.forEachIndexed { i, f ->
            Log.i("PanoramaJNI", "captured[$i]=${f.name} az=${azFromName(f)}")
        }


        val byAz = capturedFiles.sortedBy { azFromName(it) } // 0..330
        val firstAz = azFromName(byAz.first()).toFloat()
        val lastAz  = azFromName(byAz.last()).toFloat()
        val span = ((lastAz - firstAz + 360f) % 360f)

        val full360 = (byAz.size >= maxShots && span >= 330f)
        val closeLoop = true

        val orderedFiles = if (closeLoop) {
            byAz
        } else {
            orderByAzimuthContinuous(byAz)
        }

        Log.i("PanoramaJNI", "Ordered by azimuth:")
        orderedFiles.forEach { Log.i("PanoramaJNI", " - ${it.name} az=${azFromName(it)}") }
        Log.i("PanoramaJNI", "closeLoop=$closeLoop span=$span firstAz=$firstAz lastAz=$lastAz n=${orderedFiles.size}")

        val paths = orderedFiles.map { it.absolutePath }.toTypedArray()

        stitchInProgress = true
        Log.e("PanoramaJNI", "Starting real stitch with ${paths.size} images (force=$force closeLoop=$closeLoop)")

        cameraExecutor.execute {
            try {
                val md = if (paths.size >= 10) 1600 else 1800
                //val md = 1800
                val jpeg = NativeStitcher.stitchFromFiles(
                    paths = paths,
                    jpegQuality = 92,
                    maxDim = md,
                    closeLoop = closeLoop
                )

                if (jpeg != null) {
                    outFile.writeBytes(jpeg)
                    stitchDone = true
                    Log.i("PanoramaJNI", "Real pano saved: ${outFile.absolutePath} size=${jpeg.size}")
                } else {
                    Log.w("PanoramaJNI", "stitchFromFiles returned null")
                }
            } catch (t: Throwable) {
                Log.e("PanoramaJNI", "Real stitch failed", t)
            } finally {
                stitchInProgress = false
            }
        }
    }

    private fun readImageSize(file: File): Pair<Int?, Int?> {
        return try {
            val opts = android.graphics.BitmapFactory.Options().apply {
                inJustDecodeBounds = true
            }
            android.graphics.BitmapFactory.decodeFile(file.absolutePath, opts)
            val w = opts.outWidth.takeIf { it > 0 }
            val h = opts.outHeight.takeIf { it > 0 }
            w to h
        } catch (_: Throwable) {
            null to null
        }
    }

    private fun appendFrameRecord(
        file: File,
        target: GuideView.CapturePoint,
        shotIndex: Int,
        pose: CapturedPose
    ) {
        val paths = sessionPaths ?: return
        val loc = lastLocation
        val (imgW, imgH) = readImageSize(file)

        val frame = FrameRecord(

            frameId = file.nameWithoutExtension,

            sessionId = currentSessionId ?: "unknown_session",

            ringId = ringIdForTarget(target),

            shotIndex = shotIndex,

            originalPath = file.absolutePath,

            capturedAtUtcMs = pose.capturedAtUtcMs,
            targetAzimuthDeg = target.azimuth,

            targetPitchDeg = target.pitch,

            measuredAzimuthDeg = pose.azimuthDeg,

            measuredPitchDeg = pose.pitchDeg,

            measuredRollDeg = pose.rollDeg,

            absAzimuthDeg = pose.absAzimuthDeg,
            absPitchDeg = pose.absPitchDeg,
            absRollDeg = pose.absRollDeg,

            rotationM00 = pose.rotationM00,

            rotationM01 = pose.rotationM01,

            rotationM02 = pose.rotationM02,

            rotationM10 = pose.rotationM10,

            rotationM11 = pose.rotationM11,

            rotationM12 = pose.rotationM12,

            rotationM20 = pose.rotationM20,

            rotationM21 = pose.rotationM21,

            rotationM22 = pose.rotationM22,

            latitudeDeg = loc?.latitude,

            longitudeDeg = loc?.longitude,

            altitudeM = loc?.altitude,

            imageWidthPx = imgW,

            imageHeightPx = imgH,

            hfovDeg = currentHfovDeg,

            vfovDeg = currentVfovDeg
        )

        Log.d(
            "FrameRecordSave",
            "file=${file.name} target=${target.azimuth}/${target.pitch} " +
                    "saved=${"%.2f".format(pose.azimuthDeg)}/${"%.2f".format(pose.pitchDeg)}/${"%.2f".format(pose.rollDeg)} " +
                    "hfov=${currentHfovDeg} vfov=${currentVfovDeg}"
        )

        sessionStore.appendFrame(frame, paths)
    }

    private fun ringIdForTarget(target: GuideView.CapturePoint): String = when {
        target.pitch >= 80f -> "Z0"
        target.pitch >= 30f -> "H45"
        else -> "H0"
    }

    private fun openProjectedAtlasAll() {
        val dir = sessionDir ?: ensureSessionDir()

        cameraExecutor.execute {
            try {
                val atlasFile = atlasBuildUseCase.buildProjectedAtlas(dir)

                runOnUiThread {
                    Toast.makeText(this, "Atlas multi-frame generado", Toast.LENGTH_SHORT).show()
                    startActivity(
                        Intent(this, AnalysisActivity::class.java).apply {
                            putExtra("panorama_path", atlasFile.absolutePath)
                        }
                    )
                }
            } catch (t: Throwable) {
                runOnUiThread {
                    Toast.makeText(
                        this,
                        "Error generando atlas multi-frame: ${t.message}",
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_CODE_PERMISSIONS) {
            if (allPermissionsGranted()) {
                startCamera()
                startLocationUpdates()
            } else {
                Toast.makeText(this, "Permissions not granted.", Toast.LENGTH_SHORT).show()
                finish()
            }
        }
    }

    private fun openAngularAtlasDebug() {
        val dir = sessionDir ?: ensureSessionDir()

        cameraExecutor.execute {
            try {
                val atlasFile = atlasBuildUseCase.buildDebugAtlas(dir)

                runOnUiThread {
                    Toast.makeText(this, "Atlas debug generado", Toast.LENGTH_SHORT).show()
                    startActivity(
                        Intent(this, AnalysisActivity::class.java).apply {
                            putExtra("panorama_path", atlasFile.absolutePath)
                        }
                    )
                }
            } catch (t: Throwable) {
                runOnUiThread {
                    Toast.makeText(
                        this,
                        "Error generando atlas debug: ${t.message}",
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }
    }

    private fun normalize360(deg: Float): Float {
        var v = deg % 360f
        if (v < 0f) v += 360f
        return v
    }

    private fun angleDeltaDeg(a: Float, b: Float): Float {
        var d = normalize360(a) - normalize360(b)
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return d
    }

    private fun adaptiveAlpha(dtSec: Float, tauSec: Float): Float {
        return (1f - kotlin.math.exp(-dtSec / tauSec)).coerceIn(0.01f, 1f)
    }

    private fun deadbandCircular(prev: Float, next: Float, bandDeg: Float): Float {
        return if (kotlin.math.abs(angleDeltaDeg(next, prev)) < bandDeg) prev else next
    }

    private fun deadbandLinear(prev: Float, next: Float, bandDeg: Float): Float {
        return if (kotlin.math.abs(next - prev) < bandDeg) prev else next
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
        sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    private fun normalize(v: Vec3): Vec3 {
        val n = norm(v)
        if (n < 1e-6f) return Vec3(0f, 0f, 0f)
        return Vec3(v.x / n, v.y / n, v.z / n)
    }

    private fun add(a: Vec3, b: Vec3): Vec3 =
        Vec3(a.x + b.x, a.y + b.y, a.z + b.z)

    private fun scale(v: Vec3, s: Float): Vec3 =
        Vec3(v.x * s, v.y * s, v.z * s)

    private fun worldUpFromAngles(azDeg: Float, pitchDeg: Float, rollDeg: Float): Vec3 {
        val forward = worldForwardFromAngles(azDeg, pitchDeg)
        var right0 = cross(forward, Vec3(0f, 0f, 1f))
        if (norm(right0) < 1e-4f) right0 = Vec3(1f, 0f, 0f)
        right0 = normalize(right0)

        val up0 = normalize(cross(right0, forward))

        val roll = Math.toRadians(rollDeg.toDouble())
        val cosR = cos(roll).toFloat()
        val sinR = sin(roll).toFloat()

        return normalize(
            add(
                scale(up0, cosR),
                scale(right0, -sinR)
            )
        )
    }

    private fun rollFromBasis(forward: Vec3, up: Vec3): Float {
        var right0 = cross(forward, Vec3(0f, 0f, 1f))
        if (norm(right0) < 1e-4f) right0 = Vec3(1f, 0f, 0f)
        right0 = normalize(right0)

        val up0 = normalize(cross(right0, forward))

        val sinR = -dot(up, right0)
        val cosR = dot(up, up0)

        return Math.toDegrees(atan2(sinR.toDouble(), cosR.toDouble())).toFloat()
    }

    private fun updateGuidePose(rawAzDeg: Float, rawPitchDeg: Float, rawRollDeg: Float, alpha: Float) {
        val rawForward = worldForwardFromAngles(rawAzDeg, rawPitchDeg)
        val rawUp = worldUpFromAngles(rawAzDeg, rawPitchDeg, rawRollDeg)

        if (!guidePoseInitialized) {
            guideForward = rawForward
            guideUp = rawUp
            guidePoseInitialized = true
        } else {
            guideForward = normalize(
                add(
                    scale(guideForward, 1f - alpha),
                    scale(rawForward, alpha)
                )
            )

            val upCandidate = normalize(
                add(
                    scale(guideUp, 1f - alpha),
                    scale(rawUp, alpha)
                )
            )

            var right = cross(guideForward, upCandidate)
            if (norm(right) < 1e-4f) {
                right = cross(guideForward, Vec3(0f, 0f, 1f))
            }
            right = normalize(right)
            guideUp = normalize(cross(right, guideForward))
        }

        guideAzimuthDeg = normalize360(
            Math.toDegrees(atan2(guideForward.x.toDouble(), guideForward.y.toDouble())).toFloat()
        )
        guidePitchDeg = Math.toDegrees(
            asin(guideForward.z.coerceIn(-1f, 1f).toDouble())
        ).toFloat()
        guideRollDeg = rollFromBasis(guideForward, guideUp)
    }

    private fun directionErrorDeg(targetAzDeg: Float, targetPitchDeg: Float): Float {
        val targetDir = worldForwardFromAngles(targetAzDeg, targetPitchDeg)
        val d = dot(guideForward, targetDir).coerceIn(-1f, 1f)
        return Math.toDegrees(acos(d.toDouble())).toFloat()
    }

    private fun zenithErrorDeg(): Float {
        val d = guideForward.z.coerceIn(-1f, 1f)
        return Math.toDegrees(acos(d.toDouble())).toFloat()
    }

    private fun maybeLogGuide(target: GuideView.CapturePoint?, aligned: Boolean, dirErr: Float, zenErr: Float) {
        val now = SystemClock.elapsedRealtime()
        if (now - lastGuideLogMs < 250L) return
        lastGuideLogMs = now

        Log.d(
            "SunGuide",
            "target=${target?.azimuth}/${target?.pitch} " +
                    "guideAz=${"%.2f".format(guideAzimuthDeg)} guidePitch=${"%.2f".format(guidePitchDeg)} guideRoll=${"%.2f".format(guideRollDeg)} " +
                    "measuredAz=${"%.2f".format(lastAzimuth)} measuredPitch=${"%.2f".format(lastPitch)} measuredRoll=${"%.2f".format(lastRoll)} " +
                    "dirErr=${"%.2f".format(dirErr)} zenErr=${"%.2f".format(zenErr)} aligned=$aligned"
        )
    }

    private fun filterAngles(
        state: AngleFilterState,
        rawAzimuth: Float,
        rawPitch: Float,
        rawRoll: Float,
        alpha: Float
    ): FloatArray {
        val rawYawX = kotlin.math.cos(Math.toRadians(rawAzimuth.toDouble())).toFloat()
        val rawYawY = kotlin.math.sin(Math.toRadians(rawAzimuth.toDouble())).toFloat()

        if (!state.initialized) {
            state.yawX = rawYawX
            state.yawY = rawYawY
            state.pitch = rawPitch
            state.roll = rawRoll
            state.initialized = true
        } else {
            state.yawX += alpha * (rawYawX - state.yawX)
            state.yawY += alpha * (rawYawY - state.yawY)
            state.pitch += alpha * (rawPitch - state.pitch)
            state.roll += alpha * (rawRoll - state.roll)
        }

        val yaw = normalize360(
            Math.toDegrees(kotlin.math.atan2(state.yawY.toDouble(), state.yawX.toDouble())).toFloat()
        )

        return floatArrayOf(yaw, state.pitch, state.roll)
    }

    private fun isZenithTarget(target: GuideView.CapturePoint?): Boolean {
        return target?.pitch?.let { it >= 80f } == true
    }

    private fun effectiveGuideRoll(rollDeg: Float, zenithMode: Boolean): Float {
        return if (zenithMode) rollDeg * 0.18f else rollDeg
    }

    private fun openProjectedSingleFrame(frameIndex: Int = 0) {
        val dir = sessionDir ?: ensureSessionDir()

        cameraExecutor.execute {
            try {
                val atlasFile = atlasBuildUseCase.buildSingleFrameProjection(dir, frameIndex)

                runOnUiThread {
                    Toast.makeText(this, "Atlas proyectado generado", Toast.LENGTH_SHORT).show()
                    startActivity(
                        Intent(this, AnalysisActivity::class.java).apply {
                            putExtra("panorama_path", atlasFile.absolutePath)
                        }
                    )
                }
            } catch (t: Throwable) {
                Log.e("AtlasBuild", "Error generando atlas multi-frame", t)
                runOnUiThread {
                    Toast.makeText(
                        this,
                        "Error generando atlas multi-frame: ${t.message}",
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }
    }

    private fun sensorDtSec(tsNs: Long, lastTsNs: Long): Float {
        return if (lastTsNs == 0L) {
            0.016f
        } else {
            ((tsNs - lastTsNs).toDouble() / 1_000_000_000.0)
                .toFloat()
                .coerceIn(0.001f, 0.05f)
        }
    }

    private fun extractAnglesFromRotationVector(values: FloatArray): FloatArray {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, values)

        SensorManager.remapCoordinateSystem(
            rotationMatrix,
            SensorManager.AXIS_X,
            SensorManager.AXIS_Z,
            remappedRotationMatrix
        )

        val azimuth = (
                Math.toDegrees(
                    SensorManager.getOrientation(remappedRotationMatrix, orientationAngles)[0].toDouble()
                ).toFloat() + 360f
                ) % 360f

        val pitch = Math.toDegrees(asin(remappedRotationMatrix[7].toDouble())).toFloat()
        val roll = Math.toDegrees(
            -atan2(remappedRotationMatrix[6], remappedRotationMatrix[8]).toDouble()
        ).toFloat()
        Log.d(
            "SunAnglesRaw",
            "az=${"%.2f".format(azimuth)} pitch=${"%.2f".format(pitch)} roll=${"%.2f".format(roll)}"
        )
        return floatArrayOf(azimuth, pitch, roll)
    }
    private fun updateLastAbsRotationMatrix(rotationVectorValues: FloatArray) {
        val rawR = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(rawR, rotationVectorValues)
        System.arraycopy(rawR, 0, lastAbsRotationMatrix, 0, 9)
        hasLastAbsRotationMatrix = true
    }
    private fun normalizeDeg(value: Float): Float {
        var out = value % 360f
        if (out < 0f) out += 360f
        return out
    }
    private fun applyDeclination(azimuthDeg: Float): Float {
        var out = azimuthDeg
        lastLocation?.let {
            val geomagneticField = GeomagneticField(
                it.latitude.toFloat(),
                it.longitude.toFloat(),
                it.altitude.toFloat(),
                System.currentTimeMillis()
            )
            out = normalize360(out + geomagneticField.declination)
        }
        return out
    }

    private fun blendAngleDeg(current: Float, target: Float, alpha: Float): Float {
        return normalize360(current + alpha * angleDeltaDeg(target, current))
    }

    private fun headingReliable(): Boolean {
        return magneticAccuracy >= SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM
    }

    private fun worldForwardFromAngles(azDeg: Float, pitchDeg: Float): Vec3 {
        val az = Math.toRadians(azDeg.toDouble())
        val alt = Math.toRadians(pitchDeg.toDouble())
        val cosAlt = cos(alt).toFloat()

        return normalize(
            Vec3(
                (cosAlt * sin(az)).toFloat(),
                (cosAlt * cos(az)).toFloat(),
                kotlin.math.sin(alt).toFloat()
            )
        )
    }

    private fun updateZenithAssistLatch(zenithTargetActive: Boolean, pitchDeg: Float): Boolean {
        if (!zenithTargetActive) {
            zenithAssistLatched = false
            return false
        }

        val absPitch = kotlin.math.abs(pitchDeg)
        zenithAssistLatched = if (zenithAssistLatched) {
            absPitch >= zenithAssistExitDeg
        } else {
            absPitch >= zenithAssistEnterDeg
        }
        return zenithAssistLatched
    }

    private fun directionErrorDeg(
        camAz: Float,
        camPitch: Float,
        targetAz: Float,
        targetPitch: Float
    ): Float {
        val a = worldForwardFromAngles(camAz, camPitch)
        val b = worldForwardFromAngles(targetAz, targetPitch)

        val d = dot(a, b).coerceIn(-1f, 1f)
        return Math.toDegrees(acos(d.toDouble())).toFloat()
    }
    private fun lowPassScalar(prev: Float, current: Float, alpha: Float): Float {
        return prev + alpha * (current - prev)
    }
    companion object {
        private const val REQUEST_CODE_PERMISSIONS = 10
        private val REQUIRED_PERMISSIONS =
            arrayOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION)
    }
}

@OptIn(ExperimentalCamera2Interop::class)
fun CameraInfo.getCameraCharacteristics(): CameraCharacteristics {
    return Camera2CameraInfo.extractCameraCharacteristics(this)
}