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
import com.example.sunproject.domain.camera.CameraIntrinsicsProbe
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

    // Ring buffer of the last N absolute-yaw samples. Used to compute a
    // time-averaged pose for the Z0 frame, which reduces jitter in the
    // most error-sensitive capture of the session.
    private val zenithPoseBufferSize = 20
    private val zenithYawBuffer = FloatArray(zenithPoseBufferSize)
    private val zenithPitchBuffer = FloatArray(zenithPoseBufferSize)
    private val zenithRollBuffer = FloatArray(zenithPoseBufferSize)
    private var zenithPoseBufferIndex = 0
    private var zenithPoseBufferFilled = false

    // ============================================================
    // STABILITY GATE para H0/H45 (Palanca A)
    // ============================================================
    // Ring buffer de samples del display pose durante el holdMs.
    // Usado para verificar peak-to-peak antes de disparar: el código
    // viejo solo chequea "está dentro de la caja de tolerancia"; éste
    // chequea "ESTUVO QUIETO dentro de la caja durante todo el hold".
    //
    // Tamaño 64: a 60Hz de SENSOR_DELAY_GAME cubre ~1.07s, suficiente
    // para holdMs=800ms con margen.
    private val holdBufferSize = 64
    private val holdYawBuffer = FloatArray(holdBufferSize)
    private val holdPitchBuffer = FloatArray(holdBufferSize)
    private val holdRollBuffer = FloatArray(holdBufferSize)

    // Paralelos a los display* pero con los valores absolutos del IMU
    // (rotation_vector remapeado a Euler global). Usados para promediar
    // absAzimuthDeg/PitchDeg/RollDeg al disparar — Palanca B.
    private val holdAbsYawBuffer = FloatArray(holdBufferSize)
    private val holdAbsPitchBuffer = FloatArray(holdBufferSize)
    private val holdAbsRollBuffer = FloatArray(holdBufferSize)

    private var holdBufferCount = 0
    private var holdBufferIndex = 0

    // Peak-to-peak máximo permitido durante el hold para considerar la
    // pose "quieta". Si excede esto, se resetea el counter (forzar
    // re-estabilización). Punto de partida: 0.8°. Subir si rechaza
    // demasiado, bajar si sigue habiendo ghost.
    private val holdStabilityPeakToPeakDeg = 0.8f
    // Last absolute yaw captured while the phone was far from the Euler
    // singularity (pitch < 65°). Near pitch=90° the AXIS_X/AXIS_Z remap
    // used to compute absoluteYawDeg is gimbal-locked: yaw gets
    // arbitrarily redistributed into the roll channel, so averaging
    // those samples over a 650ms window still gives a physically
    // meaningless number. This snapshot preserves a good yaw from just
    // before the singularity, typically 1-3s before the Z0 shutter.
    private var lastStablePreZenithAbsYawDeg: Float? = null
    private var lastStablePreZenithUpdateMs: Long = 0L

    // ============================================================
    // Z0 GRAV+MAG MATRIX BUFFERS (Fase 5)
    // ============================================================
    // Ring buffers de los últimos N samples de gravedad y campo
    // magnético crudo. En el momento del disparo del Z0 se promedian
    // los componentes (vectores, no ángulos) y se llama
    // SensorManager.getRotationMatrix() para construir la matriz de
    // orientación absoluta sin pasar por TYPE_ROTATION_VECTOR.
    //
    // Tamaño 30 a SENSOR_DELAY_GAME ≈ 150-600ms de ventana. Suficiente
    // para suavizar el ruido del magnetómetro (~1-3° jitter típico).
    private val gravityBufferX = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private val gravityBufferY = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private val gravityBufferZ = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private var gravityBufferIndex = 0
    private var gravityBufferFilled = false

    private val magVecBufferX = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private val magVecBufferY = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private val magVecBufferZ = FloatArray(Z0_MATRIX_BUFFER_SIZE)
    private var magVecBufferIndex = 0
    private var magVecBufferFilled = false

    //private val alpha = 0.08f
    //private var isFirstReading = true
    private data class Vec3(var x: Float, var y: Float, var z: Float)
    private var gameRotationVectorSensor: Sensor? = null
    private var magneticFieldSensor: Sensor? = null
    private var gravitySensor: Sensor? = null
    private var gyroscopeSensor: Sensor? = null

    // Buffer circular del giroscopio para transporte de orientación
    // entre el último H45 y el Z0 (Fase 7).
    private val gyroTransport = com.example.sunproject.domain.sensor.GyroTransport(
        capacity = GYRO_BUFFER_CAPACITY,
        maxGapNs = GYRO_MAX_GAP_NS
    )

    // Snapshot atómico de (matriz rotvec, timestamp) tomado al disparar
    // el último frame H45. Sirve como anchor para propagar la orientación
    // hasta el shutter del Z0 vía integración del giroscopio.
    private val gyroAnchorMatrix = FloatArray(9)
    private var gyroAnchorTimestampNs: Long = 0L
    private var hasGyroAnchor: Boolean = false

    // Buffer circular del rotvec para promediar el anchor en una ventana
    // alrededor del shutter. Refinement 1 de Fase 7: reduce el ruido
    // alta-frecuencia del Kalman interno (jitter ~±0.5-1°) por factor √N.
    private val rotvecBuffer = com.example.sunproject.domain.sensor.RotvecBuffer(
        capacity = ROTVEC_BUFFER_CAPACITY
    )

    // Anchor congelado al inicio de takePhotoForTarget. Sigue el mismo
    // patrón que frozenPose: capturado ANTES del shutter, guardado para
    // que onImageSaved lo persista cuando el callback eventualmente corra
    // (con latencia variable de la cámara).
    //
    // Es la matriz promediada via SVD sobre los últimos
    // ROTVEC_ANCHOR_AVG_WINDOW_MS antes de disparar la cámara.
    // Si la ventana no tiene samples (caso patológico), se cae al
    // snapshot instantáneo de lastAbsRotationMatrix.
    private val frozenAnchorMatrix = FloatArray(9)
    private var frozenAnchorTimestampNs: Long = 0L
    private var hasFrozenAnchor: Boolean = false

    // Vector gravity instantáneo más reciente (en device frame).
    // Se usa para refinar el tilt del Z0 luego del transporte.
    private val instantGravity = FloatArray(3)
    private var hasInstantGravity: Boolean = false

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

    // Tolerancia de roll específica para H45. Validada empíricamente
    // en sesión "roll-cuidadoso": cuando el usuario mantiene roll cerca
    // de 0° también en los H45 (no solo en H0), el ghost de torres y
    // postes altos en zona de solape H0↔H45 se reduce significativamente.
    //
    // Hipótesis activa: la descomposición de la matriz IMU en yaw/pitch/roll
    // con pitch=45° amplifica pequeños sesgos cuando roll es grande en
    // magnitud. Con roll cerca de 0° la fórmula se vuelve trivial
    // (cos(0)=1, sin(0)=0) y el sesgo desaparece o se hace despreciable.
    //
    // Mantenemos rollTolDeg=3° para H0 porque empíricamente los H0
    // capturados ya entran naturalmente en [0, 1.3°] sin esfuerzo
    // extra del usuario. Endurecer H0 no aporta y solo agregaría
    // rejects innecesarios.
    //
    // Si en una sesión el usuario ve muchos rejects en H45 y no logra
    // capturar, se puede aflojar a 2.0f o 2.5f temporalmente.
    private val h45RollTolDeg = 1.5f
    // Zenith tolerances — tightened 2026-04. The Z0 frame is the hardest
    // to place correctly because it sits at the pole of the equirectangular
    // projection where small pose errors become large angular errors on
    // the surrounding sky. Values were relaxed historically to get any Z0
    // at all; we now prefer fewer but geometrically better Z0 captures.
    private val zenithPitchTolDeg = 1.0f           // was 1.4 — tighter
    private val zenithRollTolDeg = 1.0f            // was 2.2 — much tighter (roll impacts azimuth near pole)

    private val holdMs = 800L
    private val zenithHoldMs = 2500L               // was 1300 — longer settle for gyro to stabilize
    private val zenithSettleMs = 900L              // was 450 — double settle window
    private val zenithAbsPitchDriftTolDeg = 0.5f   // was 0.8 — tighter drift tolerance
    private val zenithAbsRollDriftTolDeg = 0.7f    // was 1.2 — tighter drift tolerance

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

        // Matriz principal — la que el resto del pipeline lee para
        // proyección. Para H0/H45 viene de TYPE_ROTATION_VECTOR. Para Z0
        // puede venir de grav+mag o de TYPE_ROTATION_VECTOR según el flag
        // USE_GRAV_MAG_Z0_MATRIX.
        val rotationM00: Float?,

        val rotationM01: Float?,

        val rotationM02: Float?,

        val rotationM10: Float?,

        val rotationM11: Float?,

        val rotationM12: Float?,

        val rotationM20: Float?,

        val rotationM21: Float?,

        val rotationM22: Float?,

        // Matriz grav+mag — siempre que las validaciones pasen, en TODOS
        // los frames (H0/H45 y Z0). Null si los buffers no estaban listos
        // o las normas/accuracy no cumplieron umbral. Persiste como dato
        // adicional; nadie la lee aguas abajo hasta la edición 4.
        val rotationGravMagM00: Float?,

        val rotationGravMagM01: Float?,

        val rotationGravMagM02: Float?,

        val rotationGravMagM10: Float?,

        val rotationGravMagM11: Float?,

        val rotationGravMagM12: Float?,

        val rotationGravMagM20: Float?,

        val rotationGravMagM21: Float?,

        val rotationGravMagM22: Float?,

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
        gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
        gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        Log.i(
            "Z0MatrixGravMag",
            "init flag=$USE_GRAV_MAG_Z0_MATRIX " +
                    "gravitySensor=${gravitySensor != null} " +
                    "magSensor=${magneticFieldSensor != null}"
        )
        Log.i(
            "Z0GyroTransport",
            "init flag=$USE_GYRO_TRANSPORT_Z0 " +
                    "gyroscopeSensor=${gyroscopeSensor != null}"
        )
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

        gravitySensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }

        gyroscopeSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }

        // Android does not guarantee onAccuracyChanged fires right after
        // register. If the magnetometer was already in UNRELIABLE from a
        // previous app run (cached in sensor stack), we never get a
        // notification and our magneticAccuracy stays at its initial
        // default value. Log the "last known state" on resume so we at
        // least see the current status explicitly.
        Log.i(
            "SunCompassGuard",
            "ON_RESUME currentMagAccuracy=$magneticAccuracy " +
                    "fieldNorm=${"%.1f".format(magneticFieldNormUt)}uT"
        )
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

            // Verbose interpretation for diagnostic. The azimuth reported
            // by TYPE_ROTATION_VECTOR depends on magnetometer calibration.
            // UNRELIABLE/LOW → azimuth can be off by 20-90° from true North.
            // That error is SILENT: there is no runtime failure, just a
            // biased atlas that looks "rotated" between sessions.
            //
            // Recommended user action if LOW/UNRELIABLE: move the phone
            // in a figure-8 pattern in the air for ~5 seconds BEFORE
            // starting the session.
            val accuracyLabel = when (accuracy) {
                SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "HIGH"
                SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "MEDIUM"
                SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "LOW"
                SensorManager.SENSOR_STATUS_UNRELIABLE -> "UNRELIABLE"
                else -> "UNKNOWN($accuracy)"
            }
            val headingReliable = accuracy >= SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM

            Log.d(
                "SunSensors",
                "magAccuracy=$accuracy label=$accuracyLabel " +
                        "headingReliable=$headingReliable " +
                        "fieldNorm=${"%.1f".format(magneticFieldNormUt)}uT"
            )

            if (!headingReliable) {
                Log.w(
                    "SunCompassGuard",
                    "COMPASS_UNRELIABLE: accuracy=$accuracyLabel. Absolute yaw is " +
                            "untrustworthy right now. Atlas built in this state " +
                            "will be rotated relative to other sessions. Move the " +
                            "phone in a figure-8 pattern to recalibrate."
                )
            }
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_MAGNETIC_FIELD -> {
                val x = event.values[0]
                val y = event.values[1]
                val z = event.values[2]
                magneticFieldNormUt = sqrt(x * x + y * y + z * z)

                // Push crudo (x,y,z) al ring buffer para construir la
                // matriz de orientación del Z0 (Fase 5). Promedio de
                // vectores, no de ángulos: lineal y sin singularidades.
                magVecBufferX[magVecBufferIndex] = x
                magVecBufferY[magVecBufferIndex] = y
                magVecBufferZ[magVecBufferIndex] = z
                magVecBufferIndex = (magVecBufferIndex + 1) % Z0_MATRIX_BUFFER_SIZE
                if (magVecBufferIndex == 0) magVecBufferFilled = true
            }

            Sensor.TYPE_ROTATION_VECTOR -> {
                updateLastAbsRotationMatrix(event.values)
                // Push al buffer circular para promedio del anchor.
                // lastAbsRotationMatrix recién quedó actualizado en la
                // línea anterior, así que es la matriz coherente con
                // event.timestamp.
                rotvecBuffer.pushSample(event.timestamp, lastAbsRotationMatrix)
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

                // Push into the zenith ring buffer. This runs at sensor
                // rate (~200 Hz with SENSOR_DELAY_GAME). We read the last
                // N samples when capturing a Z0 to get a time-averaged
                // pose.
                zenithYawBuffer[zenithPoseBufferIndex] = lastAzimuth
                zenithPitchBuffer[zenithPoseBufferIndex] = lastPitch
                zenithRollBuffer[zenithPoseBufferIndex] = lastRoll
                zenithPoseBufferIndex = (zenithPoseBufferIndex + 1) % zenithPoseBufferSize
                if (zenithPoseBufferIndex == 0) zenithPoseBufferFilled = true

                // Keep a rolling "last good yaw" while still well below the
                // Euler singularity. 65° is comfortably away from pitch=90°
                // where the AXIS_X/AXIS_Z remap starts distributing yaw
                // rotation into the roll channel. This value will be used
                // at Z0 capture time in place of the averaged Euler yaw.
                //
                // We record displayAzimuth (derived from GAME_ROTATION_VECTOR,
                // gyro-only, anchored at session start) rather than
                // absoluteYawDeg. The atlas projector uses measuredAzimuthDeg
                // (= displayAzimuth) as the geometric truth for H0/H45
                // frames because it does not suffer mid-session magnetometer
                // recalibrations. The Z0 must live in the same reference
                // frame to stay aligned with the H45 ring below it.
                if (absolutePitchDeg < 65f) {
                    lastStablePreZenithAbsYawDeg = displayAzimuth
                    lastStablePreZenithUpdateMs = SystemClock.elapsedRealtime()
                }

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

            Sensor.TYPE_GRAVITY -> {
                // Vector gravedad pre-filtrado por Android (descontada
                // la aceleración lineal). Norma esperada ≈ 9.81 m/s²
                // cuando el celular está quieto. Push directo al buffer.
                gravityBufferX[gravityBufferIndex] = event.values[0]
                gravityBufferY[gravityBufferIndex] = event.values[1]
                gravityBufferZ[gravityBufferIndex] = event.values[2]
                gravityBufferIndex = (gravityBufferIndex + 1) % Z0_MATRIX_BUFFER_SIZE
                if (gravityBufferIndex == 0) gravityBufferFilled = true

                // Snapshot del último gravity instantáneo. Se usa para
                // refinar el tilt del Z0 luego del transporte gyro.
                instantGravity[0] = event.values[0]
                instantGravity[1] = event.values[1]
                instantGravity[2] = event.values[2]
                hasInstantGravity = true
            }

            Sensor.TYPE_GYROSCOPE -> {
                // Velocidad angular en rad/s, frame del device.
                // Android compensa el bias internamente en TYPE_GYROSCOPE
                // (a diferencia de TYPE_GYROSCOPE_UNCALIBRATED).
                // Empuja al buffer circular para integración posterior.
                gyroTransport.pushSample(
                    event.timestamp,
                    event.values[0],
                    event.values[1],
                    event.values[2]
                )
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

                // FIX TRUE-NORTH (2026-05-15): el anchor capture la declinación
                // requiere que applyDeclination(raw[0]) ya tenga lastLocation
                // disponible. Si la GPS fix llega después del primer rotvec,
                // sin este gate displayNorthOffsetDeg quedaba en mag-N puro
                // y todos los H0/H45 se proyectaban desplazados por declinación.
                if (!displayOffsetInitialized && absoluteYawDeg != 0f && lastLocation != null) {
                    displayNorthOffsetDeg = normalize360(absoluteYawDeg - gameYawDeg)
                    displayOffsetInitialized = true
                    Log.i(
                        "SunDeclination",
                        "ANCHOR_INIT lat=${"%.4f".format(lastLocation!!.latitude)} " +
                                "lon=${"%.4f".format(lastLocation!!.longitude)} " +
                                "absYaw_trueN=${"%.2f".format(absoluteYawDeg)}° " +
                                "gameYaw=${"%.2f".format(gameYawDeg)}° " +
                                "offset_includes_declination=${"%.2f".format(displayNorthOffsetDeg)}°"
                    )
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

        // FIX TRUE-NORTH (2026-05-15): no disparar hasta que el anchor capture
        // la declinación. Sin esto, los frames quedan en mag-N y el ábaco
        // aparece desplazado ~declinación° respecto al sol real.
        if (!displayOffsetInitialized) {
            if (lastLocation == null) {
                Log.d("SunDeclination", "AUTOCAP_BLOCKED waiting for GPS fix")
            }
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
// Target = 90° y tol=1.0° => acepta pitch >= 89°. Con pitch=89°, el cap
// baja a ~54.6° del lado del tel y ~56.6° del lado opuesto (asimetría 2°
// en vez de 4° cuando permitíamos pitch=86°). Con pitch=90° es simétrico.
            val aligned = pitchErr <= 1.0f

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

        // Roll más estricto para H45 que para H0. Ver comentario en
        // h45RollTolDeg. Los Z0 tienen su propio bloque arriba con
        // zenithRollTolDeg, así que acá solo decidimos entre H0 y H45.
        val effectiveRollTolDeg = if (target.pitch >= 30f) {
            h45RollTolDeg
        } else {
            rollTolDeg
        }

        val aligned =
            azErr <= azTolDeg &&
                    pitchErr <= pitchTolDeg &&
                    rollErr <= effectiveRollTolDeg

        if (!aligned) {
            alignmentStartMs = 0L
            resetHoldBuffer()
            return
        }

        if (alignmentStartMs == 0L) {
            alignmentStartMs = now
            resetHoldBuffer()
        }

        // Acumular sample para el stability gate.
        pushHoldSample(
            displayAzimuth, displayPitchDeg, displayRollDeg,
            absoluteYawDeg, absolutePitchDeg, absoluteRollDeg
        )

        // Stability gate: chequear peak-to-peak SOLO con suficientes
        // samples (>= 12 ≈ 200ms). Antes de eso no hay datos
        // estadísticamente significativos.
        if (holdBufferCount >= 12) {
            val (ptpYaw, ptpPitch, ptpRoll) = computeHoldPeakToPeak()
            val maxPtP = maxOf(ptpYaw, ptpPitch, ptpRoll)
            if (maxPtP > holdStabilityPeakToPeakDeg) {
                Log.d(
                    "SunAutoCaptureStability",
                    "REJECT target=${target.azimuth}/${target.pitch} " +
                            "ptpYaw=${"%.2f".format(ptpYaw)} " +
                            "ptpPitch=${"%.2f".format(ptpPitch)} " +
                            "ptpRoll=${"%.2f".format(ptpRoll)} " +
                            "limit=${holdStabilityPeakToPeakDeg} " +
                            "samples=${holdBufferCount}"
                )
                // El celular se movió demasiado durante el hold. Reset
                // del counter: esperar holdMs continuos de pose quieta.
                alignmentStartMs = now
                resetHoldBuffer()
                pushHoldSample(
                    displayAzimuth, displayPitchDeg, displayRollDeg,
                    absoluteYawDeg, absolutePitchDeg, absoluteRollDeg
                )
                return
            }
        }

        val held = (now - alignmentStartMs) >= holdMs
        val cooled = (now - lastShotMs) >= cooldownMs

        if (held && cooled) {
            Log.d(
                "SunAutoCaptureStability",
                "FIRE target=${target.azimuth}/${target.pitch} " +
                        "samples=${holdBufferCount} " +
                        "heldMs=${now - alignmentStartMs}"
            )
            takePhotoForTarget(target, reason = "AUTO")
            alignmentStartMs = 0L
        }
    }

    private fun resetHoldBuffer() {
        holdBufferCount = 0
        holdBufferIndex = 0
    }

    private fun pushHoldSample(
        yaw: Float, pitch: Float, roll: Float,
        absYaw: Float, absPitch: Float, absRoll: Float
    ) {
        holdYawBuffer[holdBufferIndex] = yaw
        holdPitchBuffer[holdBufferIndex] = pitch
        holdRollBuffer[holdBufferIndex] = roll
        holdAbsYawBuffer[holdBufferIndex] = absYaw
        holdAbsPitchBuffer[holdBufferIndex] = absPitch
        holdAbsRollBuffer[holdBufferIndex] = absRoll
        holdBufferIndex = (holdBufferIndex + 1) % holdBufferSize
        if (holdBufferCount < holdBufferSize) holdBufferCount++
    }

    /**
     * Peak-to-peak (max - min) en cada eje sobre los samples acumulados.
     * Para yaw maneja el wrap 0/360 calculando el arco máximo entre
     * pares de samples (O(N²), pero N≤64 → 4096 ops, despreciable).
     */
    private fun computeHoldPeakToPeak(): Triple<Float, Float, Float> {
        if (holdBufferCount == 0) return Triple(0f, 0f, 0f)

        var pitchMin = Float.POSITIVE_INFINITY
        var pitchMax = Float.NEGATIVE_INFINITY
        var rollMin = Float.POSITIVE_INFINITY
        var rollMax = Float.NEGATIVE_INFINITY
        for (i in 0 until holdBufferCount) {
            val p = holdPitchBuffer[i]
            val r = holdRollBuffer[i]
            if (p < pitchMin) pitchMin = p
            if (p > pitchMax) pitchMax = p
            if (r < rollMin) rollMin = r
            if (r > rollMax) rollMax = r
        }

        var yawMaxArc = 0f
        for (i in 0 until holdBufferCount) {
            for (j in i + 1 until holdBufferCount) {
                val d = absAngleDiff(holdYawBuffer[i], holdYawBuffer[j])
                if (d > yawMaxArc) yawMaxArc = d
            }
        }

        return Triple(yawMaxArc, pitchMax - pitchMin, rollMax - rollMin)
    }

    /**
     * Promedio de los samples acumulados en el hold buffer. Devuelve
     * null si hay < 12 samples (consistente con el threshold del
     * stability gate). Yaw circular vía sin/cos para manejar wrap
     * 0/360°. Pitch/roll aritmético.
     *
     * Returns FloatArray de 6 elementos:
     *   [0]=display yaw   [1]=display pitch   [2]=display roll
     *   [3]=abs yaw       [4]=abs pitch       [5]=abs roll
     */
    private fun computeHoldAveragePose(): FloatArray? {
        if (holdBufferCount < 12) return null

        var dispSinSum = 0.0
        var dispCosSum = 0.0
        var dispPitchSum = 0f
        var dispRollSum = 0f
        var absSinSum = 0.0
        var absCosSum = 0.0
        var absPitchSum = 0f
        var absRollSum = 0f

        for (i in 0 until holdBufferCount) {
            val dy = Math.toRadians(holdYawBuffer[i].toDouble())
            dispSinSum += kotlin.math.sin(dy)
            dispCosSum += kotlin.math.cos(dy)
            dispPitchSum += holdPitchBuffer[i]
            dispRollSum += holdRollBuffer[i]

            val ay = Math.toRadians(holdAbsYawBuffer[i].toDouble())
            absSinSum += kotlin.math.sin(ay)
            absCosSum += kotlin.math.cos(ay)
            absPitchSum += holdAbsPitchBuffer[i]
            absRollSum += holdAbsRollBuffer[i]
        }

        val n = holdBufferCount.toFloat()
        val dispYawAvg = normalize360(
            Math.toDegrees(kotlin.math.atan2(dispSinSum, dispCosSum)).toFloat()
        )
        val absYawAvg = normalize360(
            Math.toDegrees(kotlin.math.atan2(absSinSum, absCosSum)).toFloat()
        )

        return floatArrayOf(
            dispYawAvg,
            dispPitchSum / n,
            dispRollSum / n,
            absYawAvg,
            absPitchSum / n,
            absRollSum / n
        )
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
        // Captura los tags SunDeclination/AtlasBuildUseCase/ZenithMatrix
        // a <sessionDir>/logs/sun_log_<ts>.txt. Tiene que arrancar ANTES
        // del próximo Log.i para que capture la línea de SESSION_DECLINATION.
        com.example.sunproject.diagnostics.FileLogger.start(dir)

        // FIX TRUE-NORTH (2026-05-13): declinación magnética computada una
        // sola vez por sesión usando GeomagneticField (modelo WMM, preciso
        // a ~0.5°). Es la única fuente de verdad para llevar mag-N a true-N
        Log.i("PanoramaCAP", "Session dir: ${dir.absolutePath}")
        runOnUiThread { Toast.makeText(this, "Sesión: ${dir.name}", Toast.LENGTH_SHORT).show() }

        val loc = lastLocation

        // Session anchors are no longer set here. They will be set in
        // appendFrameRecord when the first frame of the session is
        // captured, using that frame's absolute yaw (not the pre-capture
        // display value). See the session-relative anchor logic in
        // appendFrameRecord for details.
        //
        // Leaving these as null means SessionRecord will be saved with
        // null anchors initially. The anchors are persisted in JSON when
        // the first frame is saved — see persistSessionAnchors().

        // FIX TRUE-NORTH (2026-05-13): declinación magnética computada una
        // sola vez por sesión usando GeomagneticField (modelo WMM, preciso
        // a ~0.5°). Es la única fuente de verdad para llevar mag-N a true-N.
        // La consumen:
        //   - ZenithMatrixProjector → lleva el Z0 (que vive en mag-N porque
        //     rotationM** viene crudo de getRotationMatrixFromVector) al
        //     mismo true-N en que ya viven los H0/H45 vía el anchor inicial.
        //   - GyroCameraController de la vista 3D → lleva la cámara virtual
        //     de mag-N a true-N, alineándola con el atlas y con el ábaco.
        // Si no hay GPS, queda null → AtlasProjector y GyroCameraController
        // caen a corrección=0 (atlas se ve en mag-N, ábaco desplazado).
        val sessionDeclinationDeg: Float? = computeCurrentDeclinationDeg()
        Log.i(
            "SunDeclination",
            "SESSION_DECLINATION sessionId=${dir.name} " +
                    "declination=${sessionDeclinationDeg?.let { "%.2f".format(it) } ?: "null (sin GPS)"}° " +
                    "lat=${loc?.latitude?.let { "%.4f".format(it) } ?: "null"} " +
                    "lon=${loc?.longitude?.let { "%.4f".format(it) } ?: "null"}"
        )

        val session = SessionRecord(
            sessionId = dir.name,
            startedAtUtcMs = System.currentTimeMillis(),
            latitudeDeg = loc?.latitude,
            longitudeDeg = loc?.longitude,
            altitudeM = loc?.altitude,
            declinationDeg = sessionDeclinationDeg,
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

        // Snapshot of compass state at session start. This is the azimuth
        // reference that the entire atlas will be built against. If this
        // is UNRELIABLE/LOW, the whole session's atlas will be rotated
        // relative to other sessions — even if every frame within this
        // session is internally consistent.
        val magAccLabel = when (magneticAccuracy) {
            SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "HIGH"
            SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "MEDIUM"
            SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "LOW"
            SensorManager.SENSOR_STATUS_UNRELIABLE -> "UNRELIABLE"
            else -> "UNKNOWN($magneticAccuracy)"
        }
        Log.i(
            "SunCompassGuard",
            "SESSION_START sessionId=${dir.name} " +
                    "magAccuracy=$magneticAccuracy label=$magAccLabel " +
                    "fieldNorm=${"%.1f".format(magneticFieldNormUt)}uT " +
                    "initialYawAbs=${"%.2f".format(absoluteYawDeg)}"
        )

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

        // For Z0 captures, use a time-averaged pose from the sensor ring
        // buffer instead of the instantaneous sample. This reduces jitter
        // on the most error-sensitive capture of the session.
        val isZenithCapture = target.pitch >= 80f
        val averagedZenith = if (isZenithCapture) computeAveragedZenithPose() else null
        // Palanca B: para H0/H45, promediar los samples acumulados durante
        // el holdMs en lugar de tomar snapshot instantáneo. Reduce el
        // jitter residual dentro de la ventana garantizada por el
        // stability gate (Palanca A) por √N adicional.
        val averagedHold: FloatArray? = if (!isZenithCapture) computeHoldAveragePose() else null
        val poseYawForFrame = averagedHold?.get(0) ?: displayAzimuth
        val posePitchForFrame = averagedHold?.get(1) ?: displayPitchDeg
        val poseRollForFrame = averagedHold?.get(2) ?: displayRollDeg

        if (averagedHold != null) {
            Log.d(
                "SunAutoCaptureStability",
                "AVERAGED_POSE target=${target.azimuth}/${target.pitch} " +
                        "displayYawAvg=${"%.2f".format(averagedHold[0])} (snap=${"%.2f".format(displayAzimuth)}) " +
                        "displayPitchAvg=${"%.2f".format(averagedHold[1])} (snap=${"%.2f".format(displayPitchDeg)}) " +
                        "displayRollAvg=${"%.2f".format(averagedHold[2])} (snap=${"%.2f".format(displayRollDeg)}) " +
                        "absYawAvg=${"%.2f".format(averagedHold[3])} (snap=${"%.2f".format(absoluteYawDeg)}) " +
                        "absPitchAvg=${"%.2f".format(averagedHold[4])} (snap=${"%.2f".format(absolutePitchDeg)}) " +
                        "absRollAvg=${"%.2f".format(averagedHold[5])} (snap=${"%.2f".format(absoluteRollDeg)}) " +
                        "samples=${holdBufferCount}"
            )
        }
        // For Z0, yaw requires special treatment. The averaged yaw from
        // computeAveragedZenithPose is corrupted by gimbal lock at
        // pitch≈88°: the IMU's Euler decomposition redistributes yaw
        // rotation into the roll channel, so the 20-sample circular
        // mean is physically meaningless (last 4 sessions: 18.6°,
        // 337.7°, 19.3°, 112.3° — no correlation with the actual
        // physical orientation of the phone at shutter). The snapshot
        // taken while pitch<65° is far from the singularity and
        // geometrically valid; we trust it even though it predates the
        // shutter by 1-3s, because the user is tilting upward rather
        // than yaw-rotating during that window.
        val preZenithAgeMs = SystemClock.elapsedRealtime() - lastStablePreZenithUpdateMs
        val snapshotYaw = lastStablePreZenithAbsYawDeg
        val hasUsableSnapshotYaw =
            isZenithCapture &&
                    snapshotYaw != null &&
                    lastStablePreZenithUpdateMs > 0L &&
                    preZenithAgeMs < 30_000L

        val absYawForFrame = when {
            hasUsableSnapshotYaw -> snapshotYaw
            averagedZenith != null -> averagedZenith.first
            averagedHold != null -> averagedHold[3]
            hasLastAbsRotationMatrix -> absoluteYawDeg
            else -> null
        }
        val absPitchForFrame = when {
            averagedZenith != null -> averagedZenith.second
            averagedHold != null -> averagedHold[4]
            hasLastAbsRotationMatrix -> absolutePitchDeg
            else -> null
        }
        val absRollForFrame = when {
            averagedZenith != null -> averagedZenith.third
            averagedHold != null -> averagedHold[5]
            hasLastAbsRotationMatrix -> absoluteRollDeg
            else -> null
        }

        if (isZenithCapture) {
            val yawSource = when {
                hasUsableSnapshotYaw -> "PRE_ZENITH_SNAPSHOT"
                averagedZenith != null -> "AVERAGED_EULER_FALLBACK"
                hasLastAbsRotationMatrix -> "INSTANT_EULER_FALLBACK"
                else -> "NONE"
            }
            Log.i(
                "SunZenithCapture",
                "Z0 capture yawSource=$yawSource " +
                        "yawUsed=${absYawForFrame?.let { "%.2f".format(it) } ?: "null"} " +
                        "snapshotYaw=${snapshotYaw?.let { "%.2f".format(it) } ?: "null"} " +
                        "snapshotAgeMs=$preZenithAgeMs " +
                        "averagedEulerYaw=${averagedZenith?.first?.let { "%.2f".format(it) } ?: "null"} " +
                        "instantEulerYaw=${"%.2f".format(absoluteYawDeg)} " +
                        "absPitch=${absPitchForFrame?.let { "%.2f".format(it) } ?: "null"} " +
                        "absRoll=${absRollForFrame?.let { "%.2f".format(it) } ?: "null"}"
            )
        }

        // ============================================================
        // Cómputo de la matriz grav+mag — sigue activo SOLO como dato
        // adicional persistido (campos rotationGravMag**). Ya NO se usa
        // como matriz principal del Z0 desde Fase 7.
        // ============================================================
        val gravMagMatrix: FloatArray? = computeGravMagZenithMatrix()

        // ============================================================
        // GYRO TRANSPORT (Fase 7) — solo para Z0
        // ============================================================
        // Si hay anchor disponible del último H45, propagamos su orientación
        // por integración del giroscopio hasta el momento de este shutter.
        // Después refinamos el tilt usando gravity instantánea (mantiene el
        // heading propagado, corrige solo pitch/roll si el gyro acumuló
        // drift en la vertical).
        var gyroTransportedMatrix: FloatArray? = null
        var gyroTransportDiag: FloatArray? = null
        if (isZenithCapture && USE_GYRO_TRANSPORT_Z0 && hasGyroAnchor && hasLastAbsRotationMatrix) {
            val nowNs = SystemClock.elapsedRealtimeNanos()
            val deltaTMs = (nowNs - gyroAnchorTimestampNs) / 1_000_000L

            if (deltaTMs in 0..MAX_TRANSPORT_DT_MS) {
                val diag = FloatArray(3)
                val rDelta = gyroTransport.integrateBetween(
                    gyroAnchorTimestampNs,
                    nowNs,
                    diag
                )
                if (rDelta != null) {
                    // R_propagated = R_anchor · R_delta_device
                    val propagated = multiply3x3(gyroAnchorMatrix, rDelta)

                    // Refinar tilt con gravity instantánea: el gyro pudo
                    // haber acumulado drift en pitch/roll durante la propagación,
                    // pero gravity en t_now mide la vertical real con error <1°.
                    val refined = if (hasInstantGravity) {
                        refineTiltWithGravity(propagated, instantGravity)
                    } else {
                        propagated
                    }
                    gyroTransportedMatrix = refined
                    gyroTransportDiag = diag

                    Log.i(
                        "Z0GyroTransport",
                        "TRANSPORT_OK deltaTMs=$deltaTMs " +
                                "integratedAngle=${"%.2f".format(diag[0])}° " +
                                "samples=${diag[1].toInt()} " +
                                "maxGapMs=${"%.1f".format(diag[2])} " +
                                "tiltRefined=${hasInstantGravity}"
                    )
                } else {
                    Log.w(
                        "Z0GyroTransport",
                        "TRANSPORT_FAIL_INTEGRATE deltaTMs=$deltaTMs " +
                                "samples=${gyroTransport.sampleCount()}"
                    )
                }
            } else {
                Log.w(
                    "Z0GyroTransport",
                    "TRANSPORT_FAIL_DT deltaTMs=$deltaTMs > $MAX_TRANSPORT_DT_MS"
                )
            }
        }

        // Matriz "principal" — selección con prioridad:
        //   1. Gyro transport (Fase 7) si Z0 + anchor + integración válida
        //   2. Grav+mag (Fase 5) si Z0 + flag + lectura válida
        //   3. Rotvec instantáneo (comportamiento histórico)
        val useGyroTransport = gyroTransportedMatrix != null
        val useGravMagAsPrimary = !useGyroTransport &&
                isZenithCapture &&
                USE_GRAV_MAG_Z0_MATRIX &&
                gravMagMatrix != null

        val matrixForFrame: FloatArray? = when {
            useGyroTransport -> gyroTransportedMatrix
            useGravMagAsPrimary -> gravMagMatrix
            hasLastAbsRotationMatrix -> lastAbsRotationMatrix
            else -> null
        }

        if (isZenithCapture) {
            val matrixSource = when {
                useGyroTransport -> "GYRO_TRANSPORT"
                useGravMagAsPrimary -> "GRAV_MAG"
                hasLastAbsRotationMatrix -> "ROTATION_VECTOR_FALLBACK"
                else -> "NONE"
            }
            Log.i(
                "Z0GyroTransport",
                "Z0 capture MATRIX_SOURCE=$matrixSource " +
                        "flagGyro=$USE_GYRO_TRANSPORT_Z0 flagGravMag=$USE_GRAV_MAG_Z0_MATRIX"
            )
        }

        // Estos logs
        // alimentan la edición 4 (cómputo del bias).
        if (!isZenithCapture) {
            Log.i(
                "Z0MatrixGravMag",
                "H0/H45 capture target=${target.azimuth}/${target.pitch} " +
                        "gravMagMatrix=${if (gravMagMatrix != null) "OK" else "null"}"
            )
        }

        // ============================================================
        // FROZEN ANCHOR (Refinement 1, Fase 7)
        // ============================================================
        // Congelar la matriz anchor para el gyro transport JUSTO ANTES
        // del shutter, con el mismo timing que frozenPose.
        //
        // Para H0/H45: la matriz se promedia via SVD sobre los últimos
        // ROTVEC_ANCHOR_AVG_WINDOW_MS del rotvec, atenuando el jitter
        // del Kalman interno por √N samples.
        //
        // Para Z0: no se computa anchor (el Z0 NO sirve como anchor del
        // próximo Z0; siempre lo hereda del último H45).
        //
        // onImageSaved (que corre con latencia variable post-shutter)
        // lee este frozenAnchorMatrix en lugar de la matriz viva, así
        // todos los anchors quedan tomados en el mismo instante físico
        // que la foto.
        if (!isZenithCapture && hasLastAbsRotationMatrix) {
            val nowNs = SystemClock.elapsedRealtimeNanos()
            val diag = IntArray(1)
            val avgMatrix = rotvecBuffer.averageOverWindow(
                endTimeNs = nowNs,
                windowMs = ROTVEC_ANCHOR_AVG_WINDOW_MS,
                outDiag = diag
            )
            if (avgMatrix != null) {
                System.arraycopy(avgMatrix, 0, frozenAnchorMatrix, 0, 9)
                frozenAnchorTimestampNs = nowNs
                hasFrozenAnchor = true
                Log.d(
                    "Z0GyroTransport",
                    "ANCHOR_FROZEN avg samples=${diag[0]} window=${ROTVEC_ANCHOR_AVG_WINDOW_MS}ms"
                )
            } else {
                // Fallback: snapshot instantáneo si la ventana quedó vacía
                // (improbable pero defensivo).
                System.arraycopy(lastAbsRotationMatrix, 0, frozenAnchorMatrix, 0, 9)
                frozenAnchorTimestampNs = lastAbsTsNs
                hasFrozenAnchor = true
                Log.w(
                    "Z0GyroTransport",
                    "ANCHOR_FROZEN_FALLBACK no samples in window, using instant snapshot"
                )
            }
        } else {
            hasFrozenAnchor = false
        }

        val frozenPose = CapturedPose(

            azimuthDeg = poseYawForFrame,

            pitchDeg = posePitchForFrame,

            rollDeg = poseRollForFrame,

            absAzimuthDeg = absYawForFrame,
            absPitchDeg = absPitchForFrame,
            absRollDeg = absRollForFrame,

            rotationM00 = matrixForFrame?.get(0),

            rotationM01 = matrixForFrame?.get(1),

            rotationM02 = matrixForFrame?.get(2),

            rotationM10 = matrixForFrame?.get(3),

            rotationM11 = matrixForFrame?.get(4),

            rotationM12 = matrixForFrame?.get(5),

            rotationM20 = matrixForFrame?.get(6),

            rotationM21 = matrixForFrame?.get(7),

            rotationM22 = matrixForFrame?.get(8),

            rotationGravMagM00 = gravMagMatrix?.get(0),

            rotationGravMagM01 = gravMagMatrix?.get(1),

            rotationGravMagM02 = gravMagMatrix?.get(2),

            rotationGravMagM10 = gravMagMatrix?.get(3),

            rotationGravMagM11 = gravMagMatrix?.get(4),

            rotationGravMagM12 = gravMagMatrix?.get(5),

            rotationGravMagM20 = gravMagMatrix?.get(6),

            rotationGravMagM21 = gravMagMatrix?.get(7),

            rotationGravMagM22 = gravMagMatrix?.get(8),

            capturedAtUtcMs = System.currentTimeMillis()
        )

        Log.i(
            "PanoramaCAP",
            "Capturing -> ${file.name} (az=$safeAz p=$safePi) " +
                    "pose=${"%.2f".format(frozenPose.azimuthDeg)}/${"%.2f".format(frozenPose.pitchDeg)}/${"%.2f".format(frozenPose.rollDeg)}"
        )

        // Per-frame compass snapshot. The absoluteYawDeg of this frame is
        // ONLY as trustworthy as the magnetometer accuracy at capture time.
        val magAccLabel = when (magneticAccuracy) {
            SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "HIGH"
            SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "MEDIUM"
            SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "LOW"
            SensorManager.SENSOR_STATUS_UNRELIABLE -> "UNRELIABLE"
            else -> "UNKNOWN($magneticAccuracy)"
        }
        Log.i(
            "SunCompassGuard",
            "CAPTURE frame=${file.nameWithoutExtension} " +
                    "magAccuracy=$magneticAccuracy label=$magAccLabel " +
                    "fieldNorm=${"%.1f".format(magneticFieldNormUt)}uT " +
                    "absYaw=${frozenPose.absAzimuthDeg?.let { "%.2f".format(it) } ?: "null"}"
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

                    // ============================================================
                    // GYRO TRANSPORT ANCHOR (Fase 7 + Refinement 1)
                    // ============================================================
                    // Cada vez que se captura un frame que NO es Z0, copiamos el
                    // FROZEN anchor (computado al inicio de takePhotoForTarget,
                    // antes del cap.takePicture) como anchor para el gyro
                    // transport del próximo Z0.
                    //
                    // Por qué frozen y no live:
                    //   - El callback onImageSaved corre con latencia variable
                    //     post-shutter (50-200ms). Si copiáramos el snapshot
                    //     LIVE acá, el celular probablemente ya empezó a moverse
                    //     hacia el siguiente target, contaminando el anchor con
                    //     la transición.
                    //   - frozenAnchorMatrix se computó en el mismo instante
                    //     físico que la foto, promediando los 200ms previos
                    //     (reducción de jitter √40 ≈ 6×).
                    if (!isZenithCapture && hasFrozenAnchor) {
                        System.arraycopy(frozenAnchorMatrix, 0, gyroAnchorMatrix, 0, 9)
                        gyroAnchorTimestampNs = frozenAnchorTimestampNs
                        hasGyroAnchor = true
                        Log.d(
                            "Z0GyroTransport",
                            "ANCHOR_UPDATED frame=${file.nameWithoutExtension} " +
                                    "tsNs=$gyroAnchorTimestampNs " +
                                    "shotIndex=${capturedFiles.size} " +
                                    "source=FROZEN_AVG"
                        )
                    }

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

        // Sesión 2026-05-13: se probó desactivar OIS/EIS vía Camera2Interop
        // como hipótesis de bottleneck del ghost H0↔H45 en exterior.
        // Resultado: el atlas empeoró (probablemente más motion blur sin
        // OIS durante la exposición) y el ghost no se redujo. OIS no era
        // la causa raíz. Revertido al builder original.
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
        // Loguear qué modos de stabilization soporta este celular. Si OIS
        // está disponible (modes incluye 1), el cambio en el builder de
        // arriba sí tiene efecto. Si modes es null o solo [0], el cambio
        // no aporta (no había OIS para empezar) y hay que buscar otra
        // causa del ghost.
        val oisModes = characteristics.get(
            CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION
        )
        val videoStabModes = characteristics.get(
            CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES
        )
        Log.i(
            "SunOISCheck",
            "OIS modes available: ${oisModes?.joinToString() ?: "null (no OIS support)"} " +
                    "Video stab modes: ${videoStabModes?.joinToString() ?: "null"}"
        )
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

            // PROBE Camera2 calibration metadata (Fase 1: solo diagnóstico).
            // Si el OEM expone LENS_DISTORTION y LENS_INTRINSIC_CALIBRATION, se
            // loggean los k1..k3, p1, p2, fx, fy, cx, cy reales y un veredicto
            // sobre la magnitud de la distorsión radial. NO se integran todavía
            // al pipeline — eso es Fase 2, condicional a este resultado.
            val probe = CameraIntrinsicsProbe.probe(
                currentCameraId ?: "unknown",
                characteristics
            )
            val verdictMsg = when (probe.verdict) {
                CameraIntrinsicsProbe.Verdict.NOT_AVAILABLE ->
                    "Cámara: no expone calibración Camera2"
                CameraIntrinsicsProbe.Verdict.LOW ->
                    "Cámara: distorsión leve <0.5°"
                CameraIntrinsicsProbe.Verdict.MODERATE ->
                    "Cámara: distorsión moderada ~${"%.1f".format(probe.distortionAt100PctEdgeDeg)}°"
                CameraIntrinsicsProbe.Verdict.HIGH ->
                    "Cámara: distorsión alta ~${"%.1f".format(probe.distortionAt100PctEdgeDeg)}°"
            }
            Toast.makeText(this, verdictMsg, Toast.LENGTH_LONG).show()
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

            rotationGravMagM00 = pose.rotationGravMagM00,

            rotationGravMagM01 = pose.rotationGravMagM01,

            rotationGravMagM02 = pose.rotationGravMagM02,

            rotationGravMagM10 = pose.rotationGravMagM10,

            rotationGravMagM11 = pose.rotationGravMagM11,

            rotationGravMagM12 = pose.rotationGravMagM12,

            rotationGravMagM20 = pose.rotationGravMagM20,

            rotationGravMagM21 = pose.rotationGravMagM21,

            rotationGravMagM22 = pose.rotationGravMagM22,

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

        // On the first frame of the session, anchor the session's azimuth
        // reference to this frame's absolute yaw. This makes the atlas
        // self-consistent regardless of compass calibration: subsequent
        // frames are projected as (absYaw - anchorYaw) + target.azimuth
        // of this first frame, so the "North" of the atlas is always
        // where the first frame was pointing.
        //
        // Why this works: within a single session the gyroscope keeps
        // relative azimuth differences stable (sub-degree) even when
        // magneticAccuracy is UNRELIABLE. Only the absolute reference to
        // world-North is unreliable, and that's exactly what we opt out
        // of by using a per-session anchor.
        if (sessionYawAnchorDeg == null && pose.absAzimuthDeg != null) {
            sessionYawAnchorDeg = pose.absAzimuthDeg
            sessionPitchAnchorDeg = pose.absPitchDeg
            sessionRollAnchorDeg = pose.absRollDeg

            Log.i(
                "SunSessionAnchor",
                "ANCHOR_SET sessionId=${currentSessionId} " +
                        "firstFrame=${frame.frameId} " +
                        "anchorYaw=${"%.2f".format(sessionYawAnchorDeg!!)} " +
                        "anchorPitch=${"%.2f".format(sessionPitchAnchorDeg ?: Float.NaN)} " +
                        "anchorRoll=${"%.2f".format(sessionRollAnchorDeg ?: Float.NaN)} " +
                        "firstTargetAz=${target.azimuth} firstTargetPitch=${target.pitch}"
            )

            // Re-persist session.json with the new anchors.
            persistSessionAnchors()
        }

        sessionStore.appendFrame(frame, paths)
    }

    private fun persistSessionAnchors() {
        val dir = sessionDir ?: return
        val paths = sessionPaths ?: return
        val loc = lastLocation

        // Re-computamos declinación. Si el fix GPS llegó después de
        // ensureSessionDir, ahora sí queda persistida. Si ya estaba bien,
        // este recálculo confirma el valor (estable a escala de segundos).
        val sessionDeclinationDeg: Float? = computeCurrentDeclinationDeg()

        val session = SessionRecord(
            sessionId = currentSessionId ?: dir.name,
            startedAtUtcMs = System.currentTimeMillis(),
            latitudeDeg = loc?.latitude,
            longitudeDeg = loc?.longitude,
            altitudeM = loc?.altitude,
            declinationDeg = sessionDeclinationDeg,
            cameraId = currentCameraId ?: "widest_back_camera",
            sensorMode = "rotation_vector",
            sessionYawAnchorDeg = sessionYawAnchorDeg,
            sessionPitchAnchorDeg = sessionPitchAnchorDeg,
            sessionRollAnchorDeg = sessionRollAnchorDeg,
            notes = null
        )
        sessionStore.saveSession(session, paths)
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

    /**
     * Returns a time-averaged absolute pose computed from the last N
     * sensor samples stored in the zenith buffer. Uses circular mean for
     * yaw (handles 359°→0° wrap correctly). Only used for Z0 captures,
     * where the pose needs to be especially stable.
     *
     * Returns null if the buffer is empty or pitch is not zenith-like
     * (safety fallback so this is never applied to H0/H45).
     */
    private fun computeAveragedZenithPose(): Triple<Float, Float, Float>? {
        val count = if (zenithPoseBufferFilled) zenithPoseBufferSize else zenithPoseBufferIndex
        if (count < 5) return null  // not enough samples yet

        // Circular mean for yaw
        var sumSin = 0.0
        var sumCos = 0.0
        var sumPitch = 0.0
        var sumRoll = 0.0

        for (i in 0 until count) {
            val yawRad = Math.toRadians(zenithYawBuffer[i].toDouble())
            sumSin += kotlin.math.sin(yawRad)
            sumCos += kotlin.math.cos(yawRad)
            sumPitch += zenithPitchBuffer[i]
            sumRoll += zenithRollBuffer[i]
        }

        val avgYaw = normalize360(
            Math.toDegrees(kotlin.math.atan2(sumSin, sumCos)).toFloat()
        )
        val avgPitch = (sumPitch / count).toFloat()
        val avgRoll = (sumRoll / count).toFloat()

        Log.d(
            "SunZenithAvg",
            "samplesUsed=$count " +
                    "avgYaw=${"%.2f".format(avgYaw)} " +
                    "avgPitch=${"%.2f".format(avgPitch)} " +
                    "avgRoll=${"%.2f".format(avgRoll)} " +
                    "currentYaw=${"%.2f".format(absoluteYawDeg)}"
        )

        return Triple(avgYaw, avgPitch, avgRoll)
    }

    /**
     * Construye la matriz de rotación R_world←device del Z0 a partir
     * de los buffers de gravedad y campo magnético, sin pasar por
     * TYPE_ROTATION_VECTOR.
     *
     * Pipeline:
     *   1. Promediar los componentes (x,y,z) de gravedad y mag sobre
     *      los últimos N samples (linear, no circular — son vectores).
     *   2. Validar magneticAccuracy ≥ MEDIUM.
     *   3. Validar norma de gravedad ≈ 9.81 (celular quieto).
     *   4. Validar norma magnética en rango razonable (20-80 µT).
     *   5. Llamar SensorManager.getRotationMatrix(R, null, grav, mag).
     *      Esa API construye R via TRIAD/Wahba sobre los dos vectores
     *      de referencia, sin filtros ni Euler. No tiene gimbal lock
     *      cerca del cenit.
     *
     * Devuelve los 9 floats row-major de R, o null si la lectura falla
     * cualquier validación. En ese caso el caller debe hacer fallback
     * a la matriz de TYPE_ROTATION_VECTOR (lastAbsRotationMatrix).
     */
    private fun computeGravMagZenithMatrix(): FloatArray? {
        val gravCount =
            if (gravityBufferFilled) Z0_MATRIX_BUFFER_SIZE else gravityBufferIndex
        val magCount =
            if (magVecBufferFilled) Z0_MATRIX_BUFFER_SIZE else magVecBufferIndex

        if (gravCount < Z0_MATRIX_MIN_SAMPLES || magCount < Z0_MATRIX_MIN_SAMPLES) {
            Log.w(
                "Z0MatrixGravMag",
                "REJECT not enough samples gravCount=$gravCount magCount=$magCount " +
                        "(min=$Z0_MATRIX_MIN_SAMPLES)"
            )
            return null
        }

        // Promedio lineal de cada componente. Es correcto porque son
        // vectores físicos, no ángulos — no hay wrap a 360°.
        var sumGx = 0.0; var sumGy = 0.0; var sumGz = 0.0
        for (i in 0 until gravCount) {
            sumGx += gravityBufferX[i]
            sumGy += gravityBufferY[i]
            sumGz += gravityBufferZ[i]
        }
        val gx = (sumGx / gravCount).toFloat()
        val gy = (sumGy / gravCount).toFloat()
        val gz = (sumGz / gravCount).toFloat()

        var sumMx = 0.0; var sumMy = 0.0; var sumMz = 0.0
        for (i in 0 until magCount) {
            sumMx += magVecBufferX[i]
            sumMy += magVecBufferY[i]
            sumMz += magVecBufferZ[i]
        }
        val mx = (sumMx / magCount).toFloat()
        val my = (sumMy / magCount).toFloat()
        val mz = (sumMz / magCount).toFloat()

        val gravNorm = sqrt(gx * gx + gy * gy + gz * gz)
        val magNorm = sqrt(mx * mx + my * my + mz * mz)

        // Validación 1 (RELAJADA — Fase 6): aceptamos cualquier
        // magneticAccuracy, incluido UNRELIABLE (0).
        //
        // Razonamiento: para la corrección de bias rotvec ↔ grav+mag
        // no necesitamos true-N preciso. Lo único que importa es la
        // CONSISTENCIA entre rotvec y grav+mag. Como TYPE_ROTATION_VECTOR
        // también usa el magnetómetro internamente, ambas lecturas tienen
        // el mismo sesgo cuando el sensor está unreliable, y la diferencia
        // (que es lo que define R_bias) lo cancela.
        //
        // El ruido residual lo controla la validación de spread en
        // computeRotvecToGravMagBias: si las muestras individuales son
        // muy dispersas, el bias se rechaza globalmente. Acá solo nos
        // aseguramos de tener DATOS para que esa validación opere.
        //
        // Para el caso del Z0 con flag activo (Fase 5): el bias correction
        // de Fase 6 lo va a llevar al frame correcto aunque el grav+mag
        // crudo del Z0 esté sesgado. Si Fase 6 falla, el fallback a
        // ROTATION_VECTOR queda igual que antes.
        //
        // Logueamos el valor para diagnóstico, pero no rechazamos.
        val magAccLabel = when (magneticAccuracy) {
            SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "HIGH"
            SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "MEDIUM"
            SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "LOW"
            SensorManager.SENSOR_STATUS_UNRELIABLE -> "UNRELIABLE"
            else -> "UNKNOWN($magneticAccuracy)"
        }

        // Validación 2: norma de gravedad. Si el celular está quieto,
        // |g| ≈ 9.81 m/s². Una desviación >0.5 m/s² indica movimiento
        // de mano o aceleración lineal residual — TYPE_GRAVITY no
        // descuenta del todo el shake.
        if (kotlin.math.abs(gravNorm - 9.81f) > 0.5f) {
            Log.w(
                "Z0MatrixGravMag",
                "REJECT gravNorm anomalous: ${"%.3f".format(gravNorm)} (expected ≈9.81)"
            )
            return null
        }

        // Validación 3 (RELAJADA — Fase 6): rechazo solo de valores
        // físicamente absurdos (>200 µT indica imán cerca del celular,
        // <5 µT indica falla del sensor). El rango "normal" del campo
        // terrestre 25-65 µT lo dejamos pasar y también valores bajos
        // por interferencia local — son inevitables en muchos sitios
        // reales y la validación de spread los va a filtrar después si
        // son inconsistentes.
        if (magNorm < 5f || magNorm > 200f) {
            Log.w(
                "Z0MatrixGravMag",
                "REJECT magNorm physically absurd: ${"%.1f".format(magNorm)}uT"
            )
            return null
        }

        // Llamada canónica: TRIAD / Wahba 2-vectores via API legacy de
        // Android. Devuelve R_world←device en frame ENU magnético.
        val R = FloatArray(9)
        val gravArr = floatArrayOf(gx, gy, gz)
        val magArr = floatArrayOf(mx, my, mz)
        val ok = SensorManager.getRotationMatrix(R, null, gravArr, magArr)

        if (!ok) {
            // Falla solo si gravity y mag son colineales (escenario
            // físicamente imposible salvo bug del sensor).
            Log.w(
                "Z0MatrixGravMag",
                "REJECT getRotationMatrix returned false " +
                        "grav=(${"%.2f".format(gx)},${"%.2f".format(gy)},${"%.2f".format(gz)}) " +
                        "mag=(${"%.2f".format(mx)},${"%.2f".format(my)},${"%.2f".format(mz)})"
            )
            return null
        }

        Log.i(
            "Z0MatrixGravMag",
            "ACCEPT gravCount=$gravCount magCount=$magCount " +
                    "gravNorm=${"%.3f".format(gravNorm)} " +
                    "magNorm=${"%.1f".format(magNorm)}uT " +
                    "magAccuracy=$magneticAccuracy ($magAccLabel) " +
                    "R=[${"%.3f".format(R[0])},${"%.3f".format(R[1])},${"%.3f".format(R[2])}; " +
                    "${"%.3f".format(R[3])},${"%.3f".format(R[4])},${"%.3f".format(R[5])}; " +
                    "${"%.3f".format(R[6])},${"%.3f".format(R[7])},${"%.3f".format(R[8])}]"
        )

        return R
    }

    /**
     * Calcula la declinación magnética para la ubicación actual usando
     * GeomagneticField (modelo WMM de Android). Devuelve null si no hay
     * GPS fix todavía. La declinación es la diferencia angular entre
     * norte magnético y norte verdadero — positiva al este, negativa
     * al oeste. En BA es ~−5°, en Cauchari ~−1°.
     */
    private fun computeCurrentDeclinationDeg(): Float? {
        val loc = lastLocation ?: return null
        val gmf = GeomagneticField(
            loc.latitude.toFloat(),
            loc.longitude.toFloat(),
            loc.altitude.toFloat(),
            System.currentTimeMillis()
        )
        return gmf.declination
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

    /**
     * Multiplicación de matrices 3x3 row-major.
     * Devuelve C = A · B en un nuevo FloatArray(9).
     */
    private fun multiply3x3(a: FloatArray, b: FloatArray): FloatArray {
        val c = FloatArray(9)
        c[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6]
        c[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7]
        c[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8]
        c[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6]
        c[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7]
        c[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8]
        c[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6]
        c[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7]
        c[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8]
        return c
    }

    /**
     * Refina el tilt de R (matriz device→world) usando gravity instantánea.
     *
     * Idea: la tercera columna de R (cuando R es device→world row-major,
     * R = [right | up | forward] en convención típica) representa cuál es
     * la dirección "device-Z" expresada en world. La gravity instantánea
     * representa la dirección "world-down" expresada en device.
     *
     * Para nuestro convenio (consistente con SensorManager.getRotationMatrix
     * row-major, que produce R tal que R · v_device = v_world):
     *   - La fila 3 de R (índices 6,7,8) es lo que device-Z mapea a world.
     *   - g_device normalizado debería mapear a (0, 0, -1)_world bajo R^T.
     *
     * Aplicamos una rotación R_correction tal que:
     *   - R_correction · up_propagated = up_measured
     *   - donde up_propagated = -R^T[2,:] = (-R[2], -R[5], -R[8]) (en world)
     *   - up_measured = -g_device normalizado, llevado a world via R
     *
     * En la práctica esto se reduce a: rotar R alrededor de un eje horizontal
     * para alinear su "up estimado" con el "up medido" por gravity. La rotación
     * preserva el heading (yaw) porque su eje vive en el plano horizontal.
     *
     * Si el desalineamiento es < 0.1° devuelve R sin tocar (no refinamiento
     * innecesario). Si > 5° devuelve R sin tocar (algo está mal, mejor no
     * empeorarlo — caería al log de diagnóstico).
     */
    private fun refineTiltWithGravity(R: FloatArray, gDevice: FloatArray): FloatArray {
        // Normalizar gravity en device frame.
        val gNorm = sqrt(gDevice[0]*gDevice[0] + gDevice[1]*gDevice[1] + gDevice[2]*gDevice[2])
        if (gNorm < 1e-3f) return R
        val gx = gDevice[0] / gNorm
        val gy = gDevice[1] / gNorm
        val gz = gDevice[2] / gNorm

        // up_measured en world = R · g_device_normalized.
        // TYPE_GRAVITY de Android apunta hacia ARRIBA en device frame (es la
        // dirección "Up" que siente el celular en reposo, no la aceleración
        // gravitacional hacia abajo). Por lo tanto NO hay que invertir el signo:
        // R · g_device = up_world directamente.
        val upMx = R[0]*gx + R[1]*gy + R[2]*gz
        val upMy = R[3]*gx + R[4]*gy + R[5]*gz
        val upMz = R[6]*gx + R[7]*gy + R[8]*gz

        // up_propagated en world: la dirección Up del world según R actual = (0, 0, 1).
        // Si R fuera "perfecta", upMx,upMy,upMz ya serían (0,0,1). El desalineamiento
        // es la rotación más corta entre upM y (0,0,1).
        val dot = upMz.coerceIn(-1f, 1f)
        val angleRad = kotlin.math.acos(dot)
        val angleDeg = Math.toDegrees(angleRad.toDouble()).toFloat()

        if (angleDeg < 0.1f) return R   // ya está bien alineado
        if (angleDeg > 5f) {
            Log.w(
                "Z0GyroTransport",
                "tilt refinement skipped: misalignment=${"%.2f".format(angleDeg)}° > 5°"
            )
            return R
        }

        // Eje de rotación = upM × (0,0,1), normalizado.
        val axX = upMy
        val axY = -upMx
        val axZ = 0f
        val axNorm = sqrt(axX*axX + axY*axY + axZ*axZ)
        if (axNorm < 1e-6f) return R
        val ax = axX / axNorm
        val ay = axY / axNorm
        val az = axZ / axNorm

        // Construir R_correction (Rodrigues) que rota upM hacia (0,0,1).
        // angleRad es el ángulo entre upM y +Z; el eje (axX,axY,axZ) viene de
        // upM × (0,0,1), que es la dirección correcta para rotar upM HACIA +Z
        // con ángulo positivo. No hace falta negar.
        val theta = angleRad
        val c = kotlin.math.cos(theta)
        val s = kotlin.math.sin(theta)
        val omc = 1f - c
        val rc = floatArrayOf(
            c + ax*ax*omc,      ax*ay*omc - az*s,   ax*az*omc + ay*s,
            ay*ax*omc + az*s,   c + ay*ay*omc,      ay*az*omc - ax*s,
            az*ax*omc - ay*s,   az*ay*omc + ax*s,   c + az*az*omc
        )
        val refined = multiply3x3(rc, R)

        Log.d(
            "Z0GyroTransport",
            "tilt refined: misalignment=${"%.2f".format(angleDeg)}° corrected"
        )
        return refined
    }

    companion object {
        private const val REQUEST_CODE_PERMISSIONS = 10
        private val REQUIRED_PERMISSIONS =
            arrayOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION)

        // ============================================================
        // Z0 MATRIX SOURCE FLAG (Fase 5)
        // ============================================================
        // Si está en true, la matriz 3x3 R_world←device del Z0 se construye
        // vía SensorManager.getRotationMatrix(R, null, gravity, magnetic) en
        // el momento del disparo, en vez de copiarla de TYPE_ROTATION_VECTOR.
        //
        // Por qué: TYPE_ROTATION_VECTOR es composite (accel+mag+gyro vía
        // Kalman). Entre la última H45 y el Z0, el yaw integrado por gyro
        // acumula deriva. La matriz construida desde gravity+mag es
        // self-contained, no depende de frames anteriores ni del estado
        // del filtro, y cerca del cenit no degenera porque NO calcula yaw
        // por atan2 sobre Euler — usa los dos vectores de referencia
        // (down + heading horizontal) para construir la base ortonormal
        // directa (TRIAD / Wahba 2-vectores).
        //
        // Si la lectura falla validación (magAccuracy bajo, norma de
        // gravedad anómala, etc.) hay fallback automático a la matriz vieja.
        //
        // Mantener TRUE: este es el fallback de segundo nivel cuando gyro
        // transport (Fase 7) no está disponible. Solo se ejecuta si
        // USE_GYRO_TRANSPORT_Z0 = false o si no hay anchor válido.
        private const val USE_GRAV_MAG_Z0_MATRIX = true

        private const val Z0_MATRIX_BUFFER_SIZE = 30
        private const val Z0_MATRIX_MIN_SAMPLES = 15

        // ============================================================
        // Z0 GYRO TRANSPORT FLAG (Fase 7)
        // ============================================================
        // Si está en true, la matriz del Z0 se construye propagando con
        // el giroscopio desde el último H45 capturado, en lugar de leer
        // rotvec/gravmag instantáneo en el shutter del Z0.
        //
        // Por qué: el cluster H0/H45 vive en frame "gyro-anchored" (ver
        // displayAzimuth y comentario en TYPE_ROTATION_VECTOR handler).
        // Lecturas instantáneas del magnetómetro en el momento del Z0
        // pueden tener jitter de 4°+ frame-a-frame en sitios magnéticos
        // sucios. Transportar por gyro mantiene al Z0 en el MISMO frame
        // del cluster, con drift acotado por la duración del transporte.
        //
        // Drift esperado: 0.05-0.2°/s post-bias-compensation de Android.
        // Para Δt típico de 5-10s entre img_20 y Z0, drift acumulado
        // < 1° en condiciones normales.
        //
        // Si no hay anchor válido (no hubo H45 previo, gaps en buffer del
        // gyro, Δt > MAX_TRANSPORT_DT_MS), fallback automático al método
        // anterior (gravmag o rotvec según USE_GRAV_MAG_Z0_MATRIX).
        private const val USE_GYRO_TRANSPORT_Z0 = true

        // Δt máximo entre anchor (último H45) y Z0 para considerar la
        // integración confiable. Si excede, el drift acumulado del gyro
        // es inaceptable y se hace fallback.
        private const val MAX_TRANSPORT_DT_MS = 30_000L

        // Capacidad del buffer del giroscopio. A 200 Hz con SENSOR_DELAY_GAME
        // cubre ~20s, suficiente para cubrir img_20 → Z0 con holgura.
        private const val GYRO_BUFFER_CAPACITY = 4096

        // Gap máximo entre samples consecutivos del gyro durante el
        // intervalo de integración. Un gap > 200ms suele indicar que
        // Android pausó el sensor y la integración ya no es confiable.
        private const val GYRO_MAX_GAP_NS = 200_000_000L

        // ============================================================
        // ANCHOR ROTVEC AVERAGING (Refinement 1, Fase 7)
        // ============================================================
        // El anchor del último H45 se promediaba como snapshot instantáneo
        // del rotvec. Un sample único tiene jitter del Kalman interno de
        // ±0.5-1°. Promediando los últimos N ms en una ventana antes del
        // shutter, el ruido alta-frecuencia se atenúa por √N samples.
        //
        // Ventana de 200ms a SENSOR_DELAY_GAME (~200 Hz) ≈ 40 samples.
        // Reducción de ruido √40 ≈ 6×. La parte de baja frecuencia
        // (sesgo del magnetómetro durante todo el ring) NO se atenúa
        // con esto — ese es el techo físico.
        private const val ROTVEC_ANCHOR_AVG_WINDOW_MS = 400L

        // Capacidad del buffer del rotvec. 256 samples a SENSOR_DELAY_GAME
        // cubre ~1.3s, holgura amplia para ventanas de 200ms.
        private const val ROTVEC_BUFFER_CAPACITY = 256
    }

}

@OptIn(ExperimentalCamera2Interop::class)
fun CameraInfo.getCameraCharacteristics(): CameraCharacteristics {
    return Camera2CameraInfo.extractCameraCharacteristics(this)
}