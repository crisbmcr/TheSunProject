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
    private var smoothedAngles = FloatArray(3)
    private val alpha = 0.15f
    private var isFirstReading = true

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
    private val azTolDeg = 6f
    private val pitchTolDeg = 3f
    private val rollTolDeg = 3f
    private val holdMs = 650L
    private val cooldownMs = 1200L
    private var alignmentStartMs = 0L
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

    private val displayFilter = AngleFilterState()
    private val captureFilter = AngleFilterState()

    private var lastSensorTsNs: Long = 0L

    private var displayAzimuth = 0f
    private var displayPitchDeg = 0f
    private var displayRollDeg = 0f

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

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return

        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        SensorManager.remapCoordinateSystem(
            rotationMatrix,
            SensorManager.AXIS_X,
            SensorManager.AXIS_Z,
            remappedRotationMatrix
        )

        val rawAzimuth = (
                Math.toDegrees(
                    SensorManager.getOrientation(remappedRotationMatrix, orientationAngles)[0].toDouble()
                ).toFloat() + 360f
                ) % 360f

        val rawPitch = Math.toDegrees(asin(remappedRotationMatrix[7].toDouble())).toFloat()
        val rawRoll = Math.toDegrees(-atan2(remappedRotationMatrix[6], remappedRotationMatrix[8]).toDouble()).toFloat()

        if (isFirstReading) {
            smoothedAngles[0] = rawAzimuth
            smoothedAngles[1] = rawPitch
            smoothedAngles[2] = rawRoll
            isFirstReading = false
        } else {
            var deltaAz = rawAzimuth - smoothedAngles[0]
            if (deltaAz > 180) deltaAz -= 360f
            if (deltaAz < -180) deltaAz += 360f
            smoothedAngles[0] = (smoothedAngles[0] + alpha * deltaAz + 360f) % 360f
            smoothedAngles[1] += alpha * (rawPitch - smoothedAngles[1])
            smoothedAngles[2] += alpha * (rawRoll - smoothedAngles[2])
        }

        var azimuth = smoothedAngles[0]
        val pitch = smoothedAngles[1]
        val roll = smoothedAngles[2]

        lastLocation?.let {
            val geomagneticField = GeomagneticField(
                it.latitude.toFloat(),
                it.longitude.toFloat(),
                it.altitude.toFloat(),
                System.currentTimeMillis()
            )
            azimuth = (azimuth + geomagneticField.declination + 360f) % 360f
        }

        lastAzimuth = azimuth
        lastPitch = pitch
        lastRoll = roll

        guideView.updateOrientation(azimuth, pitch, roll)
        debugAzimuth.text = "Azimuth: ${azimuth.toInt()}°"
        debugPitch.text = "Pitch: ${pitch.toInt()}°"
        debugRoll.text = "Roll: ${roll.toInt()}°"

        checkAutoCapture()
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

        val dAz = absAngleDiff(lastAzimuth, target.azimuth)
        val dPitch = abs(lastPitch - target.pitch)
        val dRoll = abs(lastRoll)   // para el anillo horizontal queremos roll ~ 0°

        val aligned = (
                dAz <= azTolDeg &&
                        dPitch <= pitchTolDeg &&
                        dRoll <= rollTolDeg
                )

        val now = SystemClock.elapsedRealtime()

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

        if (sessionYawAnchorDeg == null) sessionYawAnchorDeg = lastAzimuth
        if (sessionPitchAnchorDeg == null) sessionPitchAnchorDeg = lastPitch
        if (sessionRollAnchorDeg == null) sessionRollAnchorDeg = lastRoll

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


    private fun appendMetadata(fileName: String) {
        val dir = ensureSessionDir()
        val meta = File(dir, "metadata.csv")
        val loc = lastLocation
        val lat = loc?.latitude ?: Double.NaN
        val lon = loc?.longitude ?: Double.NaN
        val alt = loc?.altitude ?: Double.NaN
        val line = "${fileName},${System.currentTimeMillis()},${lastAzimuth},${lastPitch},${lastRoll},${lat},${lon},${alt}\n"
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

        Log.i("PanoramaCAP", "Capturing -> ${file.name} (az=$safeAz p=$safePi)")

        val outputOptions = ImageCapture.OutputFileOptions.Builder(file).build()
        lastShotMs = SystemClock.elapsedRealtime()

        cap.takePicture(
            outputOptions,
            cameraExecutor,
            object : ImageCapture.OnImageSavedCallback {
                override fun onImageSaved(outputFileResults: ImageCapture.OutputFileResults) {
                    capturedFiles.add(file)
                    appendMetadata(file.name)
                    appendFrameRecord(file, target, capturedFiles.size)

                    target.isCaptured = true
                    guideView.postInvalidate()

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

    private fun appendFrameRecord(file: File, target: GuideView.CapturePoint, shotIndex: Int) {
        val paths = sessionPaths ?: return
        val loc = lastLocation
        val (imgW, imgH) = readImageSize(file)

        val frame = FrameRecord(
            frameId = file.nameWithoutExtension,
            sessionId = currentSessionId ?: "unknown_session",
            ringId = ringIdForTarget(target),
            shotIndex = shotIndex,
            originalPath = file.absolutePath,
            capturedAtUtcMs = System.currentTimeMillis(),
            targetAzimuthDeg = target.azimuth,
            targetPitchDeg = target.pitch,
            measuredAzimuthDeg = lastAzimuth,
            measuredPitchDeg = lastPitch,
            measuredRollDeg = lastRoll,
            latitudeDeg = loc?.latitude,
            longitudeDeg = loc?.longitude,
            altitudeM = loc?.altitude,
            imageWidthPx = imgW,
            imageHeightPx = imgH,
            hfovDeg = currentHfovDeg,
            vfovDeg = currentVfovDeg
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
                runOnUiThread {
                    Toast.makeText(
                        this,
                        "Error proyectando frame: ${t.message}",
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }
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