package com.example.sunproject

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.hardware.SensorManager
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.domain.atlas.AtlasProjector
import com.example.sunproject.domain.render3d.PanoramaRenderer
import com.example.sunproject.domain.render3d.abacus.SolarChartMesh3D
import com.example.sunproject.domain.render3d.camera.GyroCameraController
import com.example.sunproject.domain.solar.SolarPathGenerator
import com.google.android.gms.location.LocationServices
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withContext
import java.io.File
import java.time.Instant
import java.time.ZoneId
import kotlin.coroutines.resume

private const val TAG = "PanoramaViewActivity"

class PanoramaViewActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_PANORAMA_PATH = "panorama_path"
    }

    private lateinit var glView: GLSurfaceView
    private lateinit var hudText: TextView
    private var renderer: PanoramaRenderer? = null
    private var gyroController: GyroCameraController? = null

    // Estado del HUD — actualizado desde múltiples coroutines (bitmap y ábaco).
    // renderHud() reconstruye el texto completo desde estas variables. Acceso
    // sólo desde el main thread, no necesita sincronización.
    private var hudBitmapState: String = "cargando..."
    private var hudChartState: String = "cargando..."
    private var hudYawCorrectionDeg: Float = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_panorama_view)

        glView = findViewById(R.id.glSurfaceView)
        hudText = findViewById(R.id.hudText)

        val panoramaPath = intent.getStringExtra(EXTRA_PANORAMA_PATH)
        if (panoramaPath.isNullOrBlank()) {
            Log.e(TAG, "Falta $EXTRA_PANORAMA_PATH en el Intent — cerrando.")
            Toast.makeText(this, "Ruta de atlas vacía", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        // === Configuración del renderer — debe hacerse ANTES que GLSurfaceView se adjunte
        // a la ventana, o sea, sincronicamente en onCreate. Cualquier cosa async (decodificar
        // bitmap, leer session.json, etc.) se hace después y se inyecta al renderer vía setters.
        glView.setEGLContextClientVersion(2)

        // FIX TRUE-NORTH (2026-05-13): leemos session.json asociado al atlas
        // para obtener declinación magnética + lat/lon/timestamp. Convención:
        // panorama vive en <sessionDir>/atlas/atlas_projected_all.png, igual
        // que AnalysisActivity. GyroCameraController usa la declinación para
        // llevar el rotvec sensor (mag-N) a true-N, alineando la cámara
        // virtual con el atlas. lat/lon/timestamp se usan para generar el
        // ábaco solar en 3D.
        val sessionInfo = loadSessionInfo(panoramaPath)
        val yawCorrectionDeg: Float = sessionInfo?.declinationDeg ?: 0f
        AtlasProjector.setSessionDeclinationDeg(sessionInfo?.declinationDeg)
        Log.i(TAG, "Corrección yaw (true-N): ${"%.2f".format(yawCorrectionDeg)}°")

        hudYawCorrectionDeg = yawCorrectionDeg

        val sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        val newRenderer = PanoramaRenderer()
        val newController = GyroCameraController(sensorManager, yawCorrectionDeg)

        // Conectar renderer ↔ controller. Referencia del array se mantiene; el controller
        // muta el contenido cuando llegan eventos del sensor.
        newRenderer.setExternalViewMatrix(newController.currentViewMatrix())

        renderer = newRenderer
        gyroController = newController

        glView.setRenderer(newRenderer)
        glView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        // HUD inicial mientras cargan bitmap y ábaco en paralelo.
        renderHud()

        // Arrancar el sensor ya, aunque el bitmap todavía no esté listo — así la
        // cámara virtual ya está orientada cuando aparezcan los píxeles.
        newController.start()

        // === Carga del bitmap en background. Cuando termine, se inyecta al renderer y
        // en el siguiente frame se hace el GL upload.
        lifecycleScope.launch {
            val bitmap = withContext(Dispatchers.IO) {
                runCatching {
                    BitmapFactory.decodeFile(panoramaPath)
                }.getOrNull()
            }

            if (bitmap == null) {
                Log.e(TAG, "No se pudo decodificar el atlas en $panoramaPath")
                hudBitmapState = "error"
                renderHud()
                Toast.makeText(this@PanoramaViewActivity,
                    "No se pudo abrir $panoramaPath", Toast.LENGTH_LONG).show()
                return@launch
            }

            Log.i(TAG, "Atlas cargado ${bitmap.width}x${bitmap.height} de ${File(panoramaPath).name}")

            newRenderer.setAtlasBitmap(bitmap)

            hudBitmapState = "${bitmap.width}x${bitmap.height}"
            renderHud()
        }

        // === Carga del ábaco solar 3D en background, en paralelo al bitmap. Si la
        // sesión no tiene lat/lon, cae al fallback de LocationServices (mismo
        // patrón que AnalysisActivity). Si todo falla, el ábaco queda en "no
        // disponible" y el panorama 3D sigue siendo usable.
        lifecycleScope.launch {
            val mesh = withContext(Dispatchers.Default) {
                buildSolarChartMesh(sessionInfo)
            }

            if (mesh == null) {
                Log.w(TAG, "No se pudo generar el ábaco 3D (session sin lat/lon y/o sin location fallback)")
                hudChartState = "no disponible"
                renderHud()
                return@launch
            }

            Log.i(TAG, "Ábaco 3D generado — daily=${mesh.dailyPaths.size} " +
                    "hourly=${mesh.hourlyPaths.size} capture=${mesh.captureDayPath != null} " +
                    "sun=${mesh.sunMarker != null}")
            newRenderer.setSolarChartMesh(mesh)

            hudChartState = "cargado"
            renderHud()
        }
    }

    override fun onResume() {
        super.onResume()
        glView.onResume()
        gyroController?.start()
    }

    override fun onPause() {
        super.onPause()
        gyroController?.stop()
        glView.onPause()
    }

    // ============================================================
    // HUD
    // ============================================================

    /**
     * Reconstruye el texto del HUD desde el estado actual. Llamar desde
     * el main thread cada vez que se actualiza una de las variables hud*.
     */
    private fun renderHud() {
        val correctionStr = if (hudYawCorrectionDeg != 0f) {
            "yaw fix: %.2f°".format(hudYawCorrectionDeg)
        } else {
            "gyro-N puro (sin fix)"
        }
        hudText.text = "Panorama 3D — $hudBitmapState\n" +
                "modo: giroscopio  |  $correctionStr\n" +
                "ábaco: $hudChartState"
    }

    // ============================================================
    // SESSION + ÁBACO SOLAR 3D
    // ============================================================

    /**
     * Lee el session.json asociado al atlas en una sola pasada. Devuelve null
     * si no se puede resolver (panorama mal ubicado, JSON corrupto, etc.).
     * Sync porque es un JSON chico — la lectura toma <5ms.
     */
    private fun loadSessionInfo(panoramaPath: String): SessionInfo? = runCatching {
        val panoramaFile = File(panoramaPath)
        val sessionDir = panoramaFile.parentFile?.parentFile ?: return@runCatching null
        val store = JsonSessionStore()
        val paths = store.createSessionPaths(sessionDir)
        val session = store.loadSession(paths) ?: return@runCatching null
        SessionInfo(
            declinationDeg = session.declinationDeg,
            latitudeDeg = session.latitudeDeg,
            longitudeDeg = session.longitudeDeg,
            startedAtUtcMs = session.startedAtUtcMs
        )
    }.getOrElse {
        Log.w(TAG, "No pude leer session.json", it)
        null
    }

    /**
     * Resuelve lat/lon (de la sesión o del fallback), genera el SolarChart con
     * NOAA y lo convierte a mesh GL. Devuelve null si no hay forma de obtener
     * lat/lon. Pensado para correr en Dispatchers.Default.
     */
    private suspend fun buildSolarChartMesh(sessionInfo: SessionInfo?): SolarChartMesh3D? {
        if (sessionInfo == null) return null

        val lat: Double
        val lon: Double
        if (sessionInfo.latitudeDeg != null && sessionInfo.longitudeDeg != null) {
            lat = sessionInfo.latitudeDeg
            lon = sessionInfo.longitudeDeg
        } else {
            val resolved = getLastKnownLocation() ?: return null
            lat = resolved.first
            lon = resolved.second
            Log.i(TAG, "Lat/lon resueltos por LocationServices fallback: $lat, $lon")
        }

        val year = Instant.ofEpochMilli(sessionInfo.startedAtUtcMs)
            .atZone(ZoneId.systemDefault()).year

        val chart = SolarPathGenerator.generate(
            latitudeDeg = lat,
            longitudeDeg = lon,
            year = year,
            captureEpochMillisUtc = sessionInfo.startedAtUtcMs
        )

        return SolarChartMesh3D.build(chart)
    }

    /**
     * Wraps LocationServices.lastLocation como suspend fun. Devuelve null si
     * no hay permiso, no hay last-known location, o el Task falla. No solicita
     * permisos — esa responsabilidad es del flow upstream.
     */
    @SuppressLint("MissingPermission")
    private suspend fun getLastKnownLocation(): Pair<Double, Double>? {
        val granted = ContextCompat.checkSelfPermission(
            this, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        if (!granted) {
            Log.w(TAG, "Sin permiso ACCESS_FINE_LOCATION — no se puede resolver lat/lon")
            return null
        }

        return suspendCancellableCoroutine { cont ->
            LocationServices.getFusedLocationProviderClient(this)
                .lastLocation
                .addOnSuccessListener { loc ->
                    if (cont.isActive) {
                        cont.resume(loc?.let { Pair(it.latitude, it.longitude) })
                    }
                }
                .addOnFailureListener {
                    Log.w(TAG, "FusedLocation lastLocation falló", it)
                    if (cont.isActive) cont.resume(null)
                }
        }
    }

    private data class SessionInfo(
        val declinationDeg: Float?,
        val latitudeDeg: Double?,
        val longitudeDeg: Double?,
        val startedAtUtcMs: Long
    )
}