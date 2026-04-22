package com.example.sunproject

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.widget.ImageView
import android.widget.Toast
import android.widget.ToggleButton
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.domain.atlas.AtlasConfig
import com.example.sunproject.domain.atlas.SolarPathOverlay
import com.example.sunproject.domain.solar.SolarChart
import com.example.sunproject.domain.solar.SolarPathGenerator
import com.google.android.gms.location.LocationServices
import java.io.File
import java.time.Instant
import java.time.ZoneId

class AnalysisActivity : AppCompatActivity() {

    private lateinit var panoramaImageView: ImageView
    private lateinit var btnSolarChart: ToggleButton

    private var baseBitmap: Bitmap? = null
    private var overlayBitmap: Bitmap? = null
    private var cachedChart: SolarChart? = null
    private var atlasPath: String? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_analysis)

        panoramaImageView = findViewById(R.id.panoramaImageView)
        btnSolarChart = findViewById(R.id.btnSolarChart)

        atlasPath = intent.getStringExtra("panorama_path")
        if (atlasPath == null) {
            Toast.makeText(this, "Sin panorama para mostrar", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        // Render inicial: sólo atlas.
        baseBitmap = BitmapFactory.decodeFile(atlasPath)
        if (baseBitmap == null) {
            Toast.makeText(this, "No se pudo abrir el panorama", Toast.LENGTH_LONG).show()
            finish()
            return
        }
        panoramaImageView.setImageURI(Uri.fromFile(File(atlasPath!!)))

        btnSolarChart.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) renderWithOverlay() else renderBase()
        }
    }

    private fun renderBase() {
        panoramaImageView.setImageBitmap(baseBitmap)
    }

    private fun renderWithOverlay() {
        cachedChart?.let { chart ->
            applyOverlayAsync(chart)
            return
        }
        resolveChartAsync { chart ->
            if (chart == null) {
                runOnUiThread {
                    btnSolarChart.isChecked = false
                    Toast.makeText(
                        this,
                        "No hay ubicación conocida. Activá el GPS al menos una vez.",
                        Toast.LENGTH_LONG
                    ).show()
                }
                return@resolveChartAsync
            }
            cachedChart = chart
            applyOverlayAsync(chart)
        }
    }

    private fun applyOverlayAsync(chart: SolarChart) {
        Thread {
            try {
                val base = baseBitmap ?: return@Thread
                // Config por defecto del atlas. Debe coincidir con AtlasBuildUseCase.
                val cfg = AtlasConfig(widthPx = base.width, heightPx = base.height)
                val mutable = base.copy(Bitmap.Config.ARGB_8888, true)
                SolarPathOverlay.drawChart(mutable, cfg, chart)
                overlayBitmap = mutable
                runOnUiThread { panoramaImageView.setImageBitmap(mutable) }
            } catch (t: Throwable) {
                Log.e("SolarOverlay", "render falló", t)
            }
        }.start()
    }

    private fun resolveChartAsync(onDone: (SolarChart?) -> Unit) {
        val sessionInfo = readSessionInfo(atlasPath!!)

        // Path 1: la sesión tiene lat/lon persistida.
        if (sessionInfo != null && sessionInfo.lat != null && sessionInfo.lon != null) {
            val year = Instant.ofEpochMilli(sessionInfo.startedAtUtcMs)
                .atZone(ZoneId.systemDefault()).year
            onDone(
                SolarPathGenerator.generate(
                    latitudeDeg = sessionInfo.lat,
                    longitudeDeg = sessionInfo.lon,
                    year = year,
                    captureEpochMillisUtc = sessionInfo.startedAtUtcMs
                )
            )
            return
        }

        // Path 2: fallback a última ubicación conocida del teléfono.
        val granted = ContextCompat.checkSelfPermission(
            this, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        if (!granted) {
            onDone(null)
            return
        }

        @SuppressLint("MissingPermission")
        LocationServices.getFusedLocationProviderClient(this)
            .lastLocation
            .addOnSuccessListener { loc ->
                if (loc == null) {
                    onDone(null)
                    return@addOnSuccessListener
                }
                val year = sessionInfo?.startedAtUtcMs
                    ?.let { Instant.ofEpochMilli(it).atZone(ZoneId.systemDefault()).year }
                    ?: ZoneId.systemDefault().let {
                        Instant.now().atZone(it).year
                    }
                onDone(
                    SolarPathGenerator.generate(
                        latitudeDeg = loc.latitude,
                        longitudeDeg = loc.longitude,
                        year = year,
                        captureEpochMillisUtc = sessionInfo?.startedAtUtcMs
                    )
                )
            }
            .addOnFailureListener { onDone(null) }
    }

    /**
     * Resuelve el session.json asociado al panorama.
     * Convención actual: el panorama vive en <sessionDir>/atlas/atlas_projected_all.png.
     */
    private fun readSessionInfo(panoramaPath: String): SessionInfo? {
        return try {
            val panoramaFile = File(panoramaPath)
            val sessionDir = panoramaFile.parentFile?.parentFile ?: return null
            val paths = JsonSessionStore().createSessionPaths(sessionDir)
            val session = JsonSessionStore().loadSession(paths) ?: return null
            SessionInfo(
                lat = session.latitudeDeg,
                lon = session.longitudeDeg,
                startedAtUtcMs = session.startedAtUtcMs
            )
        } catch (t: Throwable) {
            Log.w("SolarOverlay", "No pude leer session.json", t)
            null
        }
    }

    private data class SessionInfo(
        val lat: Double?,
        val lon: Double?,
        val startedAtUtcMs: Long
    )
}