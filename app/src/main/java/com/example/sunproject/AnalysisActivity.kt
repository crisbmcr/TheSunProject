package com.example.sunproject

import android.Manifest
import android.annotation.SuppressLint
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ImageView
import android.widget.ProgressBar
import android.widget.Toast
import android.widget.ToggleButton
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.domain.atlas.AtlasConfig
import com.example.sunproject.domain.atlas.SolarPathOverlay
import com.example.sunproject.domain.horizon.HorizonProfile
import com.example.sunproject.domain.horizon.HorizonProfileDetector
import com.example.sunproject.domain.horizon.HorizonProfileOverlay2D
import com.example.sunproject.domain.horizon.HorizonProfileStore
import com.example.sunproject.domain.horizon.SkySegmenterTFLite
import com.example.sunproject.domain.solar.SolarChart
import com.example.sunproject.domain.solar.SolarPathGenerator
import com.google.android.gms.location.LocationServices
import java.io.File
import java.time.Instant
import java.time.ZoneId
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

class AnalysisActivity : AppCompatActivity() {

    private lateinit var panoramaImageView: ImageView
    private lateinit var btnSolarChart: ToggleButton
    private lateinit var btnHorizonProfile: ToggleButton
    private lateinit var btnRegenerateProfile: Button
    private lateinit var btnOpen3d: Button
    private lateinit var loadingIndicator: ProgressBar

    private var baseBitmap: Bitmap? = null
    private var cachedChart: SolarChart? = null
    private var cachedProfile: HorizonProfile? = null
    private var atlasPath: String? = null

    // Flags para exportar PNGs derivados (atlas+ábaco, atlas+perfil) una sola
    // vez por sesión de análisis. Igual que el comportamiento previo del ábaco.
    private var chartPngExported = false
    private var profilePngExported = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_analysis)

        panoramaImageView = findViewById(R.id.panoramaImageView)
        btnSolarChart = findViewById(R.id.btnSolarChart)
        btnHorizonProfile = findViewById(R.id.btnHorizonProfile)
        btnRegenerateProfile = findViewById(R.id.btnRegenerateProfile)
        btnOpen3d = findViewById(R.id.btnOpen3d)
        loadingIndicator = findViewById(R.id.loadingIndicator)

        atlasPath = intent.getStringExtra("panorama_path")
        if (atlasPath == null) {
            Toast.makeText(this, "Sin panorama para mostrar", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        baseBitmap = BitmapFactory.decodeFile(atlasPath)
        if (baseBitmap == null) {
            Toast.makeText(this, "No se pudo abrir el panorama", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        // Render inicial: sólo atlas crudo.
        panoramaImageView.setImageURI(Uri.fromFile(File(atlasPath!!)))

        btnSolarChart.setOnCheckedChangeListener { _, _ -> onToggleChanged() }
        btnHorizonProfile.setOnCheckedChangeListener { _, _ -> onToggleChanged() }
        btnRegenerateProfile.setOnClickListener { regenerateProfile() }

        btnOpen3d.setOnClickListener {
            val path = atlasPath
            if (path.isNullOrBlank() || !File(path).exists()) {
                Toast.makeText(this, "No hay atlas para visualizar en 3D", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            val intent = Intent(this, PanoramaViewActivity::class.java).apply {
                putExtra(PanoramaViewActivity.EXTRA_PANORAMA_PATH, path)
            }
            startActivity(intent)
        }

    }

    /**
     * Disparado al cambiar cualquiera de los dos toggles. Asegura los recursos
     * necesarios (chart y/o profile) en background y después renderiza el
     * estado actual sobre el bitmap.
     */
    private fun onToggleChanged() {
        ensureResources { renderCurrentState() }
    }

    /**
     * Carga (o recupera del cache) los recursos requeridos por el estado actual
     * de los toggles y después llama onReady en main thread. Si falla algo
     * (chart sin GPS, atlas dañado), apaga el toggle correspondiente.
     */
    private fun ensureResources(onReady: () -> Unit) {
        val needChart = btnSolarChart.isChecked && cachedChart == null
        val needProfile = btnHorizonProfile.isChecked && cachedProfile == null

        if (!needChart && !needProfile) {
            onReady()
            return
        }

        showLoading(true)

        Thread {
            try {
                if (needChart) {
                    val chart = resolveChartSync()
                    if (chart == null) {
                        runOnUiThread {
                            showLoading(false)
                            btnSolarChart.isChecked = false
                            Toast.makeText(
                                this,
                                "No hay ubicación conocida. Activá el GPS al menos una vez.",
                                Toast.LENGTH_LONG
                            ).show()
                        }
                        return@Thread
                    }
                    cachedChart = chart
                }

                if (needProfile) {
                    val profile = resolveProfileSync(forceRegenerate = false)
                    if (profile == null) {
                        runOnUiThread {
                            showLoading(false)
                            btnHorizonProfile.isChecked = false
                            Toast.makeText(
                                this,
                                "No se pudo generar el perfil de obstrucción.",
                                Toast.LENGTH_LONG
                            ).show()
                        }
                        return@Thread
                    }
                    cachedProfile = profile
                }

                runOnUiThread {
                    showLoading(false)
                    onReady()
                }
            } catch (t: Throwable) {
                Log.e("AnalysisActivity", "ensureResources failed", t)
                runOnUiThread {
                    showLoading(false)
                    Toast.makeText(this, "Error: ${t.message}", Toast.LENGTH_LONG).show()
                }
            }
        }.start()
    }

    /**
     * Renderiza el bitmap actual según los toggles. Se re-compone desde
     * baseBitmap cada vez para soportar todas las combinaciones de overlays.
     *
     * Corre en background incluso cuando los recursos están cacheados, porque
     * drawChart/drawProfile sobre bitmap 3600×900 puede tomar 100-300ms y
     * bloquea la UI si se hace en main.
     */
    private fun renderCurrentState() {
        val base = baseBitmap ?: return
        val showChart = btnSolarChart.isChecked
        val showProfile = btnHorizonProfile.isChecked

        if (!showChart && !showProfile) {
            panoramaImageView.setImageBitmap(base)
            return
        }

        showLoading(true)

        Thread {
            try {
                val cfg = AtlasConfig(widthPx = base.width, heightPx = base.height)
                val composite = base.copy(Bitmap.Config.ARGB_8888, true)

                if (showChart) {
                    cachedChart?.let { chart ->
                        SolarPathOverlay.drawChart(composite, cfg, chart)
                    }
                }
                if (showProfile) {
                    cachedProfile?.let { profile ->
                        HorizonProfileOverlay2D.drawProfile(composite, cfg, profile)
                    }
                }

                runOnUiThread {
                    showLoading(false)
                    panoramaImageView.setImageBitmap(composite)
                }

                // Side-effect: exportar PNGs derivados la primera vez en la
                // sesión que se prende cada toggle. Independientes entre sí
                // (uno con sólo ábaco, otro con sólo perfil) — la composición
                // de los dos no se exporta para no multiplicar archivos.
                maybeExportPngs(showChart, showProfile, cfg)
            } catch (t: Throwable) {
                Log.e("AnalysisActivity", "renderCurrentState failed", t)
                runOnUiThread {
                    showLoading(false)
                    Toast.makeText(this, "Error renderizando: ${t.message}", Toast.LENGTH_LONG).show()
                }
            }
        }.start()
    }

    /** Exporta atlas+ábaco y atlas+perfil como PNGs separados, sólo una vez. */
    private fun maybeExportPngs(showChart: Boolean, showProfile: Boolean, cfg: AtlasConfig) {
        val atlasFileStr = atlasPath ?: return
        val atlasFile = File(atlasFileStr)

        if (showChart && !chartPngExported) {
            cachedChart?.let { chart ->
                val outName = atlasFile.nameWithoutExtension + "_abaco." + atlasFile.extension
                val outFile = File(atlasFile.parentFile, outName)
                if (!outFile.exists()) {
                    val written = SolarPathOverlay.exportWithChart(atlasFile, cfg, chart)
                    if (written != null) {
                        runOnUiThread {
                            Toast.makeText(this, "Ábaco guardado: ${written.name}", Toast.LENGTH_SHORT).show()
                        }
                    }
                }
                chartPngExported = true
            }
        }

        if (showProfile && !profilePngExported) {
            cachedProfile?.let { profile ->
                val outName = atlasFile.nameWithoutExtension + "_horizon." + atlasFile.extension
                val outFile = File(atlasFile.parentFile, outName)
                if (!outFile.exists()) {
                    val written = HorizonProfileOverlay2D.exportWithProfile(atlasFile, cfg, profile)
                    if (written != null) {
                        runOnUiThread {
                            Toast.makeText(this, "Perfil guardado: ${written.name}", Toast.LENGTH_SHORT).show()
                        }
                    }
                }
                profilePngExported = true
            }
        }
    }

    /**
     * Forzar regeneración del perfil: borra el JSON del disco, recomputa desde
     * el atlas y vuelve a renderizar. Prende el toggle del perfil si estaba
     * apagado (asumimos que el usuario quiere verlo después de regenerar).
     */
    private fun regenerateProfile() {
        showLoading(true)

        Thread {
            try {
                val profile = resolveProfileSync(forceRegenerate = true)
                runOnUiThread {
                    showLoading(false)
                    if (profile == null) {
                        Toast.makeText(this, "No se pudo regenerar el perfil.", Toast.LENGTH_LONG).show()
                        return@runOnUiThread
                    }
                    cachedProfile = profile
                    profilePngExported = false  // permitir re-exportar el PNG

                    // Prender el toggle sin disparar el listener (lo hacemos
                    // ya nosotros llamando renderCurrentState explícitamente).
                    if (!btnHorizonProfile.isChecked) {
                        btnHorizonProfile.setOnCheckedChangeListener(null)
                        btnHorizonProfile.isChecked = true
                        btnHorizonProfile.setOnCheckedChangeListener { _, _ -> onToggleChanged() }
                    }
                    renderCurrentState()
                    Toast.makeText(this, "Perfil regenerado.", Toast.LENGTH_SHORT).show()
                }
            } catch (t: Throwable) {
                Log.e("AnalysisActivity", "regenerateProfile failed", t)
                runOnUiThread {
                    showLoading(false)
                    Toast.makeText(this, "Error: ${t.message}", Toast.LENGTH_LONG).show()
                }
            }
        }.start()
    }

    /**
     * Resuelve el HorizonProfile en background:
     *   1. Si forceRegenerate=false: intentar cargar de disco.
     *   2. Si no existe o forceRegenerate=true: computar con detector y persistir.
     */
    private fun resolveProfileSync(forceRegenerate: Boolean): HorizonProfile? {
        val atlasFileStr = atlasPath ?: return null
        val atlasFile = File(atlasFileStr)
        val atlasDir = atlasFile.parentFile ?: return null

        if (!forceRegenerate) {
            HorizonProfileStore.load(atlasDir)?.let { return it }
        } else {
            HorizonProfileStore.delete(atlasDir)
        }

        val base = baseBitmap ?: return null
        val cfg = AtlasConfig(widthPx = base.width, heightPx = base.height)

        // sessionId para trazabilidad. Fallback al nombre del directorio padre
        // del atlas si no se puede leer session.json.
        val sessionId = readSessionInfo(atlasFileStr)?.sessionId
            ?: atlasDir.parentFile?.name
            ?: "unknown"

        val result = HorizonProfileDetector.detect(
            context = applicationContext,
            atlasBitmap = base,
            atlasConfig = cfg,
            sessionId = sessionId,
            sourceAtlasName = atlasFile.name,
            generateDebugInfo = false
        )

        HorizonProfileStore.save(result.profile, atlasDir)
        return result.profile
    }

    /**
     * Resuelve el SolarChart en background. Antes era resolveChartAsync con
     * callback; ahora es bloqueante (llamado desde Thread{}), usando un latch
     * para esperar el callback de FusedLocation cuando hay que ir al GPS.
     */
    private fun resolveChartSync(): SolarChart? {
        val path = atlasPath ?: return null
        val sessionInfo = readSessionInfo(path)

        // Path 1: la sesión tiene lat/lon persistida → cálculo directo.
        if (sessionInfo != null && sessionInfo.lat != null && sessionInfo.lon != null) {
            val year = Instant.ofEpochMilli(sessionInfo.startedAtUtcMs)
                .atZone(ZoneId.systemDefault()).year
            return SolarPathGenerator.generate(
                latitudeDeg = sessionInfo.lat,
                longitudeDeg = sessionInfo.lon,
                year = year,
                captureEpochMillisUtc = sessionInfo.startedAtUtcMs
            )
        }

        // Path 2: fallback a última ubicación conocida del teléfono.
        val granted = ContextCompat.checkSelfPermission(
            this, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        if (!granted) return null

        val latch = CountDownLatch(1)
        val holder = arrayOfNulls<SolarChart>(1)

        @SuppressLint("MissingPermission")
        val client = LocationServices.getFusedLocationProviderClient(this)
        client.lastLocation
            .addOnSuccessListener { loc ->
                if (loc != null) {
                    val year = sessionInfo?.startedAtUtcMs
                        ?.let { Instant.ofEpochMilli(it).atZone(ZoneId.systemDefault()).year }
                        ?: Instant.now().atZone(ZoneId.systemDefault()).year
                    holder[0] = SolarPathGenerator.generate(
                        latitudeDeg = loc.latitude,
                        longitudeDeg = loc.longitude,
                        year = year,
                        captureEpochMillisUtc = sessionInfo?.startedAtUtcMs
                    )
                }
                latch.countDown()
            }
            .addOnFailureListener { latch.countDown() }

        // 5s es generoso para lastLocation cached. Si no hay GPS conocido,
        // el callback igual dispara con loc=null y el latch se libera.
        latch.await(5, TimeUnit.SECONDS)
        return holder[0]
    }

    private fun showLoading(show: Boolean) {
        loadingIndicator.visibility = if (show) View.VISIBLE else View.GONE
    }

    /**
     * Lee la sesión asociada al panorama desde <sessionDir>/session.json.
     * Convención: panorama vive en <sessionDir>/atlas/<archivo>.png.
     */
    private fun readSessionInfo(panoramaPath: String): SessionInfo? {
        return try {
            val panoramaFile = File(panoramaPath)
            val sessionDir = panoramaFile.parentFile?.parentFile ?: return null
            val paths = JsonSessionStore().createSessionPaths(sessionDir)
            val session = JsonSessionStore().loadSession(paths) ?: return null
            SessionInfo(
                sessionId = session.sessionId,
                lat = session.latitudeDeg,
                lon = session.longitudeDeg,
                startedAtUtcMs = session.startedAtUtcMs
            )
        } catch (t: Throwable) {
            Log.w("AnalysisActivity", "No pude leer session.json", t)
            null
        }
    }

    private data class SessionInfo(
        val sessionId: String,
        val lat: Double?,
        val lon: Double?,
        val startedAtUtcMs: Long
    )
}