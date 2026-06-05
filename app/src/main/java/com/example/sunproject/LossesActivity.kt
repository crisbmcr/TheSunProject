package com.example.sunproject

import android.graphics.Color
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.domain.horizon.HorizonProfile
import com.example.sunproject.domain.horizon.HorizonProfileStore
import com.example.sunproject.domain.loss.CalculationMode
import com.example.sunproject.domain.loss.DiffuseModel
import com.example.sunproject.domain.loss.PanelOrientation
import com.example.sunproject.domain.loss.ShadingLossCalculator
import com.example.sunproject.domain.loss.ShadingLossConfig
import com.example.sunproject.domain.loss.ShadingLossResult
import com.example.sunproject.domain.loss.irradiance.IrradianceTimeSeries
import com.example.sunproject.domain.loss.irradiance.PvgisTmyDataSource
import com.github.mikephil.charting.charts.BarChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.BarData
import com.github.mikephil.charting.data.BarDataSet
import com.github.mikephil.charting.data.BarEntry
import com.github.mikephil.charting.formatter.IndexAxisValueFormatter
import com.github.mikephil.charting.formatter.ValueFormatter
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File
import kotlin.math.round

/**
 * Pantalla de resultados del cálculo de pérdidas por sombreado.
 *
 * ## Modos
 *
 * El usuario puede alternar entre dos modos vía RadioGroup:
 *
 *  - **GEOMÉTRICO** (default): integración pura cos(θ_inc) sobre el año.
 *    Reporta % pérdida geométrica anual. No requiere internet.
 *
 *  - **ENERGÉTICO**: integración POA real con datos PVGIS-ERA5 TMY del
 *    sitio. Reporta % pérdida energética + kWh/m²/año perdidos. La primera
 *    vez requiere conexión para descargar el TMY (~1.4 MB); después se
 *    sirve del cache local indefinidamente.
 *
 * ## Flujo de ejecución
 *
 *  1. onCreate carga perfil + sesión y dispara cálculo en modo Geométrico.
 *  2. Si el usuario cambia el toggle a Energético:
 *     a. Si hay cache: cargar TMY del disco, ejecutar motor con ENERGY_FULL.
 *     b. Si no hay cache: descargar TMY de PVGIS, parsear, ejecutar motor.
 *     c. Si la descarga falla: diálogo con opciones (reintentar / volver a
 *        geométrico / cancelar).
 *  3. Resultados se cachean por modo para evitar recálculos al alternar.
 *
 * ## Threading
 *
 * Usa `lifecycleScope` para coroutines (no `Thread`). I/O del fetcher en
 * `Dispatchers.IO`, motor matemático en `Dispatchers.Default`. UI en main.
 * Cualquier job pendiente se cancela al destruir la activity.
 *
 * ## Inputs por intent extras
 *  - EXTRA_SESSION_DIR: ruta absoluta del directorio de la sesión.
 *    De ahí se deriva el perfil (sessionDir/atlas/horizon_profile.json) y
 *    los metadatos (sessionDir/session.json).
 */
class LossesActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_SESSION_DIR = "session_dir"
        private const val TAG = "LossesActivity"

        private val MONTH_LABELS = arrayOf(
            "Ene", "Feb", "Mar", "Abr", "May", "Jun",
            "Jul", "Ago", "Sep", "Oct", "Nov", "Dic"
        )
    }

    // -- Views ---------------------------------------------------------

    private lateinit var loadingIndicator: ProgressBar
    private lateinit var loadingText: TextView
    private lateinit var contentContainer: ScrollView

    private lateinit var lblSite: TextView
    private lateinit var lblPanel: TextView

    private lateinit var modeRadioGroup: RadioGroup
    private lateinit var rbModeGeometric: RadioButton
    private lateinit var rbModeEnergy: RadioButton

    private lateinit var lblAnnualLossTitle: TextView
    private lateinit var lblAnnualLoss: TextView
    private lateinit var lblAnnualLossSub: TextView

    private lateinit var energyCard: LinearLayout
    private lateinit var lblEnergyLost: TextView
    private lateinit var lblPoaPotential: TextView
    private lateinit var lblPoaReal: TextView

    private lateinit var monthlyChart: BarChart
    private lateinit var heatmapView: HeatmapView

    private lateinit var lblDatasetFooter: TextView
    private lateinit var lblMethodologyNote: TextView
    private lateinit var btnBack: Button

    // -- Estado --------------------------------------------------------

    /** Dirección de la sesión recibida por intent. */
    private lateinit var sessionDir: File

    /** Cargados una vez en el setup inicial. */
    private var profile: HorizonProfile? = null
    private var lat: Double = Double.NaN
    private var lon: Double = Double.NaN
    private var panel: PanelOrientation = PanelOrientation.optimalFor(0.0)
    private var timezoneOffset: Double = 0.0

    /** Cache de resultados por modo para evitar recalcular al alternar. */
    private var cachedGeometric: ShadingLossResult? = null
    private var cachedEnergy: ShadingLossResult? = null
    private var cachedMetadata: PvgisTmyDataSource.Metadata? = null

    /** Fuente PVGIS (se construye una vez al inicializar). */
    private lateinit var tmyDataSource: PvgisTmyDataSource

    /** Job actual del cálculo. Se cancela cuando el usuario cambia de modo. */
    private var currentJob: Job? = null

    // -- Lifecycle -----------------------------------------------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_losses)

        bindViews()
        tmyDataSource = PvgisTmyDataSource(applicationContext)

        val sessionDirStr = intent.getStringExtra(EXTRA_SESSION_DIR)
        if (sessionDirStr.isNullOrBlank()) {
            Toast.makeText(this, "No se recibió el directorio de la sesión", Toast.LENGTH_LONG).show()
            finish()
            return
        }
        val dir = File(sessionDirStr)
        if (!dir.exists() || !dir.isDirectory) {
            Toast.makeText(this, "Directorio de sesión inválido", Toast.LENGTH_LONG).show()
            finish()
            return
        }
        sessionDir = dir

        // Estado inicial: loading visible, contenido oculto.
        showLoading("Cargando perfil del horizonte...")

        // Setup inicial: cargar perfil + sesión + primer cálculo geométrico.
        lifecycleScope.launch {
            try {
                val ok = withContext(Dispatchers.IO) { loadSetupBlocking() }
                if (!ok) return@launch  // showError ya fue invocado
                runCalculationForMode(CalculationMode.GEOMETRIC_ONLY)
            } catch (t: Throwable) {
                Log.e(TAG, "Setup falló", t)
                showError("Error inesperado: ${t.message ?: t.javaClass.simpleName}")
            }
        }
    }

    private fun bindViews() {
        loadingIndicator = findViewById(R.id.lossesLoadingIndicator)
        loadingText = findViewById(R.id.lossesLoadingText)
        contentContainer = findViewById(R.id.lossesContentContainer)

        lblSite = findViewById(R.id.lblSite)
        lblPanel = findViewById(R.id.lblPanel)

        modeRadioGroup = findViewById(R.id.modeRadioGroup)
        rbModeGeometric = findViewById(R.id.rbModeGeometric)
        rbModeEnergy = findViewById(R.id.rbModeEnergy)

        lblAnnualLossTitle = findViewById(R.id.lblAnnualLossTitle)
        lblAnnualLoss = findViewById(R.id.lblAnnualLoss)
        lblAnnualLossSub = findViewById(R.id.lblAnnualLossSub)

        energyCard = findViewById(R.id.energyCard)
        lblEnergyLost = findViewById(R.id.lblEnergyLost)
        lblPoaPotential = findViewById(R.id.lblPoaPotential)
        lblPoaReal = findViewById(R.id.lblPoaReal)

        monthlyChart = findViewById(R.id.monthlyChart)
        heatmapView = findViewById(R.id.heatmapView)

        lblDatasetFooter = findViewById(R.id.lblDatasetFooter)
        lblMethodologyNote = findViewById(R.id.lblMethodologyNote)

        btnBack = findViewById(R.id.btnBackLosses)
        btnBack.setOnClickListener { finish() }

        modeRadioGroup.setOnCheckedChangeListener { _, checkedId ->
            val mode = when (checkedId) {
                R.id.rbModeEnergy -> CalculationMode.ENERGY_FULL
                else -> CalculationMode.GEOMETRIC_ONLY
            }
            onModeChanged(mode)
        }
    }

    // -- Setup inicial -------------------------------------------------

    /**
     * Carga perfil y sesión sincrónicamente. Devuelve true si todo OK, false
     * si hubo error (en cuyo caso ya se invocó showError en main thread).
     */
    private suspend fun loadSetupBlocking(): Boolean {
        // 1. Cargar perfil
        val atlasDir = File(sessionDir, "atlas")
        val p = HorizonProfileStore.load(atlasDir)
        if (p == null) {
            withContext(Dispatchers.Main) {
                showError("No hay perfil de obstáculos generado en esta sesión")
            }
            return false
        }

        // 2. Cargar sesión para lat/lon
        val paths = JsonSessionStore().createSessionPaths(sessionDir)
        val session = JsonSessionStore().loadSession(paths)
        val sessionLat = session?.latitudeDeg
        val sessionLon = session?.longitudeDeg
        if (sessionLat == null || sessionLon == null) {
            withContext(Dispatchers.Main) {
                showError("Esta sesión no tiene coordenadas GPS guardadas")
            }
            return false
        }

        // 3. Persistir en campos de la activity
        profile = p
        lat = sessionLat
        lon = sessionLon
        panel = PanelOrientation.optimalFor(sessionLat)
        timezoneOffset = guessTimezoneOffsetFromLongitude(sessionLon)

        Log.i(TAG, "Setup OK: lat=$lat lon=$lon panel=$panel tz=$timezoneOffset")
        return true
    }

    // -- Toggle de modo ------------------------------------------------

    private fun onModeChanged(mode: CalculationMode) {
        if (profile == null) {
            // Setup todavía no terminó. Ignorar el evento (no debería suceder).
            return
        }

        // Si tenemos cache para este modo, mostrarlo directamente.
        val cached = when (mode) {
            CalculationMode.GEOMETRIC_ONLY -> cachedGeometric
            CalculationMode.ENERGY_FULL -> cachedEnergy
        }
        if (cached != null) {
            renderResult(cached, mode)
            return
        }

        // Si no hay cache, calcular. Cancela el job en curso si lo hay.
        currentJob?.cancel()
        currentJob = lifecycleScope.launch {
            runCalculationForMode(mode)
        }
    }

    /**
     * Ejecuta el cálculo para el modo solicitado. Maneja descarga del TMY
     * cuando corresponde y todos los caminos de error.
     */
    private suspend fun runCalculationForMode(mode: CalculationMode) {
        try {
            when (mode) {
                CalculationMode.GEOMETRIC_ONLY -> runGeometric()
                CalculationMode.ENERGY_FULL -> runEnergy()
            }
        } catch (t: Throwable) {
            Log.e(TAG, "Cálculo $mode falló", t)
            withContext(Dispatchers.Main) {
                showError("Error en el cálculo: ${t.message ?: t.javaClass.simpleName}")
            }
        }
    }

    private suspend fun runGeometric() {
        withContext(Dispatchers.Main) {
            showLoading("Calculando pérdidas por sombreado...")
        }

        val config = ShadingLossConfig(
            latitudeDeg = lat,
            longitudeDeg = lon,
            panel = panel,
            timezoneOffsetHours = timezoneOffset,
            calculationMode = CalculationMode.GEOMETRIC_ONLY
        )

        val result = withContext(Dispatchers.Default) {
            ShadingLossCalculator(profile!!, config).compute()
        }

        cachedGeometric = result
        withContext(Dispatchers.Main) {
            renderResult(result, CalculationMode.GEOMETRIC_ONLY)
        }
    }

    private suspend fun runEnergy() {
        // Mensaje contextual: si hay cache del TMY, "cargando"; si no, "descargando".
        val hasCache = tmyDataSource.hasCache(lat, lon)
        val initialMsg = if (hasCache) {
            "Cargando datos meteorológicos del sitio..."
        } else {
            "Descargando datos meteorológicos PVGIS (una sola vez por sitio)..."
        }
        withContext(Dispatchers.Main) { showLoading(initialMsg) }

        val fetchResult = tmyDataSource.fetchOrLoad(
            lat = lat,
            lon = lon,
            year = ShadingLossConfig.DEFAULT_YEAR
        )

        when (fetchResult) {
            is PvgisTmyDataSource.FetchResult.Failure -> {
                Log.w(TAG, "TMY fetch falló: ${fetchResult.reason}")
                withContext(Dispatchers.Main) {
                    showFetchFailureDialog(fetchResult.reason)
                }
                return
            }
            is PvgisTmyDataSource.FetchResult.Success -> {
                cachedMetadata = fetchResult.metadata
                Log.i(
                    TAG,
                    "TMY listo (fromCache=${fetchResult.fromCache}, " +
                            "elev=${fetchResult.metadata.elevationM}m)"
                )
                computeEnergyWithSeries(fetchResult.series)
            }
        }
    }

    private suspend fun computeEnergyWithSeries(series: IrradianceTimeSeries) {
        withContext(Dispatchers.Main) {
            showLoading("Calculando pérdidas energéticas...")
        }

        val config = ShadingLossConfig(
            latitudeDeg = lat,
            longitudeDeg = lon,
            panel = panel,
            timezoneOffsetHours = timezoneOffset,
            calculationMode = CalculationMode.ENERGY_FULL,
            // Hay-Davies elegido como modelo default para uso global del
            // motor, basado en el análisis multi-sitio (N=10) contra PVGIS
            // PVcalc: mejor MBE (-2.18% vs -4.13% del isotrópico) y mejor
            // MAE (3.82% vs 4.59%). Ver DiffuseModel.kt para detalles.
            diffuseModel = DiffuseModel.HAY_DAVIES
        )

        val result = withContext(Dispatchers.Default) {
            ShadingLossCalculator(profile!!, config, series).compute()
        }

        // Si hubiera fallado el modo energético adentro del motor (no debería),
        // hasEnergyData sería false y caemos a geométrico con un aviso.
        if (!result.hasEnergyData) {
            Log.w(TAG, "El motor no produjo datos energéticos pese a ENERGY_FULL")
            withContext(Dispatchers.Main) {
                Toast.makeText(
                    this@LossesActivity,
                    "No se pudieron calcular las pérdidas energéticas. Mostrando geométrico.",
                    Toast.LENGTH_LONG
                ).show()
                // Si no hay cache geométrico, lo calculamos en background.
                if (cachedGeometric == null) {
                    rbModeGeometric.isChecked = true  // dispara onModeChanged
                } else {
                    renderResult(cachedGeometric!!, CalculationMode.GEOMETRIC_ONLY)
                }
            }
            return
        }

        cachedEnergy = result
        // El motor también computa el geométrico aunque el modo sea ENERGY_FULL,
        // así que aprovechamos para llenar el cache geométrico si no estaba.
        if (cachedGeometric == null) {
            cachedGeometric = result
        }

        withContext(Dispatchers.Main) {
            renderResult(result, CalculationMode.ENERGY_FULL)
        }
    }

    // -- Diálogo de error en fetch -------------------------------------

    private fun showFetchFailureDialog(reason: String) {
        AlertDialog.Builder(this)
            .setTitle("No se pudieron descargar datos meteorológicos")
            .setMessage(reason + "\n\n¿Cómo querés continuar?")
            .setCancelable(false)
            .setPositiveButton("Reintentar") { dialog, _ ->
                dialog.dismiss()
                currentJob?.cancel()
                currentJob = lifecycleScope.launch { runCalculationForMode(CalculationMode.ENERGY_FULL) }
            }
            .setNeutralButton("Solo geométrico") { dialog, _ ->
                dialog.dismiss()
                // Volver el toggle a Geométrico (esto dispara onModeChanged).
                rbModeGeometric.isChecked = true
            }
            .setNegativeButton("Cancelar") { dialog, _ ->
                dialog.dismiss()
                // Restaurar toggle a Geométrico y, si tenemos cache, mostrarlo.
                rbModeGeometric.isChecked = true
            }
            .show()
    }

    // -- Render --------------------------------------------------------

    private fun renderResult(result: ShadingLossResult, mode: CalculationMode) {
        loadingIndicator.visibility = View.GONE
        loadingText.visibility = View.GONE
        contentContainer.visibility = View.VISIBLE

        val cfg = result.config

        // --- Header: sitio y panel (no cambia entre modos) ---
        lblSite.text = "Sitio: lat ${"%.3f".format(cfg.latitudeDeg)}°, " +
                "lon ${"%.3f".format(cfg.longitudeDeg)}°"
        lblPanel.text = "Panel óptimo: γ=${"%.0f".format(cfg.panel.azimuthDeg)}° " +
                "(${azimuthLabel(cfg.panel.azimuthDeg)}), " +
                "β=${"%.0f".format(cfg.panel.tiltDeg)}°"

        val showEnergy = mode == CalculationMode.ENERGY_FULL && result.hasEnergyData

        // --- Tarjeta principal: % del modo activo ---
        if (showEnergy) {
            val pct = result.energyAnnualLossPercent!!
            lblAnnualLossTitle.text = "Pérdida energética anual"
            lblAnnualLoss.text = "${"%.2f".format(pct)}%"
            lblAnnualLossSub.text = lossSubtitle(pct)
        } else {
            lblAnnualLossTitle.text = "Pérdida geométrica anual"
            lblAnnualLoss.text = "${"%.2f".format(result.geometricAnnualLossPercent)}%"
            lblAnnualLossSub.text = lossSubtitle(result.geometricAnnualLossPercent)
        }

        // --- Tarjeta secundaria: kWh perdidos (solo modo energético) ---
        if (showEnergy) {
            val lost = result.annualEnergyLostKwhM2!!
            val pot = result.annualEnergyPotentialKwhM2!!
            val real = result.annualEnergyRealKwhM2!!
            lblEnergyLost.text = "${"%.1f".format(lost)} kWh/m²/año"
            lblPoaPotential.text = "POA potencial:  ${"%.1f".format(pot)} kWh/m²/año"
            lblPoaReal.text =      "POA real:       ${"%.1f".format(real)} kWh/m²/año"
            energyCard.visibility = View.VISIBLE
        } else {
            energyCard.visibility = View.GONE
        }

        // --- Bar chart mensual ---
        val monthlyData = if (showEnergy) {
            result.energyMonthlyLossPercent!!
        } else {
            result.geometricMonthlyLossPercent
        }
        setupMonthlyBarChart(monthlyData)

        // --- Heatmap mes × hora ---
        val matrix = if (showEnergy) {
            result.energyHourlyMatrix!!
        } else {
            result.geometricHourlyMatrix
        }
        heatmapView.setMatrix(matrix)

        // --- Footer del dataset (solo modo energético) ---
        if (showEnergy && cachedMetadata != null) {
            val meta = cachedMetadata!!
            val elevPart = meta.elevationM
                ?.let { " | Elevación: ${round(it).toInt()} m" }
                ?: ""
            lblDatasetFooter.text = "Datos: PVGIS-ERA5 TMY (años 2005-2023)$elevPart"
            lblDatasetFooter.visibility = View.VISIBLE
        } else {
            lblDatasetFooter.visibility = View.GONE
        }

        // --- Nota metodológica (texto dinámico) ---
        lblMethodologyNote.text = if (showEnergy) {
            "Método: integración energética horaria (paso 15 min) usando NOAA Solar " +
                    "Calculator, el perfil de horizonte detectado y datos PVGIS-ERA5 TMY. " +
                    "POA según Duffie & Beckman con difusa isotrópica. Albedo: " +
                    "${"%.2f".format(cfg.albedo)}."
        } else {
            "Método: integración geométrica horaria (paso 15 min) usando NOAA Solar " +
                    "Calculator y el perfil de horizonte detectado. Ponderación por " +
                    "cos(θ_inc), sin datos meteorológicos."
        }
    }

    private fun setupMonthlyBarChart(monthly: DoubleArray) {
        val entries = monthly.mapIndexed { i, v -> BarEntry(i.toFloat(), v.toFloat()) }
        val dataSet = BarDataSet(entries, "Pérdida mensual").apply {
            color = Color.parseColor("#FF6B35")
            valueTextColor = Color.WHITE
            valueTextSize = 9f
            valueFormatter = object : ValueFormatter() {
                override fun getFormattedValue(value: Float): String {
                    return if (value < 0.1f) "" else "${"%.1f".format(value)}%"
                }
            }
        }
        val barData = BarData(dataSet).apply { barWidth = 0.7f }

        monthlyChart.apply {
            data = barData
            description.isEnabled = false
            setFitBars(true)
            setDrawGridBackground(false)
            setDrawBorders(false)
            setNoDataText("Sin datos")
            setTouchEnabled(true)
            isDragEnabled = false
            setScaleEnabled(false)
            setPinchZoom(false)
            legend.isEnabled = false

            xAxis.apply {
                valueFormatter = IndexAxisValueFormatter(MONTH_LABELS)
                position = XAxis.XAxisPosition.BOTTOM
                granularity = 1f
                setDrawGridLines(false)
                textColor = Color.WHITE
                textSize = 10f
            }
            axisLeft.apply {
                axisMinimum = 0f
                setDrawGridLines(true)
                gridColor = Color.parseColor("#33FFFFFF")
                textColor = Color.WHITE
                textSize = 10f
                valueFormatter = object : ValueFormatter() {
                    override fun getFormattedValue(value: Float): String = "${value.toInt()}%"
                }
            }
            axisRight.isEnabled = false

            animateY(700)
            invalidate()
        }
    }

    // -- Loading / Error UI --------------------------------------------

    private fun showLoading(message: String) {
        loadingIndicator.visibility = View.VISIBLE
        loadingText.visibility = View.VISIBLE
        loadingText.text = message
        contentContainer.visibility = View.GONE
    }

    private fun showError(message: String) {
        loadingIndicator.visibility = View.GONE
        loadingText.visibility = View.VISIBLE
        loadingText.text = message
        contentContainer.visibility = View.GONE
    }

    // -- Helpers -------------------------------------------------------

    private fun azimuthLabel(azimuthDeg: Double): String {
        val normalized = ((azimuthDeg % 360.0) + 360.0) % 360.0
        return when {
            normalized < 22.5 || normalized >= 337.5 -> "Norte"
            normalized < 67.5 -> "NE"
            normalized < 112.5 -> "Este"
            normalized < 157.5 -> "SE"
            normalized < 202.5 -> "Sur"
            normalized < 247.5 -> "SO"
            normalized < 292.5 -> "Oeste"
            else -> "NO"
        }
    }

    private fun lossSubtitle(loss: Double): String {
        return when {
            loss < 1.0 -> "Sitio prácticamente libre de obstrucciones"
            loss < 5.0 -> "Pérdida baja — sitio aceptable para PV"
            loss < 10.0 -> "Pérdida moderada"
            loss < 20.0 -> "Pérdida significativa — revisar emplazamiento"
            else -> "Pérdida muy alta — sitio comprometido"
        }
    }

    /**
     * Estimación cruda del huso horario a partir de la longitud.
     *   timezone ≈ round(longitude / 15)
     * Sirve como default razonable para visualización de la matriz hora local.
     */
    private fun guessTimezoneOffsetFromLongitude(lon: Double): Double {
        return round(lon / 15.0).coerceIn(-14.0, 14.0)
    }
}