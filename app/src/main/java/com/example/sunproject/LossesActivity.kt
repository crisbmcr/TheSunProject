package com.example.sunproject

import android.graphics.Color
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.example.sunproject.data.storage.JsonSessionStore
import com.example.sunproject.domain.horizon.HorizonProfile
import com.example.sunproject.domain.horizon.HorizonProfileStore
import com.example.sunproject.domain.loss.PanelOrientation
import com.example.sunproject.domain.loss.ShadingLossCalculator
import com.example.sunproject.domain.loss.ShadingLossConfig
import com.example.sunproject.domain.loss.ShadingLossResult
import com.github.mikephil.charting.charts.BarChart
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.BarData
import com.github.mikephil.charting.data.BarDataSet
import com.github.mikephil.charting.data.BarEntry
import com.github.mikephil.charting.formatter.IndexAxisValueFormatter
import com.github.mikephil.charting.formatter.ValueFormatter
import java.io.File
import kotlin.math.abs

/**
 * Pantalla de resultados del cálculo de pérdidas por sombreado.
 *
 * Flujo:
 *  1. onCreate muestra loading.
 *  2. Lanza Thread que: carga perfil + sesión, arma config, ejecuta calculator.
 *  3. Cuando el cálculo termina, renderiza en main thread: resumen, bar chart
 *     mensual, heatmap mes×hora.
 *
 * Inputs por intent extras:
 *  - EXTRA_SESSION_DIR: ruta absoluta del directorio de la sesión.
 *    De ahí se deriva el perfil (sessionDir/atlas/horizon_profile.json) y
 *    los metadatos (sessionDir/session.json).
 *
 * v1 — motor geométrico puro (sin datos meteorológicos). Reporta % pérdida
 * relativa del potencial geométrico anual. En C.2/C.3 se enriquece con
 * datos PVGIS/NASA POWER y entonces el output va a tener también kWh.
 */
class LossesActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_SESSION_DIR = "session_dir"
        private const val TAG = "LossesActivity"
    }

    private lateinit var loadingIndicator: ProgressBar
    private lateinit var loadingText: TextView
    private lateinit var contentContainer: LinearLayout

    private lateinit var lblSite: TextView
    private lateinit var lblPanel: TextView
    private lateinit var lblAnnualLoss: TextView
    private lateinit var lblAnnualLossSub: TextView

    private lateinit var monthlyChart: BarChart
    private lateinit var heatmapView: HeatmapView

    private lateinit var btnBack: Button

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_losses)

        loadingIndicator = findViewById(R.id.lossesLoadingIndicator)
        loadingText = findViewById(R.id.lossesLoadingText)
        contentContainer = findViewById(R.id.lossesContentContainer)

        lblSite = findViewById(R.id.lblSite)
        lblPanel = findViewById(R.id.lblPanel)
        lblAnnualLoss = findViewById(R.id.lblAnnualLoss)
        lblAnnualLossSub = findViewById(R.id.lblAnnualLossSub)

        monthlyChart = findViewById(R.id.monthlyChart)
        heatmapView = findViewById(R.id.heatmapView)

        btnBack = findViewById(R.id.btnBackLosses)
        btnBack.setOnClickListener { finish() }

        val sessionDirStr = intent.getStringExtra(EXTRA_SESSION_DIR)
        if (sessionDirStr.isNullOrBlank()) {
            Toast.makeText(this, "No se recibió el directorio de la sesión", Toast.LENGTH_LONG).show()
            finish()
            return
        }
        val sessionDir = File(sessionDirStr)
        if (!sessionDir.exists() || !sessionDir.isDirectory) {
            Toast.makeText(this, "Directorio de sesión inválido", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        // Estado inicial: loading visible, contenido oculto.
        contentContainer.visibility = View.GONE
        loadingIndicator.visibility = View.VISIBLE
        loadingText.visibility = View.VISIBLE
        loadingText.text = "Calculando pérdidas anuales por sombreado..."

        // Background: cálculo.
        Thread { runCalculation(sessionDir) }.start()
    }

    private fun runCalculation(sessionDir: File) {
        try {
            // 1. Cargar perfil
            val atlasDir = File(sessionDir, "atlas")
            val profile = HorizonProfileStore.load(atlasDir)
            if (profile == null) {
                runOnUiThread {
                    showError("No hay perfil de obstáculos generado en esta sesión")
                }
                return
            }

            // 2. Cargar sesión para lat/lon
            val paths = JsonSessionStore().createSessionPaths(sessionDir)
            val session = JsonSessionStore().loadSession(paths)
            val lat = session?.latitudeDeg
            val lon = session?.longitudeDeg
            if (lat == null || lon == null) {
                runOnUiThread {
                    showError("Esta sesión no tiene coordenadas GPS guardadas")
                }
                return
            }

            // 3. Armar config (default: orientación óptima, Argentina UTC-3)
            val panel = PanelOrientation.optimalFor(lat)
            val timezoneOffset = guessTimezoneOffsetFromLongitude(lon)

            val config = ShadingLossConfig(
                latitudeDeg = lat,
                longitudeDeg = lon,
                panel = panel,
                timezoneOffsetHours = timezoneOffset
            )

            Log.i(TAG, "Iniciando cálculo: lat=$lat lon=$lon panel=$panel tz=$timezoneOffset")

            // 4. Calcular
            val result = ShadingLossCalculator(profile, config).compute()

            // 5. Renderizar
            runOnUiThread {
                renderResult(result, profile)
            }
        } catch (t: Throwable) {
            Log.e(TAG, "Cálculo falló", t)
            runOnUiThread {
                showError("Error en el cálculo: ${t.message ?: t.javaClass.simpleName}")
            }
        }
    }

    private fun renderResult(result: ShadingLossResult, profile: HorizonProfile) {
        loadingIndicator.visibility = View.GONE
        loadingText.visibility = View.GONE
        contentContainer.visibility = View.VISIBLE

        val cfg = result.config

        // --- Encabezado: sitio y panel ---
        lblSite.text = "Sitio: lat ${"%.3f".format(cfg.latitudeDeg)}°, lon ${"%.3f".format(cfg.longitudeDeg)}°"
        lblPanel.text = "Panel óptimo: γ=${"%.0f".format(cfg.panel.azimuthDeg)}° (${azimuthLabel(cfg.panel.azimuthDeg)}), β=${"%.0f".format(cfg.panel.tiltDeg)}°"

        // --- Número grande: pérdida anual ---
        lblAnnualLoss.text = "${"%.2f".format(result.annualLossPercent)}%"
        lblAnnualLossSub.text = lossSubtitle(result.annualLossPercent)

        // --- Bar chart mensual ---
        setupMonthlyBarChart(result.monthlyLossPercent)

        // --- Heatmap mes × hora ---
        heatmapView.setMatrix(result.hourlyMatrix)
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

    private fun showError(message: String) {
        loadingIndicator.visibility = View.GONE
        loadingText.visibility = View.VISIBLE
        loadingText.text = message
        contentContainer.visibility = View.GONE
    }

    // -- Helpers ---------------------------------------------------------

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
     * Argentina (lon -66.78) → -4.45 → redondeado -4, pero el huso oficial
     * es -3. Acepto -4 vs -3 como aproximación de 1 hora — la matriz hora
     * va a estar corrida 1 columna, pero los agregados anual/mensual no se
     * afectan (los porcentajes son los mismos). Si en futuro la sesión
     * persiste el timezone oficial, se usa directo.
     */
    private fun guessTimezoneOffsetFromLongitude(longitudeDeg: Double): Double {
        return Math.round(longitudeDeg / 15.0).toDouble()
    }

    private val MONTH_LABELS = listOf(
        "Ene", "Feb", "Mar", "Abr", "May", "Jun",
        "Jul", "Ago", "Sep", "Oct", "Nov", "Dic"
    )
}