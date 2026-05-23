package com.example.sunproject.domain.loss

import android.util.Log
import com.example.sunproject.domain.horizon.HorizonProfile
import com.example.sunproject.domain.solar.SolarGeometry
import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneOffset

/**
 * Motor de cálculo de pérdidas por sombreado por integración geométrica
 * sobre todo el año.
 *
 * Algoritmo (v1 — geométrico puro, sin datos meteorológicos):
 *
 *   Para cada instante t en el año (paso configurable):
 *     1. Computar la posición del sol con NOAA (lat, lon, t_UTC).
 *     2. Si el sol está sobre el horizonte teórico:
 *        a. Computar cos(θ_inc) entre el sol y el panel.
 *        b. Si cos(θ_inc) > 0 (sol delante del panel):
 *           - Acumular cos(θ_inc) · Δt al "potencial".
 *           - Consultar el HorizonProfile en el azimut solar.
 *           - Si alt_sol >= alt_horizonte: acumular también al "real".
 *
 *   % pérdida anual = (1 − real / potencial) × 100
 *
 * Limitaciones de v1:
 *   - No usa irradiancia real (asume cada watt geométrico vale lo mismo).
 *   - No considera radiación difusa (sería relevante para sombras parciales).
 *   - No considera albedo del suelo.
 *
 * Estas tres limitaciones se levantan en C.2/v3 reemplazando el factor
 * "cos(θ_inc)" por la fórmula POA completa con DNI, DHI, GHI inyectados
 * desde un IrradianceTimeSeries.
 *
 * Costo computacional: ~35.040 iteraciones por defecto (año no bisiesto,
 * paso 15 min). Cada iteración hace 1 cómputo NOAA + 1 lookup en el perfil
 * + trigonometría. En mobile rinde ~1-2 segundos. Apto para botón
 * "Calcular pérdidas" en UI.
 *
 * Thread safety: la instancia NO es thread-safe; se asume llamada desde
 * un único coroutine (Dispatchers.Default). Crear una instancia por
 * cálculo es barato.
 */
class ShadingLossCalculator(
    private val horizonProfile: HorizonProfile,
    private val config: ShadingLossConfig
) {

    fun compute(): ShadingLossResult {
        val tStart = System.currentTimeMillis()

        // Acumuladores: total mes y matriz mes × hora_local.
        val monthlyPotential = DoubleArray(12)
        val monthlyReal = DoubleArray(12)
        val matrixPotential = Array(12) { DoubleArray(24) }
        val matrixReal = Array(12) { DoubleArray(24) }

        val dtHours = config.timeStepMinutes / 60.0
        val dtMillis = config.timeStepMinutes * 60_000L
        val tzOffsetMillis = (config.timezoneOffsetHours * 3_600_000.0).toLong()

        // Rango de integración: el año entero en UTC.
        val startEpochUtc = LocalDateTime.of(config.year, 1, 1, 0, 0)
            .toInstant(ZoneOffset.UTC).toEpochMilli()
        val endEpochUtc = LocalDateTime.of(config.year, 12, 31, 23, 59)
            .toInstant(ZoneOffset.UTC).toEpochMilli()

        var epoch = startEpochUtc
        var samples = 0
        var sunUpSamples = 0
        var inFrontSamples = 0
        var blockedSamples = 0

        while (epoch <= endEpochUtc) {
            samples++

            val sun = SolarGeometry.computePosition(
                latitudeDeg = config.latitudeDeg,
                longitudeDeg = config.longitudeDeg,
                epochMillisUtc = epoch
            )

            if (!sun.belowHorizon) {
                sunUpSamples++

                val cosInc = PanelGeometry.cosIncidence(sun, config.panel)

                if (cosInc > 0.0) {
                    inFrontSamples++

                    val contribution = cosInc * dtHours

                    // Mes y hora locales del slot al que pertenece.
                    val localEpoch = epoch + tzOffsetMillis
                    val localDt = LocalDateTime.ofInstant(
                        Instant.ofEpochMilli(localEpoch),
                        ZoneOffset.UTC
                    )
                    val monthIdx = (localDt.monthValue - 1).coerceIn(0, 11)
                    val hourIdx = localDt.hour.coerceIn(0, 23)

                    monthlyPotential[monthIdx] += contribution
                    matrixPotential[monthIdx][hourIdx] += contribution

                    // Consulta del perfil. azimuthDeg en [0, 360) — alineado.
                    val altHorizonDeg = horizonProfile
                        .altAtAzimuthDeg(sun.azimuthDeg.toFloat())
                        .toDouble()

                    val isBlocked = sun.altitudeDeg < altHorizonDeg

                    if (!isBlocked) {
                        monthlyReal[monthIdx] += contribution
                        matrixReal[monthIdx][hourIdx] += contribution
                    } else {
                        blockedSamples++
                    }
                }
            }

            epoch += dtMillis
        }

        // Reducciones finales.
        val annualPotential = monthlyPotential.sum()
        val annualReal = monthlyReal.sum()
        val annualLoss = lossPercent(annualReal, annualPotential)

        val monthlyLoss = DoubleArray(12) { m ->
            lossPercent(monthlyReal[m], monthlyPotential[m])
        }

        val hourlyLossMatrix = Array(12) { m ->
            DoubleArray(24) { h ->
                lossPercent(matrixReal[m][h], matrixPotential[m][h])
            }
        }

        val elapsed = System.currentTimeMillis() - tStart
        Log.i(
            TAG,
            "compute() samples=$samples sunUp=$sunUpSamples " +
                    "inFront=$inFrontSamples blocked=$blockedSamples " +
                    "annualLoss=${"%.2f".format(annualLoss)}% " +
                    "elapsed=${elapsed}ms " +
                    "lat=${"%.3f".format(config.latitudeDeg)} " +
                    "lon=${"%.3f".format(config.longitudeDeg)} " +
                    "panel=(γ=${"%.1f".format(config.panel.azimuthDeg)}, " +
                    "β=${"%.1f".format(config.panel.tiltDeg)})"
        )

        return ShadingLossResult(
            annualLossPercent = annualLoss,
            monthlyLossPercent = monthlyLoss,
            hourlyMatrix = hourlyLossMatrix,
            annualPotentialIntegral = annualPotential,
            annualRealIntegral = annualReal,
            config = config
        )
    }

    /**
     * % pérdida = (1 − real / potencial) × 100, con manejo seguro de
     * división por cero.
     *
     * Si el potencial es ~0 (no había nada que captar en esa celda, ej.
     * panel orientado al revés o celda hora/mes sin luz solar), la
     * pérdida por sombreado se define como 0: no perdiste nada porque
     * no había nada para perder. Es la convención correcta para que el
     * agregado anual no se distorsione con celdas nulas.
     */
    private fun lossPercent(real: Double, potential: Double): Double {
        if (potential <= 1e-9) return 0.0
        return ((1.0 - real / potential) * 100.0).coerceIn(0.0, 100.0)
    }

    companion object {
        private const val TAG = "ShadingLossCalc"
    }
}