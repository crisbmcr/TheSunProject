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
 * ## Algoritmo (modo GEOMETRIC_ONLY, único disponible hasta C.2.1)
 *
 *   Para cada instante t en el año (paso configurable):
 *     1. Computar la posición del sol con NOAA (lat, lon, t_UTC).
 *     2. Si el sol está sobre el horizonte teórico:
 *        a. Computar cos(θ_inc) entre el sol y el panel.
 *        b. Si cos(θ_inc) > 0 (sol delante del panel):
 *           - Acumular cos(θ_inc) · Δt al "potencial geométrico".
 *           - Consultar el HorizonProfile en el azimut solar.
 *           - Si alt_sol ≥ alt_horizonte: acumular también al "real".
 *
 *   % pérdida geométrica anual = (1 − real / potencial) × 100
 *
 * ## Modo ENERGY_FULL (C.2.2+)
 *
 * En C.2.2 se agrega el modo que además multiplica cada contribución por
 * la POA real (DNI×cos(θ_inc) + DHI×FV_sky + GHI×albedo×FV_ground)
 * usando una `IrradianceTimeSeries` inyectada. En la presente fase (C.2.0)
 * el motor acepta [ShadingLossConfig.calculationMode] == `ENERGY_FULL` pero
 * no produce métricas energéticas — solo loguea un warning y devuelve los
 * campos `energy*` en `null`.
 *
 * ## Costo computacional
 *
 * ~35.040 iteraciones por defecto (año no bisiesto, paso 15 min). Cada
 * iteración hace 1 cómputo NOAA + 1 lookup en el perfil + trigonometría.
 * En mobile rinde ~250-500 ms. Apto para botón "Calcular pérdidas" en UI
 * sin loading prolongado.
 *
 * ## Thread safety
 *
 * La instancia NO es thread-safe; se asume llamada desde un único coroutine
 * (Dispatchers.Default). Crear una instancia por cálculo es barato.
 */
class ShadingLossCalculator(
    private val horizonProfile: HorizonProfile,
    private val config: ShadingLossConfig
) {

    fun compute(): ShadingLossResult {
        val tStart = System.currentTimeMillis()

        // C.2.0: el modo ENERGY_FULL todavía no está implementado en el motor.
        // Si el caller lo solicita, advertimos por log pero seguimos con el
        // cálculo geométrico para no romper la UI. C.2.2 cierra esto.
        if (config.calculationMode == CalculationMode.ENERGY_FULL) {
            Log.w(
                TAG,
                "ENERGY_FULL solicitado pero IrradianceTimeSeries no está conectada (pendiente C.2.2). " +
                        "Devolviendo solo métricas geométricas; campos energy* quedarán en null."
            )
        }

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

        // Reducciones finales — todas en el dominio geométrico.
        val geomAnnualPotential = monthlyPotential.sum()
        val geomAnnualReal = monthlyReal.sum()
        val geomAnnualLoss = lossPercent(geomAnnualReal, geomAnnualPotential)

        val geomMonthlyLoss = DoubleArray(12) { m ->
            lossPercent(monthlyReal[m], monthlyPotential[m])
        }

        val geomHourlyMatrix = Array(12) { m ->
            DoubleArray(24) { h ->
                lossPercent(matrixReal[m][h], matrixPotential[m][h])
            }
        }

        val elapsed = System.currentTimeMillis() - tStart
        Log.i(
            TAG,
            "compute() samples=$samples sunUp=$sunUpSamples " +
                    "inFront=$inFrontSamples blocked=$blockedSamples " +
                    "geomAnnualLoss=${"%.2f".format(geomAnnualLoss)}% " +
                    "elapsed=${elapsed}ms " +
                    "lat=${"%.3f".format(config.latitudeDeg)} " +
                    "lon=${"%.3f".format(config.longitudeDeg)} " +
                    "panel=(γ=${"%.1f".format(config.panel.azimuthDeg)}, " +
                    "β=${"%.1f".format(config.panel.tiltDeg)}) " +
                    "mode=${config.calculationMode}"
        )

        return ShadingLossResult(
            geometricAnnualLossPercent = geomAnnualLoss,
            geometricMonthlyLossPercent = geomMonthlyLoss,
            geometricHourlyMatrix = geomHourlyMatrix,
            geometricAnnualPotential = geomAnnualPotential,
            geometricAnnualReal = geomAnnualReal,
            // Campos energéticos quedan en null hasta que C.2.2 conecte
            // la IrradianceTimeSeries y el modo ENERGY_FULL los compute.
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