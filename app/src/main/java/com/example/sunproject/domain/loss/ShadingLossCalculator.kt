package com.example.sunproject.domain.loss

import android.util.Log
import com.example.sunproject.domain.horizon.HorizonProfile
import com.example.sunproject.domain.loss.irradiance.IrradianceTimeSeries
import com.example.sunproject.domain.solar.SolarGeometry
import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneOffset
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Motor de cálculo de pérdidas por sombreado por integración anual.
 *
 * ## Modos de cálculo
 *
 * El motor soporta dos modos según [ShadingLossConfig.calculationMode]:
 *
 * ### GEOMETRIC_ONLY (default, C.1+)
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
 * ### ENERGY_FULL (C.2.2+)
 *
 * Además del cómputo geométrico, integra la POA real con datos de
 * irradiancia inyectados vía [IrradianceTimeSeries]:
 *
 *   POA_potencial(t) = DNI(t) · max(0, cos(θ_inc))
 *                    + DHI(t) · FV_sky
 *                    + GHI(t) · albedo · FV_ground
 *
 *   POA_real(t) = directaUtilizada(t)
 *               + DHI(t) · FV_sky
 *               + GHI(t) · albedo · FV_ground
 *
 *   directaUtilizada(t) = DNI(t) · cos(θ_inc)   si cos(θ_inc) > 0 y NO bloqueado
 *                       = 0                      en otro caso
 *
 * Donde:
 *   FV_sky    = (1 + cos(β)) / 2   factor de vista del cielo desde el panel
 *   FV_ground = (1 − cos(β)) / 2   factor de vista del suelo desde el panel
 *   β         = tilt del panel (ShadingLossConfig.panel.tiltDeg)
 *
 * % pérdida energética = (1 − POA_real_anual / POA_potencial_anual) × 100
 *
 * ## Diferencia importante en rango de integración
 *
 * El bloque geométrico solo cuenta instantes con cos(θ_inc) > 0 (sol
 * delante del panel). El bloque energético cuenta TODOS los instantes
 * con sol sobre el horizonte teórico, porque difusa y reflejada llegan
 * aunque el sol esté detrás del panel.
 *
 * ## Limitaciones documentadas del modelo energético v2
 *
 *  - Solo la componente directa se reduce por obstáculos del horizonte.
 *    La componente difusa mantiene DHI · FV_sky aunque el cielo esté
 *    parcialmente obstruido. Una implementación rigurosa del Sky View
 *    Factor obstruido requeriría integrar el HorizonProfile sobre el
 *    hemisferio (planificado v3).
 *  - Sin corrección térmica (T2m y WS10m no se usan aunque PVGIS los
 *    descargue).
 *  - Modelo isotrópico de difusa (no Hay-Davies ni Perez). PVGIS PVcalc
 *    usa Hay-Davies; diferencia típica ±2-3% sobre el POA anual.
 *
 * ## Costo computacional
 *
 * GEOMETRIC_ONLY: ~35.040 iteraciones (año no bisiesto, paso 15 min).
 * Cada iteración: 1 NOAA + 1 lookup horizonte + trigonometría. Mobile ~250 ms.
 *
 * ENERGY_FULL: lo mismo + 1 lookup irradiance + 5 multiplicaciones por sample.
 * El sobrecosto en mobile es <50 ms; total <400 ms. Sigue siendo apto
 * para botón "Calcular pérdidas" sin loading prolongado.
 *
 * ## Thread safety
 *
 * La instancia NO es thread-safe; se asume llamada desde un único coroutine
 * (Dispatchers.Default). Crear una instancia por cálculo es barato.
 */
class ShadingLossCalculator(
    private val horizonProfile: HorizonProfile,
    private val config: ShadingLossConfig,
    private val irradiance: IrradianceTimeSeries? = null
) {

    fun compute(): ShadingLossResult {
        val tStart = System.currentTimeMillis()

        // Determinar si el cálculo energético está activo.
        // Activo sii el caller pidió ENERGY_FULL Y proveyó una IrradianceTimeSeries.
        // Si pidió ENERGY_FULL sin proveer fuente, degradamos a geométrico
        // con warning — mismo patrón defensivo de C.2.0.
        val computeEnergy = config.calculationMode == CalculationMode.ENERGY_FULL
                && irradiance != null

        if (config.calculationMode == CalculationMode.ENERGY_FULL && irradiance == null) {
            Log.w(
                TAG,
                "ENERGY_FULL solicitado pero irradiance es null. " +
                        "Devolviendo solo métricas geométricas; campos energy* quedarán en null."
            )
        }

        // ============================================================
        // Acumuladores geométricos (siempre activos)
        // ============================================================
        val monthlyPotential = DoubleArray(12)
        val monthlyReal = DoubleArray(12)
        val matrixPotential = Array(12) { DoubleArray(24) }
        val matrixReal = Array(12) { DoubleArray(24) }

        // ============================================================
        // Acumuladores energéticos (alocados solo si computeEnergy)
        // Unidades internas: Wh/m². Se convierten a kWh/m² al final.
        // ============================================================
        val monthlyEnergyPotentialWh: DoubleArray? = if (computeEnergy) DoubleArray(12) else null
        val monthlyEnergyRealWh: DoubleArray? = if (computeEnergy) DoubleArray(12) else null
        val matrixEnergyPotentialWh: Array<DoubleArray>? =
            if (computeEnergy) Array(12) { DoubleArray(24) } else null
        val matrixEnergyRealWh: Array<DoubleArray>? =
            if (computeEnergy) Array(12) { DoubleArray(24) } else null

        // ============================================================
        // Constantes pre-calculadas para evitar trigonometría dentro del loop
        // ============================================================
        val dtHours = config.timeStepMinutes / 60.0
        val dtMillis = config.timeStepMinutes * 60_000L
        val tzOffsetMillis = (config.timezoneOffsetHours * 3_600_000.0).toLong()

        // Factores de vista del panel (Duffie & Beckman, transposición isotrópica)
        val tiltRad = Math.toRadians(config.panel.tiltDeg)
        val cosTilt = cos(tiltRad)
        val fvSky = (1.0 + cosTilt) / 2.0
        val fvGround = (1.0 - cosTilt) / 2.0
        val albedo = config.albedo

        // Rango de integración: el año entero en UTC.
        val startEpochUtc = LocalDateTime.of(config.year, 1, 1, 0, 0)
            .toInstant(ZoneOffset.UTC).toEpochMilli()
        val endEpochUtc = LocalDateTime.of(config.year, 12, 31, 23, 59)
            .toInstant(ZoneOffset.UTC).toEpochMilli()

        // ============================================================
        // Loop principal de integración
        // ============================================================
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

            if (sun.belowHorizon) {
                // Sol bajo el horizonte teórico: ni geométrico ni energético
                // tienen nada que aportar. PVGIS-ERA5 confirma que DNI/DHI/GHI=0
                // en estos timestamps. Saltamos.
                epoch += dtMillis
                continue
            }

            sunUpSamples++

            val cosInc = PanelGeometry.cosIncidence(sun, config.panel)

            // Slot temporal local del instante (compartido entre geom y energ).
            val localEpoch = epoch + tzOffsetMillis
            val localDt = LocalDateTime.ofInstant(
                Instant.ofEpochMilli(localEpoch),
                ZoneOffset.UTC
            )
            val monthIdx = (localDt.monthValue - 1).coerceIn(0, 11)
            val hourIdx = localDt.hour.coerceIn(0, 23)

            // Bloqueo por horizonte (compartido entre geom y energ).
            val altHorizonDeg = horizonProfile
                .altAtAzimuthDeg(sun.azimuthDeg.toFloat())
                .toDouble()
            val isBlocked = sun.altitudeDeg < altHorizonDeg

            // ----------------------------------------------------------
            // Bloque GEOMÉTRICO — solo cuando cos(θ_inc) > 0
            // ----------------------------------------------------------
            if (cosInc > 0.0) {
                inFrontSamples++

                val contribution = cosInc * dtHours
                monthlyPotential[monthIdx] += contribution
                matrixPotential[monthIdx][hourIdx] += contribution

                if (!isBlocked) {
                    monthlyReal[monthIdx] += contribution
                    matrixReal[monthIdx][hourIdx] += contribution
                } else {
                    blockedSamples++
                }
            }

            // ----------------------------------------------------------
            // Bloque ENERGÉTICO — activo siempre que computeEnergy y sol arriba
            // del horizonte teórico. NO requiere cos(θ_inc) > 0 porque difusa
            // y reflejada llegan también con sol detrás del panel.
            // ----------------------------------------------------------
            if (computeEnergy) {
                val sample = irradiance!!.at(epoch)

                // Componentes POA en W/m².
                // Directa potencial: lo máximo que el panel podría captar
                // si no hubiera obstáculos del horizonte.
                val directPotentialWm2 =
                    if (cosInc > 0.0) sample.dniWm2 * cosInc else 0.0
                // Directa real: bloqueada si el horizonte obstruye la dirección
                // del sol. La condición de "delante del panel" sigue siendo
                // necesaria (no se puede recibir directa de atrás).
                val directRealWm2 =
                    if (cosInc > 0.0 && !isBlocked) sample.dniWm2 * cosInc else 0.0

                // Difusa según el modelo de transposición elegido.
                //
                // Ambos modelos producen IGUAL difusa potencial y real
                // (la difusa no se bloquea por el horizonte en v2 del motor,
                // ni la isotrópica ni la circumsolar de Hay-Davies — ver
                // DiffuseModel.kt para la justificación).
                val diffuseWm2: Double = when (config.diffuseModel) {
                    DiffuseModel.ISOTROPIC -> {
                        // Liu-Jordan 1963: difusa proyectada con factor de
                        // vista del cielo, cielo isotrópico.
                        sample.dhiWm2 * fvSky
                    }
                    DiffuseModel.HAY_DAVIES -> {
                        // Hay-Davies 1980: separa la difusa en componente
                        // circumsolar (tratada como directa, proyectada
                        // con rb) e isotrópica residual.
                        //
                        // Referencia: Duffie & Beckman 2013 ec. 2.16.7
                        // Validación: multi-sitio N=10 contra PVGIS PVcalc.

                        // I_0n = irradiancia extraterrestre normal del día
                        // (W/m²). Spencer 1971 con G_SC = 1361 W/m².
                        val n = localDt.dayOfYear
                        val I0n = G_SC * (1.0 + 0.033 * cos(2.0 * PI * n / 365.0))

                        // cos(θ_z) = sin(altitudeSolar). En este punto sun
                        // está arriba del horizonte (sunUp), entonces
                        // altitudeDeg > 0 y cosZen > 0.
                        val cosZen = sin(Math.toRadians(sun.altitudeDeg))

                        // A_i = índice de anisotropía. Acotado a [0, 1] por
                        // seguridad (DNI puede teóricamente exceder I_0n en
                        // condiciones de refracción atmosférica extrema).
                        val Ai = if (I0n > 0.0) {
                            (sample.dniWm2 / I0n).coerceIn(0.0, 1.0)
                        } else 0.0

                        // r_b = razón de transposición de la directa para
                        // la fracción circumsolar. Se evita dividir cuando
                        // cosZen es muy chico (sol cerca del horizonte) para
                        // no amplificar errores numéricos. Threshold 0.01
                        // ≈ 89.4° de altitud cenital (sol a 0.6° del
                        // horizonte teórico).
                        val rb = if (cosZen > 0.01) {
                            maxOf(0.0, cosInc) / cosZen
                        } else 0.0

                        // Difusa total = circumsolar + isotrópica residual.
                        // Cuando A_i = 0 (sin DNI), se reduce exactamente al
                        // isotrópico. Cuando A_i = 1 (DNI = extraterrestre,
                        // cielo extremadamente limpio), toda la difusa se
                        // trata como directa.
                        sample.dhiWm2 * (Ai * rb + (1.0 - Ai) * fvSky)
                    }
                }

                // Reflejada: igual en ambos modelos (no afecta al modelo
                // de difusa, depende solo de GHI y del albedo del suelo).
                val reflectedWm2 = sample.ghiWm2 * albedo * fvGround

                val poaPotentialWm2 = directPotentialWm2 + diffuseWm2 + reflectedWm2
                val poaRealWm2 = directRealWm2 + diffuseWm2 + reflectedWm2

                // Energía en Wh/m² para el sub-paso (W/m² × h = Wh/m²).
                val energyPotentialWh = poaPotentialWm2 * dtHours
                val energyRealWh = poaRealWm2 * dtHours

                monthlyEnergyPotentialWh!![monthIdx] += energyPotentialWh
                monthlyEnergyRealWh!![monthIdx] += energyRealWh
                matrixEnergyPotentialWh!![monthIdx][hourIdx] += energyPotentialWh
                matrixEnergyRealWh!![monthIdx][hourIdx] += energyRealWh
            }

            epoch += dtMillis
        }

        // ============================================================
        // Reducciones finales — GEOMÉTRICO
        // ============================================================
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

        // ============================================================
        // Reducciones finales — ENERGÉTICO (si activo)
        // Conversión Wh → kWh dividiendo por 1000.
        // ============================================================
        val energyAnnualLossPercent: Double?
        val energyMonthlyLossPercent: DoubleArray?
        val energyHourlyMatrix: Array<DoubleArray>?
        val annualEnergyPotentialKwhM2: Double?
        val annualEnergyRealKwhM2: Double?
        val monthlyEnergyPotentialKwhM2: DoubleArray?
        val monthlyEnergyRealKwhM2: DoubleArray?

        if (computeEnergy) {
            val pot = monthlyEnergyPotentialWh!!
            val real = monthlyEnergyRealWh!!
            val matPot = matrixEnergyPotentialWh!!
            val matReal = matrixEnergyRealWh!!

            monthlyEnergyPotentialKwhM2 = DoubleArray(12) { m -> pot[m] / 1000.0 }
            monthlyEnergyRealKwhM2 = DoubleArray(12) { m -> real[m] / 1000.0 }
            annualEnergyPotentialKwhM2 = monthlyEnergyPotentialKwhM2.sum()
            annualEnergyRealKwhM2 = monthlyEnergyRealKwhM2.sum()

            energyAnnualLossPercent = lossPercent(real.sum(), pot.sum())
            energyMonthlyLossPercent = DoubleArray(12) { m -> lossPercent(real[m], pot[m]) }
            energyHourlyMatrix = Array(12) { m ->
                DoubleArray(24) { h -> lossPercent(matReal[m][h], matPot[m][h]) }
            }
        } else {
            energyAnnualLossPercent = null
            energyMonthlyLossPercent = null
            energyHourlyMatrix = null
            annualEnergyPotentialKwhM2 = null
            annualEnergyRealKwhM2 = null
            monthlyEnergyPotentialKwhM2 = null
            monthlyEnergyRealKwhM2 = null
        }

        // ============================================================
        // Logging
        // ============================================================
        val elapsed = System.currentTimeMillis() - tStart
        val baseLog = "compute() samples=$samples sunUp=$sunUpSamples " +
                "inFront=$inFrontSamples blocked=$blockedSamples " +
                "geomAnnualLoss=${"%.2f".format(geomAnnualLoss)}% " +
                "elapsed=${elapsed}ms " +
                "lat=${"%.3f".format(config.latitudeDeg)} " +
                "lon=${"%.3f".format(config.longitudeDeg)} " +
                "panel=(γ=${"%.1f".format(config.panel.azimuthDeg)}, " +
                "β=${"%.1f".format(config.panel.tiltDeg)}) " +
                "mode=${config.calculationMode} " +
                "energyComputed=$computeEnergy"

        val energyLog = if (computeEnergy) {
            " energyAnnualLoss=${"%.2f".format(energyAnnualLossPercent!!)}% " +
                    "poaPotential=${"%.1f".format(annualEnergyPotentialKwhM2!!)} kWh/m²/año " +
                    "poaReal=${"%.1f".format(annualEnergyRealKwhM2!!)} kWh/m²/año " +
                    "albedo=${"%.2f".format(albedo)}"
        } else ""

        Log.i(TAG, baseLog + energyLog)

        // ============================================================
        // Construcción del resultado
        // ============================================================
        return ShadingLossResult(
            geometricAnnualLossPercent = geomAnnualLoss,
            geometricMonthlyLossPercent = geomMonthlyLoss,
            geometricHourlyMatrix = geomHourlyMatrix,
            geometricAnnualPotential = geomAnnualPotential,
            geometricAnnualReal = geomAnnualReal,
            energyAnnualLossPercent = energyAnnualLossPercent,
            energyMonthlyLossPercent = energyMonthlyLossPercent,
            energyHourlyMatrix = energyHourlyMatrix,
            annualEnergyPotentialKwhM2 = annualEnergyPotentialKwhM2,
            annualEnergyRealKwhM2 = annualEnergyRealKwhM2,
            monthlyEnergyPotentialKwhM2 = monthlyEnergyPotentialKwhM2,
            monthlyEnergyRealKwhM2 = monthlyEnergyRealKwhM2,
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

        /**
         * Constante solar (W/m²). Valor SI recomendado por la WMO desde 2015.
         * Usada solo por el modelo Hay-Davies para computar la irradiancia
         * extraterrestre normal del día (necesaria para el índice de
         * anisotropía A_i = DNI / I_0n).
         */
        private const val G_SC = 1361.0
    }
}