package com.example.sunproject.domain.solar

import kotlin.math.acos
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.tan

/**
 * Posición del Sol en coordenadas horizontales (observador local).
 *
 * Convenciones — deben coincidir con el resto del proyecto:
 * - azimuthDeg: 0° = Norte geográfico, CW mirando desde arriba.
 *   Este=90°, Sur=180°, Oeste=270°. Mismo sistema que `worldDirectionRad`.
 * - altitudeDeg: 0° = horizonte, 90° = cenit. Negativo => debajo del horizonte.
 * - hourAngleDeg: ω. 0° = mediodía solar. Negativo mañana, positivo tarde.
 *   Usado para construir analemmas sin muestreo.
 *
 * Nomenclatura Clement: γs = azimuthDeg, αs = altitudeDeg, ω = hourAngleDeg.
 */
data class SolarPosition(
    val azimuthDeg: Double,
    val altitudeDeg: Double,
    val hourAngleDeg: Double,
    val declinationDeg: Double
) {
    val belowHorizon: Boolean get() = altitudeDeg < 0.0
}

/**
 * NOAA Solar Calculator.
 * Precisión ~0.02° en el rango 1901-2099.
 * Referencia: https://gml.noaa.gov/grad/solcalc/calcdetails.html
 *
 * El cálculo usa exclusivamente UTC. Para interpretar una hora local,
 * convertila a epoch millis UTC antes de llamar aquí.
 */
object SolarGeometry {

    fun computePosition(
        latitudeDeg: Double,
        longitudeDeg: Double,
        epochMillisUtc: Long
    ): SolarPosition {
        val (declinationRad, eqOfTimeMin) = solarAstronomy(epochMillisUtc)

        val millisInDayUtc = epochMillisUtc.mod(86_400_000L).toDouble()
        val minutesInDayUtc = millisInDayUtc / 60_000.0
        val trueSolarTime = (minutesInDayUtc + eqOfTimeMin + 4.0 * longitudeDeg) % 1440.0
        val hourAngleDeg =
            if (trueSolarTime / 4.0 < 0.0) trueSolarTime / 4.0 + 180.0
            else trueSolarTime / 4.0 - 180.0

        return fromGeometry(
            latitudeDeg = latitudeDeg,
            declinationRad = declinationRad,
            hourAngleDeg = hourAngleDeg
        )
    }

    /**
     * Calcula la posición del Sol desde ángulo horario directo, sin pasar
     * por tiempo civil. Útil para construir analemmas (misma hora solar a
     * lo largo del año) sin tener que muestrear.
     *
     * Para una fecha (year, month, day) el algoritmo resuelve la declinación
     * del mediodía UTC de ese día; esa declinación es suficiente para las
     * analemmas del ábaco.
     */
    fun computeFromHourAngle(
        latitudeDeg: Double,
        year: Int,
        month: Int,
        day: Int,
        solarHour: Double
    ): SolarPosition {
        // Mediodía UTC del día dado.
        val noonUtc = java.time.LocalDate.of(year, month, day)
            .atTime(12, 0)
            .toInstant(java.time.ZoneOffset.UTC)
            .toEpochMilli()

        val (declinationRad, _) = solarAstronomy(noonUtc)
        val hourAngleDeg = 15.0 * (solarHour - 12.0)

        return fromGeometry(
            latitudeDeg = latitudeDeg,
            declinationRad = declinationRad,
            hourAngleDeg = hourAngleDeg
        )
    }

    // ---- Internos ---------------------------------------------------------

    private fun solarAstronomy(epochMillisUtc: Long): Pair<Double, Double> {
        val jd = 2440587.5 + epochMillisUtc / 86_400_000.0
        val jc = (jd - 2451545.0) / 36525.0

        val geomMeanLongSun = (280.46646 + jc * (36000.76983 + jc * 0.0003032)).mod(360.0)
        val geomMeanAnomSun = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
        val eccentEarth = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)

        val m = Math.toRadians(geomMeanAnomSun)
        val sunEqOfCtr =
            sin(m) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) +
                    sin(2 * m) * (0.019993 - 0.000101 * jc) +
                    sin(3 * m) * 0.000289

        val sunTrueLong = geomMeanLongSun + sunEqOfCtr
        val sunAppLong = sunTrueLong - 0.00569 -
                0.00478 * sin(Math.toRadians(125.04 - 1934.136 * jc))

        val meanObliq = 23.0 +
                (26.0 + ((21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813)))) / 60.0) / 60.0
        val obliqCorr = meanObliq +
                0.00256 * cos(Math.toRadians(125.04 - 1934.136 * jc))

        val declinationRad = asin(
            sin(Math.toRadians(obliqCorr)) * sin(Math.toRadians(sunAppLong))
        )

        val varY = tan(Math.toRadians(obliqCorr / 2.0)).pow(2)
        val l0 = Math.toRadians(geomMeanLongSun)
        val eqOfTimeMin = 4.0 * Math.toDegrees(
            varY * sin(2 * l0) -
                    2 * eccentEarth * sin(m) +
                    4 * eccentEarth * varY * sin(m) * cos(2 * l0) -
                    0.5 * varY * varY * sin(4 * l0) -
                    1.25 * eccentEarth * eccentEarth * sin(2 * m)
        )

        return declinationRad to eqOfTimeMin
    }

    private fun fromGeometry(
        latitudeDeg: Double,
        declinationRad: Double,
        hourAngleDeg: Double
    ): SolarPosition {
        val latRad = Math.toRadians(latitudeDeg)
        val haRad = Math.toRadians(hourAngleDeg)

        val cosZenith = sin(latRad) * sin(declinationRad) +
                cos(latRad) * cos(declinationRad) * cos(haRad)
        val zenithRad = acos(cosZenith.coerceIn(-1.0, 1.0))
        val altitudeDeg = 90.0 - Math.toDegrees(zenithRad)

        val sinZenith = sin(zenithRad)
        val azimuthDeg = if (sinZenith < 1e-9) {
            // Sol en cenit/nadir: azimut indefinido, devolvemos 0 como convención.
            0.0
        } else {
            val cosAz = ((sin(latRad) * cos(zenithRad)) - sin(declinationRad)) /
                    (cos(latRad) * sinZenith)
            val azRaw = Math.toDegrees(acos(cosAz.coerceIn(-1.0, 1.0)))
            if (hourAngleDeg > 0.0) (azRaw + 180.0) % 360.0
            else (540.0 - azRaw) % 360.0
        }

        return SolarPosition(
            azimuthDeg = azimuthDeg,
            altitudeDeg = altitudeDeg,
            hourAngleDeg = hourAngleDeg,
            declinationDeg = Math.toDegrees(declinationRad)
        )
    }
}