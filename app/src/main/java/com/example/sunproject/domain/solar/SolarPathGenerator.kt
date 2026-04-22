package com.example.sunproject.domain.solar

import java.time.LocalDate
import java.time.ZoneOffset

/** Una curva del ábaco (trayectoria diaria o línea horaria/analemma). */
data class SolarPath(
    val kind: Kind,
    val label: String,
    val monthIndex: Int?,   // 1..12 si kind==DAILY, null si HOURLY
    val solarHour: Int?,    // 6..18 si kind==HOURLY, null si DAILY
    val points: List<SolarPosition>
) {
    enum class Kind { DAILY, HOURLY }
}

/** Colección completa lista para renderizar. */
data class SolarChart(
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val year: Int,
    val dailyPaths: List<SolarPath>,   // 12 curvas (21 de cada mes)
    val hourlyPaths: List<SolarPath>,  // 7 analemmas cada 2h solares
    val sunAtCapture: SolarPosition?
)

object SolarPathGenerator {

    private val HOURS_FOR_LINES = listOf(6, 8, 10, 12, 14, 16, 18)
    private const val DAILY_SAMPLE_MINUTES = 10

    fun generate(
        latitudeDeg: Double,
        longitudeDeg: Double,
        year: Int,
        captureEpochMillisUtc: Long? = null
    ): SolarChart {
        val dailyPaths = (1..12).map { month ->
            buildDailyPath(latitudeDeg, longitudeDeg, year, month)
        }
        val hourlyPaths = HOURS_FOR_LINES.map { hour ->
            buildHourlyPath(latitudeDeg, year, hour)
        }
        val sunAtCapture = captureEpochMillisUtc?.let {
            SolarGeometry.computePosition(latitudeDeg, longitudeDeg, it)
        }

        return SolarChart(
            latitudeDeg = latitudeDeg,
            longitudeDeg = longitudeDeg,
            year = year,
            dailyPaths = dailyPaths,
            hourlyPaths = hourlyPaths,
            sunAtCapture = sunAtCapture
        )
    }

    /** Trayectoria diaria del 21 del mes: 1 muestra cada N minutos UTC. */
    private fun buildDailyPath(
        latDeg: Double, lonDeg: Double, year: Int, month: Int
    ): SolarPath {
        val midnightUtc = LocalDate.of(year, month, 21)
            .atStartOfDay(ZoneOffset.UTC).toInstant().toEpochMilli()
        val stepMs = DAILY_SAMPLE_MINUTES * 60_000L
        val samples = 24 * 60 / DAILY_SAMPLE_MINUTES

        val points = (0..samples).map { i ->
            SolarGeometry.computePosition(latDeg, lonDeg, midnightUtc + i * stepMs)
        }

        return SolarPath(
            kind = SolarPath.Kind.DAILY,
            label = monthShortLabel(month),
            monthIndex = month,
            solarHour = null,
            points = points
        )
    }

    /**
     * Analemma: la misma hora solar a lo largo del año. 12 puntos (uno por mes,
     * el día 21) calculados directamente desde el ángulo horario — sin depender
     * de hora local ni ecuación del tiempo.
     */
    private fun buildHourlyPath(
        latDeg: Double, year: Int, solarHour: Int
    ): SolarPath {
        val points = (1..12).map { month ->
            SolarGeometry.computeFromHourAngle(
                latitudeDeg = latDeg,
                year = year,
                month = month,
                day = 21,
                solarHour = solarHour.toDouble()
            )
        }

        return SolarPath(
            kind = SolarPath.Kind.HOURLY,
            label = "${solarHour}h",
            monthIndex = null,
            solarHour = solarHour,
            points = points
        )
    }

    private fun monthShortLabel(month: Int): String = listOf(
        "ene", "feb", "mar", "abr", "may", "jun",
        "jul", "ago", "sep", "oct", "nov", "dic"
    )[month - 1]
}