package com.example.sunproject.domain.loss

/**
 * Configuración inmutable de un cálculo de pérdidas por sombreado.
 *
 * Encapsula todos los parámetros que definen la integración geométrica
 * anual. La instancia es agnóstica de la fuente de datos meteorológicos
 * — eso se inyecta como dependencia separada en C.2 (IrradianceTimeSeries).
 */
data class ShadingLossConfig(

    /** Latitud del sitio en grados decimales. Negativa para hemisferio sur. */
    val latitudeDeg: Double,

    /** Longitud del sitio en grados decimales. Negativa para oeste de Greenwich. */
    val longitudeDeg: Double,

    /** Orientación del panel (γ, β). */
    val panel: PanelOrientation,

    /**
     * Offset horario del sitio respecto de UTC, en horas decimales.
     *
     *  - Argentina: -3.0
     *  - Chile continental (estándar): -4.0
     *  - España (estándar): +1.0
     *
     * Solo se usa para distribuir los resultados en la matriz mensual ×
     * horaria local de [ShadingLossResult.hourlyMatrix]. El cálculo
     * astronómico (NOAA) trabaja en UTC sin importar esta config.
     */
    val timezoneOffsetHours: Double,

    /**
     * Año "típico" sobre el que se integra. Debe ser no bisiesto para que
     * el número de muestras sea constante y comparable entre sitios.
     * Cualquier año del rango 1901-2099 da prácticamente el mismo resultado
     * para sombreado puro (la órbita es casi periódica anual).
     */
    val year: Int = DEFAULT_YEAR,

    /**
     * Paso de tiempo de la integración, en minutos. 15 min es el compromiso
     * estándar entre precisión y costo. Debe dividir 1440 (un día) exacto.
     */
    val timeStepMinutes: Int = DEFAULT_TIME_STEP_MIN
) {
    init {
        require(latitudeDeg in -90.0..90.0) {
            "latitudeDeg fuera de rango: $latitudeDeg"
        }
        require(longitudeDeg in -180.0..180.0) {
            "longitudeDeg fuera de rango: $longitudeDeg"
        }
        require(timezoneOffsetHours in -14.0..14.0) {
            "timezoneOffsetHours fuera de rango: $timezoneOffsetHours"
        }
        require(timeStepMinutes in 1..60) {
            "timeStepMinutes debe estar en [1, 60], era $timeStepMinutes"
        }
        require(1440 % timeStepMinutes == 0) {
            "timeStepMinutes ($timeStepMinutes) debe dividir 1440 exacto"
        }
        require(year in 1901..2099) {
            "year fuera del rango soportado por NOAA: $year"
        }
        require(!isLeapYear(year)) {
            "year debe ser no bisiesto para integración consistente (era $year)"
        }
    }

    /**
     * Cantidad total de muestras en el año entero (informativo).
     * Útil para estimar el costo computacional del cálculo.
     */
    val totalSamples: Int
        get() = (365 * 24 * 60) / timeStepMinutes

    companion object {
        const val DEFAULT_YEAR = 2023
        const val DEFAULT_TIME_STEP_MIN = 15

        private fun isLeapYear(year: Int): Boolean {
            return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
        }
    }
}