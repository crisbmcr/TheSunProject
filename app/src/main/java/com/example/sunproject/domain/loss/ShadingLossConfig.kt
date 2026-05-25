package com.example.sunproject.domain.loss

/**
 * Configuración inmutable de un cálculo de pérdidas por sombreado.
 *
 * Encapsula todos los parámetros que definen la integración anual.
 * La instancia es agnóstica de la fuente de datos meteorológicos —
 * cuando [calculationMode] == [CalculationMode.ENERGY_FULL], la fuente
 * de irradiancia (IrradianceTimeSeries) se inyecta como dependencia
 * separada del motor (C.2.2).
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
     * horaria local de [ShadingLossResult.geometricHourlyMatrix]. El
     * cálculo astronómico (NOAA) trabaja en UTC sin importar esta config.
     */
    val timezoneOffsetHours: Double,

    /**
     * Año "típico" sobre el que se integra. Debe ser no bisiesto para que
     * el número de muestras sea constante y comparable entre sitios.
     * Cualquier año del rango 1901-2099 da prácticamente el mismo resultado
     * para sombreado puro (la órbita es casi periódica anual).
     *
     * Nota: cuando [calculationMode] == [CalculationMode.ENERGY_FULL], el
     * cálculo energético usa el TMY de PVGIS (año meteorológico típico
     * construido con 2005-2023), pero la geometría sol-panel sigue
     * computándose para este año. La elección del año no afecta el
     * resultado energético en forma significativa.
     */
    val year: Int = DEFAULT_YEAR,

    /**
     * Paso de tiempo de la integración, en minutos. 15 min es el compromiso
     * estándar entre precisión y costo. Debe dividir 1440 (un día) exacto.
     */
    val timeStepMinutes: Int = DEFAULT_TIME_STEP_MIN,

    /**
     * Modo de cálculo. Default [CalculationMode.GEOMETRIC_ONLY] (C.1).
     * Cuando es [CalculationMode.ENERGY_FULL] el motor además computa
     * pérdidas energéticas usando irradiancia real. Ver [CalculationMode]
     * para estado de implementación por fase.
     */
    val calculationMode: CalculationMode = CalculationMode.GEOMETRIC_ONLY,

    /**
     * Albedo del suelo, adimensional en [0, 1]. Fracción de la GHI que el
     * suelo refleja difusamente hacia el panel. Solo se usa cuando
     * [calculationMode] == [CalculationMode.ENERGY_FULL].
     *
     * Default 0.20: estándar de la industria PV (PVGIS, PVsyst, SAM) para
     * "suelo arenoso claro". Para sistemas monofaciales con tilt típico
     * (β < 30°) la sensibilidad del POA anual al albedo es < ±0.5% en
     * rangos realistas de suelo (validado paramétricamente sobre el TMY
     * de Cauchari), por lo que el default 0.20 es seguro para la gran
     * mayoría de los sitios sin nieve estacional.
     *
     * Casos donde conviene ajustar manualmente:
     *  - Sitio con nieve invernal:           0.60 - 0.80 estacional.
     *  - Salar o desierto muy brillante:     0.30 - 0.35.
     *  - Suelo agrícola húmedo o pasto:      0.10 - 0.15.
     */
    val albedo: Double = DEFAULT_ALBEDO
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
        require(albedo in 0.0..1.0) {
            "albedo fuera de rango físico [0, 1]: $albedo"
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
        const val DEFAULT_ALBEDO = 0.20

        private fun isLeapYear(year: Int): Boolean {
            return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
        }
    }
}