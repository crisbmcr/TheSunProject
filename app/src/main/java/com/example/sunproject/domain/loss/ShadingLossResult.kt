package com.example.sunproject.domain.loss

/**
 * Resultado de un cálculo de pérdidas por sombreado.
 *
 * Mantiene dos métricas conceptualmente distintas que conviene no confundir:
 *
 * ## Métrica GEOMÉTRICA (siempre presente)
 *
 * Pérdida ponderada por cos(θ_inc), sin irradiancia. Mide cuánto del
 * "potencial geométrico de captación" está obstruido por el horizonte.
 * Es independiente del clima y útil para evaluar la calidad del
 * emplazamiento per se. Disponible desde C.1.
 *
 *   pérdida_geom = (1 − Σ cos(θ_inc) · Δt [no bloqueado]
 *                       / Σ cos(θ_inc) · Δt [total]) × 100
 *
 * ## Métrica ENERGÉTICA (presente solo en modo ENERGY_FULL)
 *
 * Pérdida ponderada por la POA real, con datos PVGIS-ERA5 TMY del sitio.
 * Mide cuánto de la energía esperada se pierde por sombreado, en % y en
 * kWh/m² absolutos. Disponible desde C.2.2 (campos en `null` en fases
 * previas C.2.0 y C.2.1).
 *
 *   POA = DNI × cos(θ_inc) + DHI × FV_sky + GHI × albedo × FV_ground
 *
 *   pérdida_energ = (1 − Σ POA · Δt [no bloqueado]
 *                        / Σ POA · Δt [total]) × 100
 *
 * Para chequear disponibilidad de los campos energéticos usar [hasEnergyData].
 *
 * Convenciones para todos los porcentajes:
 *   - Rango [0, 100]
 *   - 0%   = sin pérdidas
 *   - 100% = pérdida total
 */
data class ShadingLossResult(

    // ============================================================
    // GEOMÉTRICO — siempre poblado
    // ============================================================

    /**
     * Pérdida geométrica anual total en %.
     * Es la métrica primaria de C.1 y la que reporta el motor cuando
     * `config.calculationMode == GEOMETRIC_ONLY`.
     */
    val geometricAnnualLossPercent: Double,

    /**
     * Pérdida geométrica mensual en %. Índice 0 = enero, 11 = diciembre.
     * Cada elemento puede ser 0 (si no había nada que captar en ese mes
     * para esta orientación) o un valor en (0, 100].
     */
    val geometricMonthlyLossPercent: DoubleArray,

    /**
     * Matriz [12 meses][24 horas locales] con la pérdida geométrica % en
     * cada celda. `geometricMonthlyLossPercent[m]` es el promedio ponderado
     * de `geometricHourlyMatrix[m][*]`.
     *
     * Horas locales se determinan aplicando [ShadingLossConfig.timezoneOffsetHours]
     * al timestamp UTC.
     */
    val geometricHourlyMatrix: Array<DoubleArray>,

    /**
     * Integral anual del potencial geométrico (sin sombras):
     *   Σ cos(θ_inc) · Δt[h] sobre todo el año.
     * Suma adimensional. NO representa kWh/m²; para energía absoluta
     * usar [annualEnergyPotentialKwhM2].
     */
    val geometricAnnualPotential: Double,

    /**
     * Integral anual de la captación real (con sombras).
     * Misma unidad que [geometricAnnualPotential].
     */
    val geometricAnnualReal: Double,

    // ============================================================
    // ENERGÉTICO — poblado solo en modo ENERGY_FULL (C.2.2+)
    //
    // En C.2.0 todos estos campos vienen en null. Los placeholders se
    // dejan ahora para que la API quede estable y C.2.2 sea un fill-in,
    // no un breaking change.
    // ============================================================

    /**
     * Pérdida energética anual total en %, considerando POA real.
     * `null` cuando el cálculo se hizo en modo `GEOMETRIC_ONLY` o cuando
     * la descarga del TMY falló y se degradó a geométrico.
     */
    val energyAnnualLossPercent: Double? = null,

    /** Pérdida energética mensual en %. `null` si no hay datos. */
    val energyMonthlyLossPercent: DoubleArray? = null,

    /** Matriz [12][24] con pérdida energética % por celda. `null` si no hay datos. */
    val energyHourlyMatrix: Array<DoubleArray>? = null,

    /**
     * POA anual potencial sin sombras, en kWh/m²/año.
     * Es la energía que recibiría el plano del panel si el horizonte
     * fuera perfectamente libre. `null` si no hay datos meteorológicos.
     */
    val annualEnergyPotentialKwhM2: Double? = null,

    /**
     * POA anual real con sombras del horizonte, en kWh/m²/año.
     * Es la energía efectivamente recibida considerando el bloqueo de
     * la componente directa por obstáculos. `null` si no hay datos.
     */
    val annualEnergyRealKwhM2: Double? = null,

    /** POA mensual potencial sin sombras, kWh/m²/mes. `null` si no hay datos. */
    val monthlyEnergyPotentialKwhM2: DoubleArray? = null,

    /** POA mensual real con sombras, kWh/m²/mes. `null` si no hay datos. */
    val monthlyEnergyRealKwhM2: DoubleArray? = null,

    // ============================================================
    // Config — para trazabilidad
    // ============================================================

    /** Config usado para el cálculo. */
    val config: ShadingLossConfig
) {

    /**
     * Pérdida absoluta geométrica = potential − real, en unidades de
     * [geometricAnnualPotential] (suma adimensional de cos(θ_inc)·Δt).
     */
    val geometricAnnualLost: Double
        get() = geometricAnnualPotential - geometricAnnualReal

    /**
     * Energía anual perdida por sombreado en kWh/m²/año.
     * `null` si no hay datos meteorológicos.
     */
    val annualEnergyLostKwhM2: Double?
        get() {
            val pot = annualEnergyPotentialKwhM2 ?: return null
            val real = annualEnergyRealKwhM2 ?: return null
            return pot - real
        }

    /**
     * `true` si este resultado contiene métricas energéticas (kWh)
     * además de las geométricas. `false` en modo `GEOMETRIC_ONLY` o si
     * la descarga de irradiancia falló y el motor degradó a geométrico.
     */
    val hasEnergyData: Boolean
        get() = energyAnnualLossPercent != null

    // ============================================================
    // equals / hashCode — manuales por presencia de Arrays
    //
    // Las data classes con Array usan identidad por default, lo que
    // rompe tests y comparaciones semánticas. Además los Arrays nullables
    // necesitan manejo explícito para no falsear igualdad cuando uno es
    // null y el otro vacío.
    // ============================================================

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ShadingLossResult) return false

        if (geometricAnnualLossPercent != other.geometricAnnualLossPercent) return false
        if (!geometricMonthlyLossPercent.contentEquals(other.geometricMonthlyLossPercent)) return false
        if (!geometricHourlyMatrix.contentDeepEquals(other.geometricHourlyMatrix)) return false
        if (geometricAnnualPotential != other.geometricAnnualPotential) return false
        if (geometricAnnualReal != other.geometricAnnualReal) return false

        if (energyAnnualLossPercent != other.energyAnnualLossPercent) return false
        if (!arraysContentEqualsNullable(energyMonthlyLossPercent, other.energyMonthlyLossPercent)) return false
        if (!arraysDeepEqualsNullable(energyHourlyMatrix, other.energyHourlyMatrix)) return false
        if (annualEnergyPotentialKwhM2 != other.annualEnergyPotentialKwhM2) return false
        if (annualEnergyRealKwhM2 != other.annualEnergyRealKwhM2) return false
        if (!arraysContentEqualsNullable(monthlyEnergyPotentialKwhM2, other.monthlyEnergyPotentialKwhM2)) return false
        if (!arraysContentEqualsNullable(monthlyEnergyRealKwhM2, other.monthlyEnergyRealKwhM2)) return false

        if (config != other.config) return false
        return true
    }

    override fun hashCode(): Int {
        var result = geometricAnnualLossPercent.hashCode()
        result = 31 * result + geometricMonthlyLossPercent.contentHashCode()
        result = 31 * result + geometricHourlyMatrix.contentDeepHashCode()
        result = 31 * result + geometricAnnualPotential.hashCode()
        result = 31 * result + geometricAnnualReal.hashCode()
        result = 31 * result + (energyAnnualLossPercent?.hashCode() ?: 0)
        result = 31 * result + (energyMonthlyLossPercent?.contentHashCode() ?: 0)
        result = 31 * result + (energyHourlyMatrix?.contentDeepHashCode() ?: 0)
        result = 31 * result + (annualEnergyPotentialKwhM2?.hashCode() ?: 0)
        result = 31 * result + (annualEnergyRealKwhM2?.hashCode() ?: 0)
        result = 31 * result + (monthlyEnergyPotentialKwhM2?.contentHashCode() ?: 0)
        result = 31 * result + (monthlyEnergyRealKwhM2?.contentHashCode() ?: 0)
        result = 31 * result + config.hashCode()
        return result
    }

    private fun arraysContentEqualsNullable(a: DoubleArray?, b: DoubleArray?): Boolean {
        if (a == null && b == null) return true
        if (a == null || b == null) return false
        return a.contentEquals(b)
    }

    private fun arraysDeepEqualsNullable(a: Array<DoubleArray>?, b: Array<DoubleArray>?): Boolean {
        if (a == null && b == null) return true
        if (a == null || b == null) return false
        return a.contentDeepEquals(b)
    }
}