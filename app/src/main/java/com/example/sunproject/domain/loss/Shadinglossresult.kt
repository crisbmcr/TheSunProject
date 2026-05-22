package com.example.sunproject.domain.loss

/**
 * Resultado de un cálculo de pérdidas por sombreado.
 *
 * Convenciones para todos los porcentajes:
 *   - Rango [0, 100]
 *   - 0%   = sin pérdidas (panel recibe todo el potencial geométrico)
 *   - 100% = pérdida total (el panel no recibe nada por sombras)
 *
 * Los acumuladores [annualPotentialIntegral] y [annualRealIntegral] son
 * sumas adimensionales en v1 (geométrico puro): cos(θ_inc) · Δt[h] sobre
 * todos los instantes válidos del año. NO representan kWh/m²; para eso
 * hay que multiplicar por una irradiancia (eso entra en C.2/v3).
 *
 * Son útiles para:
 *   - Debug interno (comparar configuraciones).
 *   - Calcular la "ganancia angular promedio" si el caller la necesita.
 */
data class ShadingLossResult(

    /** Pérdida anual total en %. */
    val annualLossPercent: Double,

    /**
     * Pérdida mensual en %. Índice 0 = enero, 11 = diciembre.
     * Cada elemento puede ser 0 (si no había nada que captar en ese mes
     * para esta orientación) o un valor en (0, 100].
     */
    val monthlyLossPercent: DoubleArray,

    /**
     * Matriz [12 meses][24 horas locales] con la pérdida % en cada celda.
     * monthlyLossPercent[m] es el promedio ponderado de hourlyMatrix[m][*].
     *
     * Horas locales se determinan aplicando [ShadingLossConfig.timezoneOffsetHours]
     * al timestamp UTC.
     */
    val hourlyMatrix: Array<DoubleArray>,

    /**
     * Integral anual del potencial geométrico (sin sombras).
     * v1: Σ cos(θ_inc) · Δt[h] sobre todo el año.
     * v3 (con irradiancia): Σ POA_potencial · Δt[h] en Wh/m².
     */
    val annualPotentialIntegral: Double,

    /**
     * Integral anual de la captación real (con sombras).
     * Misma unidad que [annualPotentialIntegral].
     */
    val annualRealIntegral: Double,

    /**
     * Config usado para el cálculo. Útil para reportes y trazabilidad.
     */
    val config: ShadingLossConfig
) {

    /**
     * Cantidad absoluta perdida = potential − real.
     * En las mismas unidades que los integrales.
     */
    val annualLostIntegral: Double
        get() = annualPotentialIntegral - annualRealIntegral

    // equals/hashCode manuales: las data classes con Array usan identidad
    // por default, lo que rompe tests y comparaciones semánticas.
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ShadingLossResult) return false
        if (annualLossPercent != other.annualLossPercent) return false
        if (!monthlyLossPercent.contentEquals(other.monthlyLossPercent)) return false
        if (!hourlyMatrix.contentDeepEquals(other.hourlyMatrix)) return false
        if (annualPotentialIntegral != other.annualPotentialIntegral) return false
        if (annualRealIntegral != other.annualRealIntegral) return false
        if (config != other.config) return false
        return true
    }

    override fun hashCode(): Int {
        var result = annualLossPercent.hashCode()
        result = 31 * result + monthlyLossPercent.contentHashCode()
        result = 31 * result + hourlyMatrix.contentDeepHashCode()
        result = 31 * result + annualPotentialIntegral.hashCode()
        result = 31 * result + annualRealIntegral.hashCode()
        result = 31 * result + config.hashCode()
        return result
    }
}