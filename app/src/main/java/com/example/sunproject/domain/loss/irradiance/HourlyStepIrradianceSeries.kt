package com.example.sunproject.domain.loss.irradiance

/**
 * Fuente de irradiancia construida sobre una serie horaria de 8760 muestras
 * (un año no bisiesto), con interpolación step-wise para sub-pasos.
 *
 * Es la implementación principal usada por el motor en C.2.2+ cuando se
 * alimenta con datos PVGIS TMY (Typical Meteorological Year).
 *
 * ## Modelo de discretización
 *
 * El motor itera con paso de 15 minutos (4 sub-pasos por hora). Esta
 * implementación devuelve **la misma muestra para los 4 sub-pasos** que
 * caen dentro de la misma hora del año (Opción A del handoff C.2).
 *
 * Por ejemplo, para hour-index = 13 (segundo bloque del día):
 *   - epoch 13:00:00 UTC → hour-index 13 → sample[13]
 *   - epoch 13:15:00 UTC → hour-index 13 → sample[13]
 *   - epoch 13:30:00 UTC → hour-index 13 → sample[13]
 *   - epoch 13:45:00 UTC → hour-index 13 → sample[13]
 *   - epoch 14:00:00 UTC → hour-index 14 → sample[14]
 *
 * Justificación: el detalle fino del sombreado (geometría sol-horizonte
 * cambia segundo a segundo) es donde aporta valor el sistema; la
 * irradiancia varía suavemente intra-hora y mantenerla constante 4
 * sub-pasos introduce un error muy menor (<1%) comparado con el error
 * intrínseco del TMY como representación de un año típico (~3% IAV).
 *
 * Una implementación con interpolación lineal entre horas se podría
 * agregar como [HourlyInterpolatedIrradianceSeries] en una fase futura
 * si la diferencia se midiese significativa para algún caso de uso.
 *
 * ## Mapeo timestamp → índice
 *
 * El motor itera el año entero en UTC. Para cada `epochMillisUtc`:
 *
 *     hourIndex = ((epochMillisUtc - yearStartEpochUtc) / 3_600_000).toInt()
 *
 * `yearStartEpochUtc` debe ser el 1-ene-00:00 UTC del año del cálculo.
 * Esta clase es **agnóstica al año real de cada muestra del TMY**: el
 * TMY es una secuencia ordenada de 8760 valores donde el índice 0 = primera
 * hora de enero, índice 8759 = última hora de diciembre, sin importar de
 * qué año "real" venga cada mes (PVGIS construye el TMY combinando meses
 * de distintos años; el lookup acá usa el año del config como referencia
 * temporal, no los años originales de los meses).
 *
 * ## Manejo de bordes
 *
 * `coerceIn(0, 8759)` defensivo. El motor solo debe llamar dentro del
 * rango pero esto evita crashes si algún cálculo deriva microsegundos
 * fuera del intervalo por error de redondeo.
 *
 * ## Costo computacional
 *
 * O(1): una resta, una división entera, un acceso a array. Negligible
 * frente a NOAA (que es la operación pesada por sample en el motor).
 */
class HourlyStepIrradianceSeries(
    private val hourlyData: Array<IrradianceSample>,
    private val yearStartEpochUtc: Long
) : IrradianceTimeSeries {

    init {
        require(hourlyData.size == HOURS_PER_NON_LEAP_YEAR) {
            "hourlyData debe tener exactamente $HOURS_PER_NON_LEAP_YEAR muestras " +
                    "(un año no bisiesto), recibí ${hourlyData.size}"
        }
    }

    override fun at(epochMillisUtc: Long): IrradianceSample {
        val deltaMs = epochMillisUtc - yearStartEpochUtc
        val rawHourIndex = (deltaMs / MILLIS_PER_HOUR).toInt()
        val safeIndex = rawHourIndex.coerceIn(0, HOURS_PER_NON_LEAP_YEAR - 1)
        return hourlyData[safeIndex]
    }

    /** Para diagnóstico: cantidad de muestras en la serie. */
    val sampleCount: Int get() = hourlyData.size

    /** Para diagnóstico: epoch del primer instante representado. */
    val startEpochUtc: Long get() = yearStartEpochUtc

    companion object {
        const val HOURS_PER_NON_LEAP_YEAR = 8760
        private const val MILLIS_PER_HOUR = 3_600_000L
    }
}