package com.example.sunproject.domain.loss.irradiance

/**
 * Fuente de datos de irradiancia para el motor de pérdidas energéticas.
 *
 * Abstrae el origen de los datos (TMY descargado de PVGIS, dataset
 * sintético para tests, irradiancia constante para sanity checks, futuras
 * fuentes multi-año o satelitales en tiempo real). El motor
 * [com.example.sunproject.domain.loss.ShadingLossCalculator] consume
 * cualquier implementación de esta interfaz sin saber su origen.
 *
 * ## Contrato
 *
 * - `at(epochMillisUtc)` debe ser determinístico: la misma marca de
 *   tiempo siempre devuelve la misma muestra.
 * - Debe ser barato (O(1) o muy cerca). El motor llama esto ~35.000
 *   veces por cálculo y ya hace además NOAA + lookup del horizonte;
 *   no podemos permitirnos I/O o cómputo pesado por sample.
 * - Las implementaciones que cargan datos remotos (PVGIS) deben tener
 *   los datos ya en memoria antes de ser consumidas — la descarga es
 *   responsabilidad del DataSource correspondiente, no de esta interface.
 *
 * ## Thread safety
 *
 * Las implementaciones deben ser thread-safe **para lectura** después
 * de construidas. El motor las consulta desde un único Dispatchers.Default,
 * pero esto no se quiere garantizar como precondición porque puede haber
 * múltiples cálculos concurrentes (ej. usuario re-corre con otra orientación).
 *
 * Las implementaciones canónicas en el dominio
 * ([ConstantIrradianceSeries], [HourlyStepIrradianceSeries]) son inmutables
 * después de la construcción, así que esto se cumple naturalmente.
 *
 * ## Resolución temporal
 *
 * La interfaz no compromete una resolución específica. Una implementación
 * puede ser horaria (TMY), 15-minutal (futuras fuentes sub-horarias), o
 * incluso constante. El motor llama `at()` con su propio paso interno
 * (15 min por defecto) y la implementación decide cómo mapear ese
 * timestamp a la muestra que devuelve (step-wise, interpolación, etc.).
 */
interface IrradianceTimeSeries {

    /**
     * Devuelve la muestra de irradiancia válida para el timestamp dado.
     *
     * @param epochMillisUtc instante UTC en milisegundos epoch.
     * @return [IrradianceSample] correspondiente al instante.
     */
    fun at(epochMillisUtc: Long): IrradianceSample
}