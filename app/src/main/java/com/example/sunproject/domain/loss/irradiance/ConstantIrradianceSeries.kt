package com.example.sunproject.domain.loss.irradiance

/**
 * Fuente de irradiancia constante. Útil para tests, sanity checks y
 * para reproducir el comportamiento geométrico de C.1 dentro del flujo
 * energético (con limitaciones, ver abajo).
 *
 * Devuelve la misma [IrradianceSample] para cualquier timestamp.
 *
 * ## Caso "modo geométrico equivalente"
 *
 * Con `IrradianceSample(dniWm2=1, dhiWm2=0, ghiWm2=1)` y `albedo=0`, la
 * fórmula POA se reduce a:
 *
 *     POA = 1 · cos(θ_inc) + 0 · FV_sky + 1 · 0 · FV_ground
 *         = cos(θ_inc)
 *
 * Que es exactamente el integrando de C.1 (modo GEOMETRIC_ONLY). Esto
 * permite verificar en C.2.2 que el motor energético, alimentado con
 * esta fuente, devuelva pérdidas % numéricamente idénticas al motor
 * geométrico — esa es la regresión de seguridad de la nueva ruta de
 * código.
 *
 * Si `albedo > 0`, el integrando energético tiene un offset constante
 * (`albedo · FV_ground`) respecto al geométrico, y los % pueden diferir
 * ligeramente (no en magnitudes, sí en decimales). Documentado.
 *
 * ## Caso "irradiancia plana"
 *
 * Para tests pedagógicos podés pasar valores realistas constantes
 * (ej. DNI=800, DHI=100, GHI=900) y obtener una integración energética
 * sobre todo el año con clima invariante. No representa ningún sitio
 * real pero sirve para validar la mecánica del motor sin depender de
 * PVGIS.
 */
class ConstantIrradianceSeries(
    private val sample: IrradianceSample = IrradianceSample(
        dniWm2 = 1.0,
        dhiWm2 = 0.0,
        ghiWm2 = 1.0
    )
) : IrradianceTimeSeries {

    override fun at(epochMillisUtc: Long): IrradianceSample = sample

    /** Acceso público a la muestra que devuelve esta serie. Inmutable. */
    val constantSample: IrradianceSample get() = sample
}