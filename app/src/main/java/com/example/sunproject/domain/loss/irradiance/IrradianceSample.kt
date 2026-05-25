package com.example.sunproject.domain.loss.irradiance

/**
 * Muestra puntual de irradiancia solar sobre el sitio.
 *
 * Representa los tres componentes que necesita la fórmula POA (Plane of
 * Array, Duffie & Beckman cap. 2.15) más metadatos opcionales que pueden
 * usarse en versiones futuras del motor (corrección térmica, viento para
 * IAM, etc.).
 *
 * ## Unidades — IMPORTANTE
 *
 * Todos los componentes radiativos están en **W/m² (instantáneo)**, no en
 * Wh/m². La integración temporal (multiplicación por Δt) la hace el motor.
 * PVGIS reporta valores horarios "instantáneos al timestamp"; este modelo
 * los toma tal cual.
 *
 * ## Convenciones
 *
 * - dniWm2  >= 0: directa normal. Es la potencia sobre un plano siempre
 *   perpendicular al sol. Se "proyecta" al panel multiplicando por cos(θ_inc).
 *
 * - dhiWm2  >= 0: difusa horizontal. Es la componente del cielo dispersa
 *   recibida en un plano horizontal. Se "proyecta" al panel mediante el
 *   factor de vista del cielo FV_sky = (1 + cos(β)) / 2.
 *
 * - ghiWm2  >= 0: global horizontal. Suma de directa proyectada al horizonte
 *   más difusa: GHI = DNI·cos(θ_z) + DHI. Se usa para computar la componente
 *   reflejada del suelo: GHI · albedo · FV_ground, con FV_ground = (1 − cos(β)) / 2.
 *
 * - airTempC: temperatura ambiente a 2m (°C). PVGIS la entrega como T2m.
 *   No usada en C.2 pero se preserva en el modelo para futuras correcciones
 *   térmicas (NOCT, Sandia, King).
 *
 * - windSpeedMs: velocidad de viento a 10m (m/s). PVGIS la entrega como
 *   WS10m. No usada en C.2; reservada para corrección térmica avanzada
 *   (enfriamiento convectivo del módulo).
 */
data class IrradianceSample(

    /** Direct Normal Irradiance (W/m²). >= 0. */
    val dniWm2: Double,

    /** Diffuse Horizontal Irradiance (W/m²). >= 0. */
    val dhiWm2: Double,

    /** Global Horizontal Irradiance (W/m²). >= 0. */
    val ghiWm2: Double,

    /** Temperatura ambiente a 2m (°C). null si no disponible. */
    val airTempC: Double? = null,

    /** Velocidad de viento a 10m (m/s). null si no disponible. */
    val windSpeedMs: Double? = null
) {
    init {
        require(dniWm2 >= 0.0) { "dniWm2 no puede ser negativo: $dniWm2" }
        require(dhiWm2 >= 0.0) { "dhiWm2 no puede ser negativo: $dhiWm2" }
        require(ghiWm2 >= 0.0) { "ghiWm2 no puede ser negativo: $ghiWm2" }
        windSpeedMs?.let {
            require(it >= 0.0) { "windSpeedMs no puede ser negativo: $it" }
        }
        // airTempC no se valida: en altitud o polos puede ser muy negativa.
    }

    companion object {
        /**
         * Muestra de noche (todo cero). Útil para tests y para representar
         * timestamps donde el sol está debajo del horizonte sin tener que
         * decidir un valor sintético.
         */
        val NIGHT = IrradianceSample(dniWm2 = 0.0, dhiWm2 = 0.0, ghiWm2 = 0.0)
    }
}