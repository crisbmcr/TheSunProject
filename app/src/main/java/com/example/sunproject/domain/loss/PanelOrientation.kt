package com.example.sunproject.domain.loss

import kotlin.math.abs

/**
 * Orientación de un panel/colector solar fijo respecto del observador.
 *
 * Convenciones — mismas que SolarPosition para que la diferencia angular
 * (γ_sol − γ_panel) tenga sentido directo en la fórmula del coseno de
 * incidencia:
 *
 *  - azimuthDeg [0, 360): 0° = Norte, 90° = Este, 180° = Sur, 270° = Oeste.
 *    Convención CW mirando desde arriba (NOAA / norma astronómica).
 *
 *  - tiltDeg [0, 90]: 0° = panel acostado horizontal (cara hacia el cenit),
 *    90° = panel vertical (cara perpendicular al suelo).
 *
 * Nomenclatura Clement / Duffie:
 *   γ_panel = azimuthDeg
 *   β       = tiltDeg
 *
 * Nota: PVGIS usa una convención de azimut DISTINTA (0=Sur, 180=Norte).
 * Cualquier adapter que lea PVGIS debe convertir al sistema de esta clase
 * antes de instanciar PanelOrientation.
 */
data class PanelOrientation(
    val azimuthDeg: Double,
    val tiltDeg: Double
) {
    init {
        require(tiltDeg in 0.0..90.0) {
            "tiltDeg debe estar en [0, 90], era $tiltDeg"
        }
        require(azimuthDeg in 0.0..360.0) {
            "azimuthDeg debe estar en [0, 360], era $azimuthDeg"
        }
    }

    companion object {
        /**
         * Orientación de referencia óptima para una latitud dada, asumiendo
         * cielo despejado y sin obstáculos del horizonte.
         *
         *  - Hemisferio sur (lat < 0): γ = 0° (al Norte), β = |lat|.
         *  - Hemisferio norte (lat ≥ 0): γ = 180° (al Sur), β = |lat|.
         *  - Ecuador (lat ≈ 0): γ arbitrario, β = 0° (panel horizontal).
         *
         * La regla β = |lat| es la heurística clásica que maximiza la
         * captación anual promedio. En presencia de asimetrías climáticas
         * (monzón estacional) o de sombreado (perfil de horizonte no plano),
         * el óptimo verdadero se desvía de esta referencia — eso es lo que
         * resolverá el optimizador de orientación en una etapa posterior.
         */
        fun optimalFor(latitudeDeg: Double): PanelOrientation {
            val tilt = abs(latitudeDeg)
            val az = when {
                latitudeDeg < 0.0 -> 0.0
                latitudeDeg > 0.0 -> 180.0
                else -> 0.0
            }
            return PanelOrientation(azimuthDeg = az, tiltDeg = tilt)
        }
    }
}