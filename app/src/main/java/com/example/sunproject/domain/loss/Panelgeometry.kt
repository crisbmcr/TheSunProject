package com.example.sunproject.domain.loss

import com.example.sunproject.domain.solar.SolarPosition
import kotlin.math.cos
import kotlin.math.sin

/**
 * Geometría del ángulo de incidencia entre un rayo solar y la normal de un
 * panel inclinado.
 *
 * Fórmula de referencia (Duffie & Beckman, "Solar Engineering of Thermal
 * Processes", 4ª ed., cap. 1.6):
 *
 *   cos(θ_inc) = sin(α) · cos(β)
 *              + cos(α) · sin(β) · cos(γ_sol − γ_panel)
 *
 * donde:
 *   α        = altitud solar (SolarPosition.altitudeDeg)
 *   γ_sol    = azimut solar (SolarPosition.azimuthDeg, convención NOAA)
 *   β        = tilt del panel (PanelOrientation.tiltDeg)
 *   γ_panel  = azimut del panel (PanelOrientation.azimuthDeg)
 *
 * Interpretación de los dos términos:
 *   - El primero (sin α · cos β) es la captación "sol-arriba × panel-acostado":
 *     si el panel está horizontal (β=0), recibe el sol con peso sin(α).
 *   - El segundo (cos α · sin β · cos Δγ) es la corrección por tilt: si el
 *     panel está inclinado en la dirección del sol (Δγ pequeño), suma; si
 *     está inclinado en sentido opuesto (Δγ ~180°), resta.
 */
object PanelGeometry {

    /**
     * Coseno del ángulo de incidencia para un sol y un panel dados.
     *
     * @return cos(θ_inc) en [-1, 1]:
     *   - 1   = sol perpendicular al panel (captación máxima)
     *   - 0   = sol rasante (rayo paralelo a la cara del panel)
     *   - <0  = sol pegando por detrás del panel — sin contribución.
     *           Los callers deben tratar valores ≤0 como cero.
     *
     * Si el sol está bajo el horizonte (belowHorizon=true), devuelve 0
     * directamente como cortocircuito.
     */
    fun cosIncidence(
        sun: SolarPosition,
        panel: PanelOrientation
    ): Double {
        if (sun.belowHorizon) return 0.0

        val alphaRad = Math.toRadians(sun.altitudeDeg)
        val betaRad = Math.toRadians(panel.tiltDeg)
        val deltaAzRad = Math.toRadians(sun.azimuthDeg - panel.azimuthDeg)

        val cosInc = sin(alphaRad) * cos(betaRad) +
                cos(alphaRad) * sin(betaRad) * cos(deltaAzRad)

        return cosInc.coerceIn(-1.0, 1.0)
    }
}