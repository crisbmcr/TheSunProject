package com.example.sunproject.domain.loss

/**
 * Modelo de transposición de difusa para el cálculo POA.
 *
 * La irradiancia difusa horizontal (DHI) llega al panel desde todo el
 * domo celeste. Para "proyectarla" sobre el plano inclinado del panel
 * existen distintos modelos físicos con distintos compromisos entre
 * exactitud, complejidad y robustez.
 *
 * Solo aplica cuando [CalculationMode.ENERGY_FULL] está activo. El modo
 * geométrico no usa irradiancia, así que este parámetro no tiene efecto.
 *
 * ## Modelos disponibles
 *
 * ### [ISOTROPIC] — Liu-Jordan 1963 (default histórico)
 *
 *     POA_dif = DHI · (1 + cos(β)) / 2
 *
 * Asume que la difusa viene uniformemente del domo celeste (modelo de
 * cielo gris). Es el modelo más simple y el de menor complejidad
 * matemática. Funciona muy bien en:
 *   - Cielos cubiertos uniformes (donde la hipótesis isotrópica se
 *     cumple casi exactamente).
 *   - Sitios de cielo muy limpio con DNI alta, donde la difusa es una
 *     fracción chica del total y su modelado preciso importa poco.
 *
 * En el sitio de validación primario del proyecto (Cauchari, Argentina,
 * Puna alta, cielo limpio), coincide con PVGIS PVcalc dentro del 0.1%.
 *
 * ### [HAY_DAVIES] — Hay-Davies 1980 (anisotrópico, recomendado)
 *
 *     A_i      = DNI / I_0n                   (índice de anisotropía)
 *     r_b      = max(0, cos(θ_inc)) / cos(θ_z)
 *     POA_dif  = DHI · [A_i · r_b + (1 - A_i) · (1 + cos(β)) / 2]
 *
 * Divide la difusa en una componente "circumsolar" (que llega de cerca
 * del disco solar y se trata como directa, proyectada con r_b) y una
 * componente isotrópica residual. Cuando A_i = 0 (sin DNI, cielo
 * totalmente cubierto) el modelo se reduce exactamente al isotrópico.
 *
 * Validación multi-sitio (N=10, latitudes -64° a +64°, fracciones
 * difusas 14% a 60%) contra PVGIS PVcalc mostró que Hay-Davies es el
 * modelo de mejor compromiso global:
 *
 *   Modelo       MBE       MAE       RMSE
 *   Isotrópico   -4.13%    4.59%     6.92%
 *   Hay-Davies   -2.18%    3.82%     5.74%  ← mejor MBE y MAE
 *   Perez        -0.52%    3.68%     5.36%  ← mejor MBE en N=10, pero
 *                                            sobreestima en cielo limpio
 *
 * En el subconjunto de sitios "limpios" (cielo de baja a media fracción
 * difusa, excluyendo outliers de dataset), Hay-Davies da MAE = 1.99%
 * vs Isotrópico 2.22% y Perez 2.25%. Por eso es la elección por defecto
 * para uso global del motor.
 *
 * ## Bloqueo de la componente circumsolar
 *
 * En este motor, las componentes difusa total y reflejada NO se bloquean
 * por el horizonte (limitación documentada v2). Aunque físicamente la
 * fracción circumsolar de Hay-Davies "viene desde la dirección del sol"
 * y debería bloquearse cuando el sol está obstruido, en esta versión
 * mantenemos la simplificación isotrópica del bloqueo para consistencia
 * con la implementación canónica de pvlib y otras herramientas estándar
 * (PVsyst, SAM). El refinamiento del Sky View Factor obstruido queda
 * planificado para v3 del motor.
 */
enum class DiffuseModel {
    ISOTROPIC,
    HAY_DAVIES
}