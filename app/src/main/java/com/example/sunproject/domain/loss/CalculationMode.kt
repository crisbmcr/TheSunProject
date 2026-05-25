package com.example.sunproject.domain.loss

/**
 * Modo de cálculo de pérdidas por sombreado.
 *
 * Determina qué métricas se computan y, en consecuencia, qué dependencias
 * externas requiere el cálculo.
 *
 * - [GEOMETRIC_ONLY]: solo % pérdida geométrica anual ponderada por
 *   cos(θ_inc). No requiere datos meteorológicos, funciona 100% offline.
 *   Es el modo de C.1 y el default histórico.
 *
 * - [ENERGY_FULL]: además del geométrico, computa % pérdida energética
 *   ponderada por la POA real
 *      POA = DNI × cos(θ_inc) + DHI × FV_sky + GHI × albedo × FV_ground
 *   y los kWh/m² absolutos. Requiere una fuente de irradiancia
 *   (IrradianceTimeSeries) — por defecto PVGIS-ERA5 TMY descargado del
 *   sitio. La primera vez que se calcula en una ubicación nueva requiere
 *   internet para descargar el TMY (~50 KB); después queda cacheado en
 *   local y los cálculos posteriores son offline.
 *
 * Estado de implementación por fase:
 *  - GEOMETRIC_ONLY: disponible desde C.1.
 *  - ENERGY_FULL:    disponible desde C.2.2. En C.2.0 y C.2.1 el motor
 *    acepta este modo pero ignora su efecto (devuelve solo el geométrico
 *    y loguea un warning para señalar que la dependencia aún no está
 *    conectada).
 */
enum class CalculationMode {
    GEOMETRIC_ONLY,
    ENERGY_FULL
}