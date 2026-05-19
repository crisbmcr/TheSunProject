package com.example.sunproject.domain.horizon

/**
 * Perfil de obstrucción del horizonte derivado de un atlas equirectangular.
 *
 * Representa la función az → alt_min, donde alt_min es la altitud mínima
 * (grados sobre el horizonte 0°) por encima de la cual el cielo está libre
 * en ese azimut. Lo que esté por debajo del perfil se considera obstáculo
 * (terreno, vegetación, estructuras).
 *
 * Convenciones:
 *  - Azimut en grados [0, 360), CW desde el Norte. Misma convención que
 *    NOAA / SolarGeometry: 0=N, 90=E, 180=S, 270=O.
 *  - Buckets uniformes en azimut. azimuthBuckets=360 → un bucket por grado.
 *    Índice 0 corresponde a az ∈ [0°, 360/buckets°).
 *  - Altitudes en grados [0, 90]. Un valor de 0° significa "no hay obstáculo
 *    en ese azimut" (horizonte libre). Un valor de 90° significaría obstáculo
 *    en todas las altitudes — caso degenerado, no debería pasar.
 *
 * El perfil es derivado del atlas, no parte de la captura. Se puede regenerar
 * en cualquier momento desde el mismo atlas con distintos DetectionParams.
 */
data class HorizonProfile(
    val schemaVersion: Int,
    val sessionId: String,
    val createdAtUtcMs: Long,
    val sourceAtlasName: String,
    val azimuthBuckets: Int,
    val altByAzimuthDeg: List<Float>,
    val detectionParams: DetectionParams
) {
    init {
        require(altByAzimuthDeg.size == azimuthBuckets) {
            "altByAzimuthDeg size (${altByAzimuthDeg.size}) != azimuthBuckets ($azimuthBuckets)"
        }
        require(azimuthBuckets > 0) {
            "azimuthBuckets must be positive, got $azimuthBuckets"
        }
    }

    /**
     * Devuelve la altitud mínima del horizonte para un azimut dado.
     *
     * El azimut se normaliza a [0, 360). Soporta tanto el rango NOAA [0, 360)
     * como el rango AtlasMath [-180, 180].
     *
     * Pensado para integrar contra trayectorias solares: para cada SolarPosition,
     * comparar su altitudeDeg con altAtAzimuthDeg(azimuthDeg) y si la altitud
     * del sol es menor, el sol está obstruido.
     */
    fun altAtAzimuthDeg(azimuthDeg: Float): Float {
        var az = azimuthDeg % 360f
        if (az < 0f) az += 360f
        val bucketWidth = 360f / azimuthBuckets
        val idx = ((az / bucketWidth).toInt()) % azimuthBuckets
        return altByAzimuthDeg[idx]
    }

    companion object {
        const val CURRENT_SCHEMA_VERSION = 1
        const val DEFAULT_AZIMUTH_BUCKETS = 360
    }
}

/**
 * Parámetros del algoritmo de detección. Se persisten junto al perfil para
 * trazabilidad — si después dudás de un resultado, podés ver con qué config
 * fue generado.
 *
 * Todos los valores tienen default razonable para v1; el constructor sin args
 * (DetectionParams()) corresponde a la configuración recomendada.
 */
data class DetectionParams(
    /**
     * Estrategia de agregación columna → bucket. Cada bucket de 1° de azimut
     * recibe varias columnas del atlas (típicamente 10 columnas si atlas=3600
     * y buckets=360). Esta estrategia decide cómo combinarlas.
     *
     * MEDIAN: robusta a outliers, descarta estructuras delgadas (cables, postes).
     * P75/P90: percentiles, intermedio entre mediana y máximo.
     * MAX: conservador para sombreado, sobreestima por cualquier columna ruidosa.
     *
     * Default MEDIAN para v1. Si en validación se ve que se pierden
     * obstrucciones reales pero finas (mástil, antena), subir a P75 o P90.
     */
    val aggregationStrategy: AggregationStrategy = AggregationStrategy.MEDIAN,

    /**
     * Cantidad de bandas verticales para calibración adaptativa de Otsu.
     * 12 bandas = una cada 30° de azimut. Cada banda calcula su propio
     * threshold sobre las filas calibrationMaxAltitudeDeg para evitar
     * contaminación con el cielo cenital (muy oscuro a alta altitud).
     */
    val azimuthBands: Int = 12,

    /**
     * Altitud máxima (deg) que se usa para CALIBRAR el threshold de Otsu
     * en cada banda. El threshold calculado se APLICA a toda la altura
     * del atlas, no solo a esta franja.
     *
     * Justificación: a alta altitud (>60°) el cielo es muy oscuro por
     * baja dispersión Rayleigh y contamina la distribución bimodal que
     * Otsu necesita. Calibrar solo abajo da un threshold limpio.
     */
    val calibrationMaxAltitudeDeg: Float = 60f,

    /**
     * Ventana en grados del suavizado circular (moving average) que se
     * aplica al perfil ya agregado por bucket. 0 desactiva el suavizado.
     */
    val smoothingWindowDeg: Int = 5,

    /**
     * Tamaño del kernel de cierre morfológico aplicado a la máscara binaria
     * cielo/suelo antes de buscar la frontera. Kernel vertical más alto que
     * ancho para preservar estructuras delgadas (postes) y rellenar huecos
     * pequeños del cielo (nubes ralas, ruido).
     */
    val morphCloseKernelHeight: Int = 3,
    val morphCloseKernelWidth: Int = 1
) {
    init {
        require(azimuthBands > 0) { "azimuthBands must be positive" }
        require(calibrationMaxAltitudeDeg in 1f..90f) {
            "calibrationMaxAltitudeDeg must be in [1, 90]"
        }
        require(smoothingWindowDeg >= 0) { "smoothingWindowDeg must be >= 0" }
        require(morphCloseKernelHeight >= 1) { "morphCloseKernelHeight must be >= 1" }
        require(morphCloseKernelWidth >= 1) { "morphCloseKernelWidth must be >= 1" }
    }

    companion object {
        val DEFAULT = DetectionParams()
    }
}

enum class AggregationStrategy {
    MEDIAN, P75, P90, MAX
}