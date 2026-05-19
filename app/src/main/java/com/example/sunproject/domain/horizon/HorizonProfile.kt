package com.example.sunproject.domain.horizon

/**
 * Perfil de obstrucción del horizonte derivado de un atlas equirectangular.
 * (... doc igual que antes ...)
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

data class DetectionParams(
    val aggregationStrategy: AggregationStrategy = AggregationStrategy.P75,
    val azimuthBands: Int = 12,
    val calibrationMaxAltitudeDeg: Float = 30f,
    val smoothingWindowDeg: Int = 5,
    val morphCloseKernelHeight: Int = 9,
    val morphCloseKernelWidth: Int = 1,

    /**
     * Saturación mínima (HSV S, rango 0-255) para considerar un pixel como
     * "cielo azul saturado". El cielo de Cauchari a 4024m tiene S típicamente
     * en 150-220 por baja humedad y poca dispersión Rayleigh. Estructuras
     * grises (concreto, metal, paneles solares) tienen S < 60. Pixels que
     * pasan el threshold de V pero no este threshold de S se rechazan como
     * cielo (probablemente son estructuras blancas/brillantes).
     */
    val saturationMinForSky: Int = 60,

    /**
     * Si V supera este valor, el pixel se acepta como cielo SIN importar S.
     * Esta "escape clause" es necesaria para el sol y su halo, que tienen
     * V≈255 pero S muy bajo (saturados blancos). Sin esta cláusula, el sol
     * se clasificaría como obstáculo. Cota conservadora: 240 evita aceptar
     * paredes blancas como cielo.
     */
    val veryHighVForcedSky: Int = 240,

    /**
     * Rango sano de fracción de cielo por banda. Después de calibrar Otsu en
     * cada banda y aplicar T_b al canal V de toda la banda, se cuenta qué
     * fracción quedó clasificada como cielo. Si esa fracción cae fuera del
     * rango sano, asumimos que Otsu falló y reemplazamos T_b por la mediana
     * de las bandas sanas.
     *
     * Justificación: en panoramas reales, cada banda debería tener un mínimo
     * de cielo (>20%) y nunca ser 100% cielo (siempre hay algo abajo).
     */
    val healthyBandSkyFractionMin: Float = 0.20f,
    val healthyBandSkyFractionMax: Float = 0.97f
) {
    init {
        require(azimuthBands > 0) { "azimuthBands must be positive" }
        require(calibrationMaxAltitudeDeg in 1f..90f) { "calibrationMaxAltitudeDeg must be in [1, 90]" }
        require(smoothingWindowDeg >= 0) { "smoothingWindowDeg must be >= 0" }
        require(morphCloseKernelHeight >= 1) { "morphCloseKernelHeight must be >= 1" }
        require(morphCloseKernelWidth >= 1) { "morphCloseKernelWidth must be >= 1" }
        require(saturationMinForSky in 0..255) { "saturationMinForSky must be in [0, 255]" }
        require(veryHighVForcedSky in 0..255) { "veryHighVForcedSky must be in [0, 255]" }
        require(healthyBandSkyFractionMin in 0f..1f) { "healthyBandSkyFractionMin in [0, 1]" }
        require(healthyBandSkyFractionMax in 0f..1f) { "healthyBandSkyFractionMax in [0, 1]" }
        require(healthyBandSkyFractionMin < healthyBandSkyFractionMax) {
            "min must be < max"
        }
    }

    companion object {
        val DEFAULT = DetectionParams()
    }
}

enum class AggregationStrategy {
    MEDIAN, P75, P90, MAX
}