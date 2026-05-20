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
 *  - Azimut en grados [0, 360), CW desde el Norte. Misma convención que NOAA.
 *  - Buckets uniformes en azimut. azimuthBuckets=360 → un bucket por grado.
 *  - Altitudes en grados [0, 90]. 0° = horizonte libre.
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
        const val CURRENT_SCHEMA_VERSION = 2  // bump por cambio de algoritmo
        const val DEFAULT_AZIMUTH_BUCKETS = 360
    }
}

/**
 * Parámetros del algoritmo de detección. Con el cambio a segmentación
 * semántica vía TFLite, los parámetros se simplificaron mucho — la mayor
 * parte del trabajo lo hace el modelo CNN.
 */
data class DetectionParams(
    /**
     * Estrategia de agregación columna → bucket de azimut. Cada bucket de
     * 1° recibe varias columnas del atlas (10 si atlas=3600). Esta estrategia
     * decide cómo combinarlas.
     */
    val aggregationStrategy: AggregationStrategy = AggregationStrategy.P75,

    /**
     * Ventana en grados del suavizado circular (moving average) sobre el
     * perfil ya agregado por bucket. 0 desactiva el suavizado.
     */
    val smoothingWindowDeg: Int = 5
) {
    init {
        require(smoothingWindowDeg >= 0) { "smoothingWindowDeg must be >= 0" }
    }

    companion object {
        val DEFAULT = DetectionParams()
    }
}

enum class AggregationStrategy {
    MEDIAN, P75, P90, MAX
}