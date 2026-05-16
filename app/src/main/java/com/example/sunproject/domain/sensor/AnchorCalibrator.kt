package com.example.sunproject.domain.sensor

import android.util.Log
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Calibrador del anchor true-N (displayNorthOffsetDeg).
 *
 * Reemplaza la inicialización "primer sample válido" por un esquema basado
 * en estabilidad del lote: acumula samples de (absoluteYawDeg - gameYawDeg)
 * en un ring buffer, calcula el spread (peak-to-peak después de filtrar
 * outliers), y solo da por inicializado el anchor cuando el spread se
 * mantiene bajo `targetSpreadDeg` durante `requiredStableMs` consecutivos.
 *
 * Esto reproduce lo que Sunshine Compass logra implícitamente con su
 * lectura continua del mag: la app espera a que el magnetómetro esté
 * realmente estable antes de "anclar".
 *
 * Si pasa el timeout sin lograrlo, se permite anclar con el mejor lote
 * disponible y un warning explícito (precisión estimada degradada).
 *
 * Convenciones angulares:
 *   - Todos los grados, todos en [0, 360).
 *   - El promedio circular se computa con sin/cos para evitar el bug
 *     359°↔1°.
 *   - Las diferencias entre samples se evalúan vía circularDist para
 *     respetar la topología del círculo.
 */
class AnchorCalibrator(
    private val capacity: Int = 150,                  // ~3s a 50Hz
    private val targetSpreadDeg: Float = 2.0f,
    private val requiredStableMs: Long = 2000L,
    private val timeoutMs: Long = 30_000L
) {

    enum class Phase {
        WAITING_FOR_LOCATION,   // sin GPS todavía
        WAITING_FOR_MAG,        // mag en UNRELIABLE (0)
        COLLECTING,             // recolectando, spread alto
        STABILIZING,            // spread bajo pero no sostenido todavía
        READY,                  // anchor listo (criterio normal)
        READY_DEGRADED          // anchor listo via timeout (warning)
    }

    /** Snapshot del estado para que la UI lo consuma sin tocar internos. */
    data class Status(
        val phase: Phase,
        val sampleCount: Int,
        val spreadDeg: Float,            // spread actual del lote filtrado
        val anchorOffsetDeg: Float?,     // null si phase < READY
        val stableForMs: Long,           // ms acumulados con spread <= target
        val elapsedMs: Long,             // ms desde primer sample
        val magAccuracy: Int,
        val precisionEstimateDeg: Float  // estimación de precisión final
    )

    private val samples = FloatArray(capacity)  // (absYaw - gameYaw) normalizado [0,360)
    private var head = 0
    private var size = 0

    private var startMs: Long = 0L
    private var stableStartMs: Long = 0L
    private var lastMagAccuracy: Int = 0

    @Volatile private var cachedAnchorDeg: Float? = null
    @Volatile private var cachedDegraded: Boolean = false

    private var cachedSpreadDeg: Float = 360f
    private var cachedPrecisionDeg: Float = 360f

    /**
     * Push un nuevo sample. Devuelve true si después de este push el
     * anchor pasó a estado READY/READY_DEGRADED (transición a listo).
     * Si ya estaba listo, devuelve false (no re-anclar).
     */
    fun push(
        absYawDeg: Float,
        gameYawDeg: Float,
        magAccuracy: Int,
        hasLocation: Boolean,
        nowMs: Long
    ): Boolean {
        lastMagAccuracy = magAccuracy

        if (!hasLocation) return false
        if (magAccuracy <= 0) {
            // UNRELIABLE: no acumular, no avanzar. El push se ignora.
            return false
        }

        if (startMs == 0L) startMs = nowMs

        // Si ya está listo, no re-acumular. El anchor queda fijo.
        if (cachedAnchorDeg != null) return false

        val delta = normalize360(absYawDeg - gameYawDeg)
        samples[head] = delta
        head = (head + 1) % capacity
        if (size < capacity) size++

        // Necesitamos al menos N/3 samples para evaluar spread con confianza.
        if (size < capacity / 3) return false

        val (spread, mean, precisionEstimate) = computeStats()
        cachedSpreadDeg = spread
        cachedPrecisionDeg = precisionEstimate

        val elapsedMs = nowMs - startMs

        if (spread <= targetSpreadDeg) {
            if (stableStartMs == 0L) stableStartMs = nowMs
            val stableForMs = nowMs - stableStartMs
            if (stableForMs >= requiredStableMs) {
                cachedAnchorDeg = mean
                cachedDegraded = false
                Log.i(
                    "AnchorCalibrator",
                    "READY mean=${"%.2f".format(mean)}° " +
                            "spread=${"%.2f".format(spread)}° " +
                            "precision=${"%.2f".format(precisionEstimate)}° " +
                            "elapsedMs=$elapsedMs magAcc=$magAccuracy"
                )
                return true
            }
        } else {
            // Spread volvió a ser alto, reiniciar el contador de estabilidad.
            stableStartMs = 0L
        }

        // Fallback: timeout. Anclar con lo que haya con warning.
        if (elapsedMs >= timeoutMs) {
            cachedAnchorDeg = mean
            cachedDegraded = true
            Log.w(
                "AnchorCalibrator",
                "READY_DEGRADED (timeout) mean=${"%.2f".format(mean)}° " +
                        "spread=${"%.2f".format(spread)}° " +
                        "precision=${"%.2f".format(precisionEstimate)}° " +
                        "elapsedMs=$elapsedMs magAcc=$magAccuracy"
            )
            return true
        }

        return false
    }

    /** Snapshot actual para la UI. Llamar a sensor rate sin problema. */
    fun status(nowMs: Long): Status {
        val anchor = cachedAnchorDeg
        val phase = when {
            anchor != null && cachedDegraded -> Phase.READY_DEGRADED
            anchor != null -> Phase.READY
            startMs == 0L && lastMagAccuracy == 0 -> Phase.WAITING_FOR_MAG
            startMs == 0L -> Phase.WAITING_FOR_LOCATION
            cachedSpreadDeg <= targetSpreadDeg -> Phase.STABILIZING
            else -> Phase.COLLECTING
        }
        val stableForMs = if (stableStartMs > 0L && anchor == null) nowMs - stableStartMs else 0L
        val elapsedMs = if (startMs > 0L) nowMs - startMs else 0L
        return Status(
            phase = phase,
            sampleCount = size,
            spreadDeg = cachedSpreadDeg,
            anchorOffsetDeg = anchor,
            stableForMs = stableForMs,
            elapsedMs = elapsedMs,
            magAccuracy = lastMagAccuracy,
            precisionEstimateDeg = cachedPrecisionDeg
        )
    }

    fun isReady(): Boolean = cachedAnchorDeg != null

    fun anchorOrNull(): Float? = cachedAnchorDeg

    fun isDegraded(): Boolean = cachedDegraded

    /**
     * Reset completo. Útil si el usuario quiere recalibrar manualmente
     * o si detectamos un evento que invalida la calibración.
     */
    fun reset() {
        head = 0
        size = 0
        startMs = 0L
        stableStartMs = 0L
        cachedAnchorDeg = null
        cachedDegraded = false
        cachedSpreadDeg = 360f
        cachedPrecisionDeg = 360f
        Log.i("AnchorCalibrator", "reset")
    }

    /**
     * Computa estadísticas del lote filtrado por outliers.
     * Devuelve (spread, mean, precisionEstimate). Todos en grados.
     *
     * - mean: promedio circular vía sin/cos sobre samples filtrados.
     * - spread: peak-to-peak de las distancias circulares al mean.
     * - precisionEstimate: spread / sqrt(N), aproximación al stderr.
     */
    private fun computeStats(): Triple<Float, Float, Float> {
        if (size == 0) return Triple(360f, 0f, 360f)

        // Primer pasada: promedio circular crudo.
        var sumSin = 0.0
        var sumCos = 0.0
        for (i in 0 until size) {
            val rad = Math.toRadians(samples[i].toDouble())
            sumSin += sin(rad)
            sumCos += cos(rad)
        }
        val crudeMeanRad = atan2(sumSin, sumCos)
        val crudeMeanDeg = (Math.toDegrees(crudeMeanRad).toFloat() + 360f) % 360f

        // Segunda pasada: filtrar outliers (|sample - mean| > 3 * targetSpread).
        // En lotes mayormente buenos, esto elimina picos espurios.
        val maxDevDeg = targetSpreadDeg * 3f
        var sumSinF = 0.0
        var sumCosF = 0.0
        var countF = 0
        var maxDevSeen = 0f
        var minDevSeen = 0f
        for (i in 0 until size) {
            val dev = circularDist(samples[i], crudeMeanDeg)
            if (dev <= maxDevDeg) {
                val rad = Math.toRadians(samples[i].toDouble())
                sumSinF += sin(rad)
                sumCosF += cos(rad)
                countF++
                val signedDev = signedCircularDist(samples[i], crudeMeanDeg)
                if (signedDev > maxDevSeen) maxDevSeen = signedDev
                if (signedDev < minDevSeen) minDevSeen = signedDev
            }
        }
        if (countF == 0) return Triple(360f, crudeMeanDeg, 360f)

        val meanRad = atan2(sumSinF, sumCosF)
        val meanDeg = (Math.toDegrees(meanRad).toFloat() + 360f) % 360f

        val spread = maxDevSeen - minDevSeen
        val precision = (spread / sqrt(countF.toFloat())).coerceAtLeast(0.05f)

        return Triple(spread, meanDeg, precision)
    }

    companion object {
        fun normalize360(deg: Float): Float {
            val r = deg % 360f
            return if (r < 0f) r + 360f else r
        }

        /** Distancia circular sin signo en [0, 180]. */
        fun circularDist(a: Float, b: Float): Float {
            val d = ((a - b) % 360f + 360f) % 360f
            return if (d > 180f) 360f - d else d
        }

        /** Distancia circular con signo en (-180, 180]. */
        fun signedCircularDist(a: Float, b: Float): Float {
            val d = ((a - b) % 360f + 360f) % 360f
            return if (d > 180f) d - 360f else d
        }
    }
}