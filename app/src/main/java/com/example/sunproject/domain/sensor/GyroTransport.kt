package com.example.sunproject.domain.sensor

import android.util.Log
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Buffer circular de samples del giroscopio + integrador de rotación incremental.
 *
 * Uso típico:
 *   - En el handler de TYPE_GYROSCOPE: pushSample(event.timestamp, wx, wy, wz)
 *   - En el momento del transporte: integrateBetween(t0Ns, t1Ns) → R_delta_device 3x3
 *
 * El integrador asume samples densos (~200 Hz con SENSOR_DELAY_GAME). Si hay
 * gaps mayores a maxGapNs entre samples consecutivos durante el intervalo
 * pedido, la integración se considera no confiable y devuelve null.
 *
 * La rotación devuelta está en el frame del DEVICE: representa "cuánto rotó el
 * celular en su propio sistema de coordenadas entre t0 y t1". Para componer con
 * una matriz device→world R_anchor:
 *     R_z0 = R_anchor · R_delta_device
 *
 * Composición: R_delta = ∏ exp(omega_i · dt_i) usando Rodrigues por sample.
 * Para rotaciones pequeñas por sample (ω·dt ≈ 0.001-0.01 rad), esto es
 * numéricamente estable y de error < 0.01° por integración de 10s.
 */
class GyroTransport(
    private val capacity: Int = 4096,
    private val maxGapNs: Long = 200_000_000L  // 200ms = sample perdido
) {
    private val tsNs = LongArray(capacity)
    private val wx = FloatArray(capacity)
    private val wy = FloatArray(capacity)
    private val wz = FloatArray(capacity)

    private var head: Int = 0      // próximo índice a escribir
    private var size: Int = 0      // cantidad de samples válidos en buffer

    @Synchronized
    fun pushSample(timestampNs: Long, gx: Float, gy: Float, gz: Float) {
        tsNs[head] = timestampNs
        wx[head] = gx
        wy[head] = gy
        wz[head] = gz
        head = (head + 1) % capacity
        if (size < capacity) size++
    }

    @Synchronized
    fun clear() {
        head = 0
        size = 0
    }

    @Synchronized
    fun sampleCount(): Int = size

    /**
     * Devuelve la matriz 3x3 row-major R_delta_device tal que
     *     R(t1) = R(t0) · R_delta_device
     * donde R(t) es la orientación device→world en el tiempo t.
     *
     * Devuelve null si:
     *   - No hay samples cubriendo el intervalo [t0Ns, t1Ns]
     *   - Hay un gap > maxGapNs dentro del intervalo
     *   - t1Ns <= t0Ns
     *
     * Si la integración es válida, también escribe diagnósticos en outDiag:
     *   outDiag[0] = ángulo total integrado (grados)
     *   outDiag[1] = número de samples usados
     *   outDiag[2] = max gap observado (ms)
     */
    @Synchronized
    fun integrateBetween(
        t0Ns: Long,
        t1Ns: Long,
        outDiag: FloatArray? = null
    ): FloatArray? {
        if (t1Ns <= t0Ns) return null
        if (size < 2) return null

        // Reconstruir orden temporal del buffer circular en arreglo lineal.
        val n = size
        val orderedTs = LongArray(n)
        val orderedWx = FloatArray(n)
        val orderedWy = FloatArray(n)
        val orderedWz = FloatArray(n)
        val startIdx = if (size < capacity) 0 else head
        for (i in 0 until n) {
            val src = (startIdx + i) % capacity
            orderedTs[i] = tsNs[src]
            orderedWx[i] = wx[src]
            orderedWy[i] = wy[src]
            orderedWz[i] = wz[src]
        }

        // Encontrar primer sample con ts >= t0
        var i0 = -1
        for (i in 0 until n) {
            if (orderedTs[i] >= t0Ns) { i0 = i; break }
        }
        if (i0 == -1) return null
        if (i0 == 0) i0 = 1  // necesitamos un sample previo para dt

        // Encontrar último sample con ts <= t1
        var i1 = -1
        for (i in n - 1 downTo 0) {
            if (orderedTs[i] <= t1Ns) { i1 = i; break }
        }
        if (i1 == -1) return null
        if (i1 < i0) return null

        // Verificar cobertura del intervalo y gaps.
        val firstTs = orderedTs[i0 - 1]
        val lastTs = orderedTs[i1]
        if (firstTs > t0Ns + maxGapNs) return null
        if (lastTs < t1Ns - maxGapNs) return null

        var maxGap = 0L
        for (i in i0..i1) {
            val gap = orderedTs[i] - orderedTs[i - 1]
            if (gap > maxGap) maxGap = gap
        }
        if (maxGap > maxGapNs) {
            Log.w("GyroTransport", "gap=$maxGap ns exceeds maxGap=$maxGapNs")
            return null
        }

        // Integrar: R = ∏ exp(omega_i · dt_i) usando Rodrigues.
        // Inicializar R como identidad row-major.
        val R = floatArrayOf(
            1f, 0f, 0f,
            0f, 1f, 0f,
            0f, 0f, 1f
        )

        var sampleCount = 0
        var totalAngleRad = 0.0

        for (i in i0..i1) {
            val dtSec = (orderedTs[i] - orderedTs[i - 1]).toDouble() * 1e-9
            if (dtSec <= 0.0) continue

            // Velocidad angular promedio entre samples (trapezoidal).
            val avgWx = (orderedWx[i] + orderedWx[i - 1]) * 0.5
            val avgWy = (orderedWy[i] + orderedWy[i - 1]) * 0.5
            val avgWz = (orderedWz[i] + orderedWz[i - 1]) * 0.5

            // Rotación incremental theta = omega · dt
            val rx = avgWx * dtSec
            val ry = avgWy * dtSec
            val rz = avgWz * dtSec
            val theta = sqrt(rx * rx + ry * ry + rz * rz)

            if (theta < 1e-9) {
                sampleCount++
                continue
            }
            totalAngleRad += theta

            val ax = rx / theta
            val ay = ry / theta
            val az = rz / theta
            val c = cos(theta)
            val s = sin(theta)
            val oneMinusC = 1.0 - c

            // Matriz de Rodrigues row-major.
            val r00 = (c + ax * ax * oneMinusC).toFloat()
            val r01 = (ax * ay * oneMinusC - az * s).toFloat()
            val r02 = (ax * az * oneMinusC + ay * s).toFloat()
            val r10 = (ay * ax * oneMinusC + az * s).toFloat()
            val r11 = (c + ay * ay * oneMinusC).toFloat()
            val r12 = (ay * az * oneMinusC - ax * s).toFloat()
            val r20 = (az * ax * oneMinusC - ay * s).toFloat()
            val r21 = (az * ay * oneMinusC + ax * s).toFloat()
            val r22 = (c + az * az * oneMinusC).toFloat()

            // R = R · R_step  (composición de rotaciones del device frame)
            val n00 = R[0] * r00 + R[1] * r10 + R[2] * r20
            val n01 = R[0] * r01 + R[1] * r11 + R[2] * r21
            val n02 = R[0] * r02 + R[1] * r12 + R[2] * r22
            val n10 = R[3] * r00 + R[4] * r10 + R[5] * r20
            val n11 = R[3] * r01 + R[4] * r11 + R[5] * r21
            val n12 = R[3] * r02 + R[4] * r12 + R[5] * r22
            val n20 = R[6] * r00 + R[7] * r10 + R[8] * r20
            val n21 = R[6] * r01 + R[7] * r11 + R[8] * r21
            val n22 = R[6] * r02 + R[7] * r12 + R[8] * r22
            R[0] = n00; R[1] = n01; R[2] = n02
            R[3] = n10; R[4] = n11; R[5] = n12
            R[6] = n20; R[7] = n21; R[8] = n22

            sampleCount++
        }

        outDiag?.let {
            if (it.size >= 3) {
                it[0] = Math.toDegrees(totalAngleRad).toFloat()
                it[1] = sampleCount.toFloat()
                it[2] = (maxGap / 1_000_000.0).toFloat()
            }
        }
        return R
    }
}