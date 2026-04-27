package com.example.sunproject.domain.sensor

import android.util.Log
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat

/**
 * Buffer circular de matrices de rotación 3x3 (TYPE_ROTATION_VECTOR
 * convertido a matriz via SensorManager.getRotationMatrixFromVector).
 *
 * Uso:
 *   - En el handler de TYPE_ROTATION_VECTOR: pushSample(timestampNs, rMatrix)
 *   - Al congelar pose para una foto: averageOverWindow(t1Ns, windowMs) →
 *     matriz de rotación promediada vía SVD sobre los samples cuyo timestamp
 *     cae en [t1Ns - windowMs*1e6, t1Ns].
 *
 * El promedio NO es elemento-a-elemento (esa "media" no es ortonormal en
 * general). Se usa la media de Markley: sumar matrices, SVD, reconstruir
 * R = U · diag(1, 1, det(U·V^T)) · V^T para asegurar det = +1.
 *
 * Tamaño típico: 256 samples a SENSOR_DELAY_GAME (~200 Hz) cubre ~1.3s.
 * Suficiente para promediar ventanas de 100-300ms con holgura.
 */
class RotvecBuffer(
    private val capacity: Int = 256
) {
    // Buffer SoA (struct-of-arrays) para evitar boxing y minimizar
    // allocations en el camino caliente del sensor.
    private val tsNs = LongArray(capacity)
    private val data = FloatArray(capacity * 9)

    private var head: Int = 0
    private var size: Int = 0

    @Synchronized
    fun pushSample(timestampNs: Long, rotationMatrix: FloatArray) {
        if (rotationMatrix.size < 9) return
        tsNs[head] = timestampNs
        val base = head * 9
        System.arraycopy(rotationMatrix, 0, data, base, 9)
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
     * Devuelve la matriz 3x3 row-major promediada sobre los samples cuyo
     * timestamp cae en [endTimeNs - windowMs*1_000_000, endTimeNs].
     *
     * Devuelve null si:
     *   - No hay ningún sample en la ventana
     *   - SVD falla (matriz suma degenerada)
     *
     * Si outDiag != null y tiene ≥ 1 elemento, escribe outDiag[0] = cantidad
     * de samples usados.
     */
    @Synchronized
    fun averageOverWindow(
        endTimeNs: Long,
        windowMs: Long,
        outDiag: IntArray? = null
    ): FloatArray? {
        if (size == 0) return null
        val startTimeNs = endTimeNs - windowMs * 1_000_000L

        // Acumular suma componente a componente.
        val sum = DoubleArray(9)
        var n = 0

        for (i in 0 until size) {
            val t = tsNs[i]
            if (t in startTimeNs..endTimeNs) {
                val base = i * 9
                for (j in 0 until 9) sum[j] += data[base + j]
                n++
            }
        }

        outDiag?.let { if (it.isNotEmpty()) it[0] = n }

        if (n == 0) return null
        if (n == 1) {
            // Optimización: con un solo sample no hace falta SVD.
            val out = FloatArray(9)
            for (j in 0 until 9) out[j] = (sum[j] / n).toFloat()
            return out
        }

        // SVD de la matriz suma promedio. Markley 2007: la rotación que
        // minimiza la suma Frobenius es R = U · diag(1, 1, det(U V^T)) · V^T.
        val avgMat = Mat(3, 3, CvType.CV_64F)
        val avgArr = DoubleArray(9)
        for (j in 0 until 9) avgArr[j] = sum[j] / n
        avgMat.put(0, 0, avgArr)

        val w = Mat()
        val u = Mat()
        val vt = Mat()
        try {
            Core.SVDecomp(avgMat, w, u, vt, Core.SVD_FULL_UV)

            // R = U · S · V^T donde S = diag(1, 1, sign(det(U·V^T)))
            val uv = Mat()
            Core.gemm(u, vt, 1.0, Mat(), 0.0, uv)
            val detUV = Core.determinant(uv)
            val sign = if (detUV < 0.0) -1.0 else 1.0
            uv.release()

            val s = Mat.eye(3, 3, CvType.CV_64F)
            s.put(2, 2, sign)

            val us = Mat()
            Core.gemm(u, s, 1.0, Mat(), 0.0, us)
            val r = Mat()
            Core.gemm(us, vt, 1.0, Mat(), 0.0, r)
            us.release()
            s.release()

            val out = DoubleArray(9)
            r.get(0, 0, out)
            r.release()

            return FloatArray(9) { out[it].toFloat() }
        } catch (t: Throwable) {
            Log.w("RotvecBuffer", "SVD failed: ${t.message}")
            return null
        } finally {
            avgMat.release()
            w.release()
            u.release()
            vt.release()
        }
    }
}