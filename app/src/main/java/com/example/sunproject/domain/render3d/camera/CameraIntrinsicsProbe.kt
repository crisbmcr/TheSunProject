package com.example.sunproject.domain.camera

import android.hardware.camera2.CameraCharacteristics
import android.util.Log
import android.util.Size
import kotlin.math.acos
import kotlin.math.atan
import kotlin.math.sqrt

/**
 * Lee parámetros de calibración intrínseca y distorsión desde
 * Camera2 characteristics, si el fabricante los expone.
 *
 * Fase 1: SOLO diagnóstico. No persiste, no integra al pipeline.
 *
 * Si tu celular es razonablemente moderno (API 28+ con OEM que
 * provea LENS_DISTORTION y LENS_INTRINSIC_CALIBRATION), este probe
 * te devuelve los k1..k3, p1, p2 reales de tu lente, los intrínsecos
 * fx, fy, cx, cy reales en píxeles del sensor raw, y un veredicto
 * sobre la magnitud de la distorsión radial.
 *
 * Fase 2 (futura, condicional al veredicto): si verdict in {MODERATE, HIGH},
 * vamos a integrar estos parámetros al AtlasBundleAdjuster (undistort en
 * pixelToDeviceRay) y al AtlasProjector (distort al muestrear el frame
 * source). Si verdict in {LOW, NOT_AVAILABLE}, esa integración no vale
 * la pena y vamos por otra palanca.
 */
object CameraIntrinsicsProbe {

    enum class Verdict {
        /** Camera2 no expuso los campos. Path 1 no se puede tomar. */
        NOT_AVAILABLE,
        /** Distorsión <0.5° en el borde. Modelar no mueve la aguja. */
        LOW,
        /** Distorsión 0.5-1.5°. Modelar puede ayudar. */
        MODERATE,
        /** Distorsión >1.5°. Modelar mejorará el atlas. */
        HIGH
    }

    data class Result(
        val cameraId: String,
        val hasLensDistortion: Boolean,
        val hasIntrinsicCalibration: Boolean,

        // Coeficientes de distorsión Camera2 → orden Brown-Conrady (OpenCV):
        // Camera2 LENS_DISTORTION devuelve [kappa_1, kappa_2, kappa_3, kappa_4, kappa_5]
        // donde kappa_1..3 son radiales (k1, k2, k3) y kappa_4..5 son tangenciales (p1, p2).
        val k1: Float?, val k2: Float?, val k3: Float?,
        val p1: Float?, val p2: Float?,

        // Calibración intrínseca en píxeles del sensor RAW (no del frame final).
        val fx: Float?, val fy: Float?,
        val cx: Float?, val cy: Float?,
        val skew: Float?,

        val rawSensorWidthPx: Int?,
        val rawSensorHeightPx: Int?,

        // Métricas derivadas para interpretación.
        val hfovFromIntrinsicsDeg: Float?,
        val vfovFromIntrinsicsDeg: Float?,
        val distortionAt50PctEdgeDeg: Float?,
        val distortionAt100PctEdgeDeg: Float?,
        val centerOpticalOffsetPx: Float?,

        val verdict: Verdict
    )

    fun probe(cameraId: String, c: CameraCharacteristics): Result {
        val distortion: FloatArray? = c.get(CameraCharacteristics.LENS_DISTORTION)
        val intrinsics: FloatArray? = c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)
        val rawSize: Size? = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)

        val hasDist = distortion != null && distortion.size >= 5
        val hasIntr = intrinsics != null && intrinsics.size >= 5

        val k1 = if (hasDist) distortion!![0] else null
        val k2 = if (hasDist) distortion!![1] else null
        val k3 = if (hasDist) distortion!![2] else null
        val p1 = if (hasDist) distortion!![3] else null
        val p2 = if (hasDist) distortion!![4] else null

        val fx = if (hasIntr) intrinsics!![0] else null
        val fy = if (hasIntr) intrinsics!![1] else null
        val cx = if (hasIntr) intrinsics!![2] else null
        val cy = if (hasIntr) intrinsics!![3] else null
        val skew = if (hasIntr) intrinsics!![4] else null

        val rawW = rawSize?.width
        val rawH = rawSize?.height

        // HFOV/VFOV reales si tenemos intrínsecos + tamaño raw.
        // Si difieren del HFOV teórico actual del proyecto (calculado a
        // partir de SENSOR_INFO_PHYSICAL_SIZE + LENS_INFO_AVAILABLE_FOCAL_LENGTHS),
        // ya tenemos un primer hallazgo: el modelo asumido en el repo está
        // sesgado por algunos grados, lo cual contamina el seed de poses.
        val hfovReal = if (fx != null && rawW != null && fx > 0f) {
            (2.0 * atan(rawW / (2.0 * fx)) * 180.0 / Math.PI).toFloat()
        } else null
        val vfovReal = if (fy != null && rawH != null && fy > 0f) {
            (2.0 * atan(rawH / (2.0 * fy)) * 180.0 / Math.PI).toFloat()
        } else null

        // Offset del centro óptico respecto del centro geométrico del sensor.
        // Típicamente 5-15 px en celulares. >30 px indica calibración inusual.
        val centerOffset = if (cx != null && cy != null && rawW != null && rawH != null) {
            val dx = cx - rawW / 2f
            val dy = cy - rawH / 2f
            sqrt(dx * dx + dy * dy)
        } else null

        // Distorsión angular a 50% y 100% del trayecto centro→esquina del sensor raw.
        val (dist50, dist100) =
            if (hasDist && hasIntr && rawW != null && rawH != null) {
                computeDistortionAtEdges(k1!!, k2!!, k3!!, p1!!, p2!!, fx!!, fy!!, cx!!, cy!!, rawW, rawH)
            } else null to null

        val verdict = when {
            !hasDist || !hasIntr || dist100 == null -> Verdict.NOT_AVAILABLE
            dist100 < 0.5f -> Verdict.LOW
            dist100 < 1.5f -> Verdict.MODERATE
            else -> Verdict.HIGH
        }

        val result = Result(
            cameraId = cameraId,
            hasLensDistortion = hasDist,
            hasIntrinsicCalibration = hasIntr,
            k1 = k1, k2 = k2, k3 = k3, p1 = p1, p2 = p2,
            fx = fx, fy = fy, cx = cx, cy = cy, skew = skew,
            rawSensorWidthPx = rawW, rawSensorHeightPx = rawH,
            hfovFromIntrinsicsDeg = hfovReal,
            vfovFromIntrinsicsDeg = vfovReal,
            distortionAt50PctEdgeDeg = dist50,
            distortionAt100PctEdgeDeg = dist100,
            centerOpticalOffsetPx = centerOffset,
            verdict = verdict
        )

        logResult(result)
        return result
    }

    /**
     * Diagnóstico de la magnitud de distorsión.
     *
     * Toma un rayo ideal del mundo en coordenadas normalizadas, simulando
     * dirigirse al 50% y al 100% del trayecto del centro óptico hacia la
     * esquina del sensor raw. Aplica el modelo Brown-Conrady forward para
     * computar dónde el sensor realmente registra ese rayo. La diferencia
     * angular entre el rayo ideal (lo que el código actual asume) y el
     * rayo registrado (modelo correcto) es el error que el modelo pinhole
     * comete en esa fracción del frame.
     *
     * Nota: esto reporta el error que el proyector y el BA actuales
     * cometen — no la "magnitud de la distorsión" en sentido fotográfico.
     * Son conceptos distintos pero relacionados.
     */
    private fun computeDistortionAtEdges(
        k1: Float, k2: Float, k3: Float, p1: Float, p2: Float,
        fx: Float, fy: Float, cx: Float, cy: Float,
        rawW: Int, rawH: Int
    ): Pair<Float, Float> {
        val cornerX = (rawW.toFloat() - cx) / fx
        val cornerY = (rawH.toFloat() - cy) / fy

        fun displacementAtFraction(f: Float): Float {
            val xN = f * cornerX
            val yN = f * cornerY
            val rSq = xN * xN + yN * yN

            val radial = 1f + k1 * rSq + k2 * rSq * rSq + k3 * rSq * rSq * rSq
            val xD = xN * radial + 2f * p1 * xN * yN + p2 * (rSq + 2f * xN * xN)
            val yD = yN * radial + p1 * (rSq + 2f * yN * yN) + 2f * p2 * xN * yN

            val len1 = sqrt(xN * xN + yN * yN + 1f)
            val len2 = sqrt(xD * xD + yD * yD + 1f)
            val dot = (xN * xD + yN * yD + 1f) / (len1 * len2)
            return Math.toDegrees(acos(dot.coerceIn(-1f, 1f)).toDouble()).toFloat()
        }

        return displacementAtFraction(0.5f) to displacementAtFraction(1.0f)
    }

    private fun logResult(r: Result) {
        if (!r.hasLensDistortion && !r.hasIntrinsicCalibration) {
            Log.w(
                TAG,
                "cameraId=${r.cameraId} no Camera2 calibration metadata — verdict=${r.verdict.name}"
            )
            return
        }
        Log.i(
            TAG,
            "cameraId=${r.cameraId} hasLensDistortion=${r.hasLensDistortion} " +
                    "hasIntrinsicCalibration=${r.hasIntrinsicCalibration}"
        )
        if (r.hasLensDistortion) {
            Log.i(
                TAG,
                "distortion k1=${"%.5f".format(r.k1)} k2=${"%.5f".format(r.k2)} " +
                        "k3=${"%.5f".format(r.k3)} p1=${"%.5f".format(r.p1)} p2=${"%.5f".format(r.p2)}"
            )
        }
        if (r.hasIntrinsicCalibration) {
            Log.i(
                TAG,
                "intrinsics fx=${"%.1f".format(r.fx)} fy=${"%.1f".format(r.fy)} " +
                        "cx=${"%.1f".format(r.cx)} cy=${"%.1f".format(r.cy)} " +
                        "skew=${"%.5f".format(r.skew)} rawSize=${r.rawSensorWidthPx}x${r.rawSensorHeightPx}"
            )
            Log.i(
                TAG,
                "derivedFromIntrinsics hfov=${r.hfovFromIntrinsicsDeg?.let { "%.2f".format(it) }}deg " +
                        "vfov=${r.vfovFromIntrinsicsDeg?.let { "%.2f".format(it) }}deg " +
                        "centerOffsetPx=${r.centerOpticalOffsetPx?.let { "%.1f".format(it) }}"
            )
        }
        if (r.distortionAt50PctEdgeDeg != null && r.distortionAt100PctEdgeDeg != null) {
            Log.i(
                TAG,
                "modelError@50%edge=${"%.3f".format(r.distortionAt50PctEdgeDeg)}deg " +
                        "modelError@100%edge=${"%.3f".format(r.distortionAt100PctEdgeDeg)}deg " +
                        "verdict=${r.verdict.name}"
            )
        } else {
            Log.i(TAG, "verdict=${r.verdict.name}")
        }
    }

    private const val TAG = "CameraIntrinsicsProbe"
}