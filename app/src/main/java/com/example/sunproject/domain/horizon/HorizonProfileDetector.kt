package com.example.sunproject.domain.horizon

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.domain.atlas.AtlasConfig
import com.example.sunproject.domain.atlas.AtlasMath
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/**
 * Detector de perfil de obstrucción sobre un atlas equirectangular.
 *
 * Pipeline:
 *   1. Bitmap → Mat RGBA → Mat RGB → Mat HSV → canal V (luminancia)
 *   2. Otsu adaptativo por bandas verticales de 30° de azimut:
 *        - Threshold CALIBRADO sobre filas con alt < calibrationMaxAltitudeDeg
 *          (evita el cielo cenital muy oscuro que sesga la distribución bimodal
 *          que Otsu necesita; en Cauchari a 4024m esto importa).
 *        - Threshold APLICADO a toda la altura del atlas.
 *   3. Máscara binaria global: V >= threshold(banda) → cielo (255), si no → suelo (0).
 *   4. Cierre morfológico vertical: rellena huecos chicos en el cielo (nubes ralas,
 *      ruido). Kernel asimétrico (más alto que ancho) para preservar estructuras
 *      verticales delgadas como postes y antenas.
 *   5. Búsqueda del horizonte por columna, ARRANCANDO DESDE ABAJO (alt=0°) HACIA
 *      ARRIBA: encontrar el primer pixel de cielo. Esa elección es robusta al
 *      problema conocido del cielo cenital oscuro. Si en validación se ve que
 *      el piso pulido/asfalto al sol genera falsos cielos abajo, evaluamos
 *      cambiar a búsqueda desde arriba o requerir que las K filas superiores
 *      sean también cielo (consistencia local).
 *   6. Agregación a buckets de azimut con la estrategia configurada (mediana
 *      por default), y suavizado circular con ventana smoothingWindowDeg.
 *
 * Costo: ~30M ops sobre atlas 3600×900. Estimado 100-300ms en teléfono moderno.
 * Llamar SIEMPRE desde background thread; nunca desde UI.
 *
 * Convención de salida: HorizonProfile.altByAzimuthDeg[0]=N, índices CW (NOAA).
 */
object HorizonProfileDetector {

    private const val TAG = "HorizonDetector"

    /** Resultado del detector. debugInfo se computa solo si se pidió. */
    data class Result(
        val profile: HorizonProfile,
        val debugInfo: DebugInfo?
    )

    /**
     * Diagnóstico de una corrida del detector. Útil para inspeccionar visualmente
     * qué pasó cuando un perfil sale raro.
     */
    data class DebugInfo(
        /** Threshold Otsu calculado en cada banda vertical [0, 255]. */
        val perBandThresholds: List<Double>,
        /** Cantidad de píxeles usados para calibrar cada banda. */
        val perBandSampleCounts: List<Int>,
        /**
         * Atlas + máscara de cielo en azul + polilínea verde del horizonte +
         * líneas de borde de banda. Pesa ~12MB. No retener más de lo necesario.
         */
        val overlayBitmap: Bitmap,
        /** Altitud por columna del atlas (pre-agregación a buckets). */
        val altByColumnDeg: FloatArray,
        /** Tiempo total de detección en ms. */
        val totalElapsedMs: Long
    )

    /**
     * Detecta el perfil de obstrucción del atlas. Llamar siempre desde background.
     *
     * @param atlasBitmap atlas equirectangular ya construido. NO se modifica.
     * @param atlasConfig configuración del atlas (rangos az/alt). Debe coincidir
     *                    con las dimensiones del bitmap.
     * @param sessionId   id de sesión, va al metadata del HorizonProfile.
     * @param sourceAtlasName nombre del PNG de origen, para trazabilidad.
     * @param params parámetros del algoritmo. DEFAULT = recomendado para v1.
     * @param generateDebugInfo si true, computa overlayBitmap (memoria extra).
     */
    fun detect(
        atlasBitmap: Bitmap,
        atlasConfig: AtlasConfig,
        sessionId: String,
        sourceAtlasName: String,
        params: DetectionParams = DetectionParams.DEFAULT,
        generateDebugInfo: Boolean = false
    ): Result {
        SunProjectApp.requireOpenCv()
        val startNs = System.nanoTime()

        require(atlasBitmap.width == atlasConfig.widthPx && atlasBitmap.height == atlasConfig.heightPx) {
            "Bitmap size (${atlasBitmap.width}×${atlasBitmap.height}) != " +
                    "config (${atlasConfig.widthPx}×${atlasConfig.heightPx})"
        }

        val w = atlasConfig.widthPx
        val h = atlasConfig.heightPx
        val numBuckets = HorizonProfile.DEFAULT_AZIMUTH_BUCKETS

        val vChannel = extractVChannel(atlasBitmap)
        try {
            // Calibrar Otsu por banda usando solo filas alt < calibrationMaxAltitudeDeg.
            val calibrationYStart = AtlasMath.altitudeToY(params.calibrationMaxAltitudeDeg, atlasConfig)
            val bandWidth = w / params.azimuthBands
            val perBandThresholds = DoubleArray(params.azimuthBands)
            val perBandSampleCounts = IntArray(params.azimuthBands)

            for (b in 0 until params.azimuthBands) {
                val xStart = b * bandWidth
                val xEnd = if (b == params.azimuthBands - 1) w else (b + 1) * bandWidth
                val (thr, count) = calibrateOtsuOnRoi(
                    vChannel, xStart, xEnd, calibrationYStart, h
                )
                perBandThresholds[b] = thr
                perBandSampleCounts[b] = count
                Log.d(
                    TAG,
                    "band[$b] x=[$xStart,$xEnd) yCalib=[$calibrationYStart,$h) " +
                            "threshold=${"%.1f".format(thr)} samples=$count"
                )
            }

            val mask = buildBinaryMaskPerBand(
                vChannel, perBandThresholds, bandWidth, params.azimuthBands
            )
            try {
                val closedMask = morphClose(
                    mask, params.morphCloseKernelWidth, params.morphCloseKernelHeight
                )
                try {
                    val altByColumnDeg = findHorizonByColumn(closedMask, atlasConfig)
                    val altByBucketRaw = aggregateColumnsToBuckets(
                        altByColumnDeg, atlasConfig, numBuckets, params.aggregationStrategy
                    )
                    val altSmoothed = if (params.smoothingWindowDeg > 0) {
                        smoothCircularProfile(altByBucketRaw, params.smoothingWindowDeg, numBuckets)
                    } else {
                        altByBucketRaw
                    }

                    val profile = HorizonProfile(
                        schemaVersion = HorizonProfile.CURRENT_SCHEMA_VERSION,
                        sessionId = sessionId,
                        createdAtUtcMs = System.currentTimeMillis(),
                        sourceAtlasName = sourceAtlasName,
                        azimuthBuckets = numBuckets,
                        altByAzimuthDeg = altSmoothed.toList(),
                        detectionParams = params
                    )

                    val debugInfo = if (generateDebugInfo) {
                        val overlay = buildDebugOverlay(
                            atlasBitmap, closedMask, altByColumnDeg,
                            atlasConfig, params.azimuthBands, bandWidth
                        )
                        DebugInfo(
                            perBandThresholds = perBandThresholds.toList(),
                            perBandSampleCounts = perBandSampleCounts.toList(),
                            overlayBitmap = overlay,
                            altByColumnDeg = altByColumnDeg,
                            totalElapsedMs = (System.nanoTime() - startNs) / 1_000_000L
                        )
                    } else null

                    val (minV, maxV, meanV) = stats(altSmoothed)
                    val elapsedMs = (System.nanoTime() - startNs) / 1_000_000L
                    Log.i(
                        TAG,
                        "Profile detected: $numBuckets buckets, " +
                                "min=${"%.1f".format(minV)}° max=${"%.1f".format(maxV)}° " +
                                "mean=${"%.1f".format(meanV)}° elapsed=${elapsedMs}ms " +
                                "strategy=${params.aggregationStrategy}"
                    )

                    return Result(profile, debugInfo)
                } finally {
                    closedMask.release()
                }
            } finally {
                mask.release()
            }
        } finally {
            vChannel.release()
        }
    }

    // -- Pipeline steps ------------------------------------------------------

    /** Bitmap RGBA → Mat HSV → devuelve canal V (CV_8UC1) clonado. */
    private fun extractVChannel(bitmap: Bitmap): Mat {
        val rgba = Mat()
        Utils.bitmapToMat(bitmap, rgba)
        try {
            val rgb = Mat()
            Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_RGBA2RGB)
            try {
                val hsv = Mat()
                Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
                try {
                    val channels = mutableListOf<Mat>()
                    Core.split(hsv, channels)
                    try {
                        // V = canal 2 (H=0, S=1, V=2). clone() para que sobreviva
                        // al release() del Mat parent dentro del split.
                        return channels[2].clone()
                    } finally {
                        channels.forEach { it.release() }
                    }
                } finally {
                    hsv.release()
                }
            } finally {
                rgb.release()
            }
        } finally {
            rgba.release()
        }
    }

    /**
     * Otsu sobre un ROI [xStart, xEnd) × [yStart, yEnd) del canal V.
     * Devuelve (threshold, samplesUsados).
     */
    private fun calibrateOtsuOnRoi(
        vChannel: Mat,
        xStart: Int, xEnd: Int,
        yStart: Int, yEnd: Int
    ): Pair<Double, Int> {
        val roiW = xEnd - xStart
        val roiH = yEnd - yStart
        if (roiW <= 0 || roiH <= 0) {
            Log.w(TAG, "calibrateOtsuOnRoi: empty ROI ($xStart,$yStart)-($xEnd,$yEnd)")
            return 127.0 to 0
        }
        val roi = vChannel.submat(yStart, yEnd, xStart, xEnd)
        try {
            val dst = Mat()
            try {
                // El 2do arg (val=0.0) lo ignora Otsu; calcula su propio threshold.
                val thr = Imgproc.threshold(
                    roi, dst, 0.0, 255.0,
                    Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU
                )
                return thr to (roiW * roiH)
            } finally {
                dst.release()
            }
        } finally {
            roi.release()
        }
    }

    /**
     * Aplica threshold por banda al canal V completo. Cada columna usa el
     * threshold de su banda. Devuelve máscara CV_8UC1 con 255=cielo, 0=suelo.
     */
    private fun buildBinaryMaskPerBand(
        vChannel: Mat,
        perBandThresholds: DoubleArray,
        bandWidth: Int,
        numBands: Int
    ): Mat {
        val mask = Mat.zeros(vChannel.size(), CvType.CV_8UC1)
        val w = vChannel.cols()
        val h = vChannel.rows()
        for (b in 0 until numBands) {
            val xStart = b * bandWidth
            val xEnd = if (b == numBands - 1) w else (b + 1) * bandWidth
            val thr = perBandThresholds[b]
            val srcRoi = vChannel.submat(0, h, xStart, xEnd)
            val dstRoi = mask.submat(0, h, xStart, xEnd)
            try {
                Imgproc.threshold(srcRoi, dstRoi, thr, 255.0, Imgproc.THRESH_BINARY)
            } finally {
                srcRoi.release()
                dstRoi.release()
            }
        }
        return mask
    }

    /** Cierre morfológico con kernel rectangular asimétrico. */
    private fun morphClose(mask: Mat, kernelW: Int, kernelH: Int): Mat {
        val kernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_RECT, Size(kernelW.toDouble(), kernelH.toDouble())
        )
        try {
            val closed = Mat()
            Imgproc.morphologyEx(mask, closed, Imgproc.MORPH_CLOSE, kernel)
            return closed
        } finally {
            kernel.release()
        }
    }

    /**
     * Para cada columna, busca desde abajo (alt=0°) hacia arriba el primer
     * pixel de cielo. Devuelve altMin en grados por columna.
     *
     * Convenciones de salida:
     *  - Columna toda suelo:  altMin = 90° (degenerado, no debería pasar)
     *  - Columna toda cielo:  altMin = 0°  (horizonte libre)
     *  - Caso normal:         altMin = altitud del primer cielo desde abajo
     */
    private fun findHorizonByColumn(mask: Mat, cfg: AtlasConfig): FloatArray {
        val w = mask.cols()
        val h = mask.rows()
        val result = FloatArray(w)

        // Bulk-read: leer toda la máscara a un ByteArray en una sola llamada JNI.
        // mask.get(0, 0, buf) píxel a píxel es órdenes de magnitud más lento.
        val maskBytes = ByteArray(w * h)
        mask.get(0, 0, maskBytes)

        for (x in 0 until w) {
            var horizonY = -1
            for (y in (h - 1) downTo 0) {
                val v = maskBytes[y * w + x].toInt() and 0xFF
                if (v >= 128) {
                    horizonY = y
                    break
                }
            }
            result[x] = if (horizonY < 0) {
                90f
            } else {
                AtlasMath.yToAltitude(horizonY, cfg)
            }
        }
        return result
    }

    /**
     * Agrega altByColumn a buckets de azimut con la estrategia indicada.
     * Convención de buckets: 0=N, CW (NOAA). Bucket b cubre az ∈ [b·360/N, (b+1)·360/N).
     */
    private fun aggregateColumnsToBuckets(
        altByColumn: FloatArray,
        cfg: AtlasConfig,
        numBuckets: Int,
        strategy: AggregationStrategy
    ): FloatArray {
        val bucketLists = Array(numBuckets) { mutableListOf<Float>() }
        val bucketWidth = 360f / numBuckets

        for (x in altByColumn.indices) {
            val azAtlas = AtlasMath.xToAzimuth(x, cfg)  // rango atlas: [-180, 180]
            var azNoaa = azAtlas
            if (azNoaa < 0f) azNoaa += 360f             // rango bucket: [0, 360)
            val bucketIdx = (azNoaa / bucketWidth).toInt().coerceIn(0, numBuckets - 1)
            bucketLists[bucketIdx].add(altByColumn[x])
        }

        return FloatArray(numBuckets) { b ->
            val values = bucketLists[b]
            if (values.isEmpty()) 0f else aggregate(values, strategy)
        }
    }

    private fun aggregate(values: List<Float>, strategy: AggregationStrategy): Float {
        val sorted = values.sorted()
        val n = sorted.size
        return when (strategy) {
            AggregationStrategy.MEDIAN -> sorted[n / 2]
            AggregationStrategy.P75 -> sorted[(n * 3 / 4).coerceAtMost(n - 1)]
            AggregationStrategy.P90 -> sorted[(n * 9 / 10).coerceAtMost(n - 1)]
            AggregationStrategy.MAX -> sorted.last()
        }
    }

    /** Moving average circular sobre el array de buckets. */
    private fun smoothCircularProfile(
        altByBucket: FloatArray,
        windowDeg: Int,
        numBuckets: Int
    ): FloatArray {
        val bucketWidth = 360f / numBuckets
        val halfWindowBuckets = (windowDeg / 2f / bucketWidth).toInt().coerceAtLeast(1)
        val result = FloatArray(numBuckets)
        for (b in 0 until numBuckets) {
            var sum = 0f
            var count = 0
            for (offset in -halfWindowBuckets..halfWindowBuckets) {
                val idx = ((b + offset) % numBuckets + numBuckets) % numBuckets
                sum += altByBucket[idx]
                count++
            }
            result[b] = sum / count
        }
        return result
    }

    /** (min, max, mean) en un solo pass. */
    private fun stats(arr: FloatArray): Triple<Float, Float, Float> {
        if (arr.isEmpty()) return Triple(0f, 0f, 0f)
        var minV = arr[0]
        var maxV = arr[0]
        var sum = 0f
        for (v in arr) {
            if (v < minV) minV = v
            if (v > maxV) maxV = v
            sum += v
        }
        return Triple(minV, maxV, sum / arr.size)
    }

    /**
     * Genera un bitmap de inspección para debugging del detector.
     *  - Capa 0: atlas original.
     *  - Capa 1: tinte azul translúcido en zonas de cielo (mask=255).
     *  - Capa 2: polilínea verde con la altitud detectada por columna.
     *  - Capa 3: líneas verticales blancas en los bordes de bandas Otsu.
     */
    private fun buildDebugOverlay(
        atlasBitmap: Bitmap,
        mask: Mat,
        altByColumnDeg: FloatArray,
        cfg: AtlasConfig,
        numBands: Int,
        bandWidth: Int
    ): Bitmap {
        val out = atlasBitmap.copy(Bitmap.Config.ARGB_8888, true)
        val w = out.width
        val h = out.height
        val canvas = Canvas(out)

        // Capa 1: tinte azul translúcido en cielo. Mezclamos 70% original + 30% azul.
        val maskBytes = ByteArray(w * h)
        mask.get(0, 0, maskBytes)
        val pixels = IntArray(w * h)
        out.getPixels(pixels, 0, w, 0, 0, w, h)
        for (i in pixels.indices) {
            val isSky = (maskBytes[i].toInt() and 0xFF) >= 128
            if (isSky) {
                val orig = pixels[i]
                val r = Color.red(orig)
                val g = Color.green(orig)
                val b = Color.blue(orig)
                val newR = (r * 0.7f).toInt().coerceIn(0, 255)
                val newG = (g * 0.7f).toInt().coerceIn(0, 255)
                val newB = (b * 0.7f + 255 * 0.3f).toInt().coerceIn(0, 255)
                pixels[i] = Color.argb(255, newR, newG, newB)
            }
        }
        out.setPixels(pixels, 0, w, 0, 0, w, h)

        // Capa 2: polilínea verde del horizonte por columna (pre-agregación).
        val horizonPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(255, 0, 255, 0)
            style = Paint.Style.STROKE
            strokeWidth = 2.5f
        }
        val path = Path()
        var first = true
        for (x in 0 until w) {
            val alt = altByColumnDeg[x]
            val y = AtlasMath.altitudeToY(alt, cfg).toFloat()
            if (first) { path.moveTo(x.toFloat(), y); first = false }
            else path.lineTo(x.toFloat(), y)
        }
        canvas.drawPath(path, horizonPaint)

        // Capa 3: bordes de banda Otsu (líneas verticales blancas finas).
        val bandPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(170, 255, 255, 255)
            style = Paint.Style.STROKE
            strokeWidth = 1f
        }
        for (b in 1 until numBands) {
            val x = (b * bandWidth).toFloat()
            canvas.drawLine(x, 0f, x, h.toFloat(), bandPaint)
        }

        return out
    }
}