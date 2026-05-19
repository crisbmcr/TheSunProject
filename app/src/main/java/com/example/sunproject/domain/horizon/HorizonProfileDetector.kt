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
 *   1. Bitmap → Mat RGBA → Mat RGB → Mat HSV → canal V (luminancia).
 *   2. Otsu adaptativo por bandas verticales de 30° de azimut:
 *        - Threshold CALIBRADO sobre filas con alt < calibrationMaxAltitudeDeg.
 *        - Threshold APLICADO a toda la altura del atlas.
 *   3. Máscara binaria global: V >= threshold(banda) → cielo (255), else suelo (0).
 *   4. Cierre morfológico vertical: rellena huecos chicos en estructuras para
 *      que el CCA aguas abajo las agrupe como una sola componente.
 *   5. Análisis de componentes conectados (CCA) sobre el no-cielo: nos
 *      quedamos solo con las componentes que tocan el borde inferior del
 *      atlas. Esto descarta nubes oscuras, ruido y manchas aisladas en el
 *      cielo (no llegan al suelo), y trata torres caladas como un solo
 *      obstáculo (cielo entre brazos no está conectado al cielo principal).
 *   6. Búsqueda del horizonte por columna DESDE ARRIBA sobre la máscara
 *      filtrada: el primer pixel marcado como obstáculo define la cima del
 *      obstáculo más alto en esa columna.
 *   7. Agregación a buckets de azimut con la estrategia configurada (P75
 *      por default), y suavizado circular con ventana smoothingWindowDeg.
 *
 * Costo: ~30M ops sobre atlas 3600×900. Estimado 200-400ms en teléfono
 * moderno. Llamar SIEMPRE desde background thread; nunca desde UI.
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
     * Diagnóstico de una corrida del detector. Útil para inspeccionar
     * visualmente qué pasó cuando un perfil sale raro.
     */
    data class DebugInfo(
        /** Threshold Otsu calculado en cada banda vertical [0, 255]. */
        val perBandThresholds: List<Double>,
        /** Cantidad de píxeles usados para calibrar cada banda. */
        val perBandSampleCounts: List<Int>,
        /**
         * Atlas + máscara de cielo en azul + polilínea verde del horizonte +
         * líneas de borde de banda. Pesa ~12MB.
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
                    // Filtrar componentes no-cielo por conectividad al borde
                    // inferior. Solo sobreviven los obstáculos físicos anclados
                    // al suelo. Nubes, ruido y huecos internos de estructuras
                    // se eliminan o se absorben en la estructura padre.
                    val obstacleMask = filterObstaclesConnectedToBottom(closedMask)
                    try {
                        val altByColumnDeg = findHorizonByColumn(obstacleMask, atlasConfig)
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
                            // Debug overlay usa la máscara filtrada por CCA
                            // para que se vea la silueta final de obstáculos
                            // (en lugar de la máscara intermedia de cielo).
                            val overlay = buildDebugOverlay(
                                atlasBitmap, obstacleMask, altByColumnDeg,
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
                        obstacleMask.release()
                    }
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
     * Toma la máscara de cielo (255=cielo, 0=no-cielo) y devuelve una nueva
     * máscara donde solo permanecen los píxeles de no-cielo que pertenecen a
     * una componente conectada que toca la fila inferior (y = h-1).
     *
     * Resultado: 255 = obstáculo verdadero (anclado al suelo),
     *              0 = cielo o artefacto no-cielo aislado (nube, ruido).
     *
     * Justificación:
     *   Los obstáculos físicos (montañas, edificios, torres, vegetación)
     *   parten desde el suelo y forman componentes conectadas al borde
     *   inferior del atlas. Una nube oscura dentro del cielo es una
     *   componente aislada sin contacto con el suelo. Filtrar por
     *   conectividad al borde inferior nos deja la silueta de obstáculos
     *   físicos sin contaminar con artefactos atmosféricos.
     *
     *   Edge case asumido: si una nube se extiende verticalmente hasta tocar
     *   el horizonte real (niebla envolviendo montañas), se tratará como
     *   parte del obstáculo. Consistente con la percepción visual del observador.
     */
    private fun filterObstaclesConnectedToBottom(skyMask: Mat): Mat {
        val nonSky = Mat()
        Core.bitwise_not(skyMask, nonSky)
        try {
            // CCA: cada componente blanca recibe un label entero único > 0.
            // 8-conectividad para tolerar conexiones diagonales (cables finos).
            val labels = Mat()
            Imgproc.connectedComponents(nonSky, labels, 8, CvType.CV_32S)
            try {
                val w = labels.cols()
                val h = labels.rows()

                // Identificar qué labels tocan la fila inferior.
                val bottomRow = IntArray(w)
                labels.get(h - 1, 0, bottomRow)
                val bottomLabels = HashSet<Int>(16)
                for (label in bottomRow) {
                    if (label != 0) bottomLabels.add(label)
                }

                if (bottomLabels.isEmpty()) {
                    Log.w(
                        TAG,
                        "filterObstaclesConnectedToBottom: 0 componentes tocan el borde inferior. " +
                                "Atlas posiblemente sin suelo visible (todo cielo o todo ruido)."
                    )
                    return Mat.zeros(h, w, CvType.CV_8UC1)
                }

                val labelsBuf = IntArray(w * h)
                labels.get(0, 0, labelsBuf)

                val outBytes = ByteArray(w * h)
                for (i in 0 until w * h) {
                    if (labelsBuf[i] in bottomLabels) {
                        outBytes[i] = 255.toByte()
                    }
                }

                val result = Mat(h, w, CvType.CV_8UC1)
                result.put(0, 0, outBytes)
                Log.d(
                    TAG,
                    "CCA: kept ${bottomLabels.size} component(s) connected to bottom row"
                )
                return result
            } finally {
                labels.release()
            }
        } finally {
            nonSky.release()
        }
    }

    /**
     * Para cada columna, busca DESDE ARRIBA el primer pixel de obstáculo
     * sobre la máscara ya filtrada por CCA. La altitud de ese pixel define
     * el horizonte.
     *
     * Convenciones de salida:
     *  - Columna sin obstáculo:  altMin = 0°  (horizonte libre)
     *  - Caso normal:            altMin = altitud del primer obstáculo
     *
     * Por qué desde arriba es robusto ahora: la máscara ya pasó por CCA, solo
     * quedan componentes ancladas al borde inferior. Nubes y manchas aisladas
     * ya fueron eliminadas. Estructuras caladas (torres, mástiles) están
     * marcadas como obstáculo "completo" incluyendo sus huecos internos.
     * Buscar desde arriba el primer pixel marcado da la cima real del
     * obstáculo más alto en cada columna.
     */
    private fun findHorizonByColumn(obstacleMask: Mat, cfg: AtlasConfig): FloatArray {
        val w = obstacleMask.cols()
        val h = obstacleMask.rows()
        val result = FloatArray(w)

        // Bulk-read: leer toda la máscara a un ByteArray en una sola llamada JNI.
        val maskBytes = ByteArray(w * h)
        obstacleMask.get(0, 0, maskBytes)

        for (x in 0 until w) {
            var horizonY = -1
            for (y in 0 until h) {
                val v = maskBytes[y * w + x].toInt() and 0xFF
                if (v >= 128) {
                    horizonY = y
                    break
                }
            }
            result[x] = if (horizonY < 0) {
                0f
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
            val azAtlas = AtlasMath.xToAzimuth(x, cfg)
            var azNoaa = azAtlas
            if (azNoaa < 0f) azNoaa += 360f
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
     *  - Capa 1: tinte rojo translúcido en zonas de obstáculo (máscara post-CCA).
     *  - Capa 2: polilínea verde con la altitud detectada por columna.
     *  - Capa 3: líneas verticales blancas en los bordes de bandas Otsu.
     *
     * Nota: tinta los obstáculos (no el cielo) porque sobre la máscara
     * post-CCA es más útil ver qué se consideró obstáculo final que ver el
     * cielo de calibración (que ya pasó por dos pasos de transformación).
     */
    private fun buildDebugOverlay(
        atlasBitmap: Bitmap,
        obstacleMask: Mat,
        altByColumnDeg: FloatArray,
        cfg: AtlasConfig,
        numBands: Int,
        bandWidth: Int
    ): Bitmap {
        val out = atlasBitmap.copy(Bitmap.Config.ARGB_8888, true)
        val w = out.width
        val h = out.height
        val canvas = Canvas(out)

        // Capa 1: tinte rojo translúcido en obstáculos. 70% original + 30% rojo.
        val maskBytes = ByteArray(w * h)
        obstacleMask.get(0, 0, maskBytes)
        val pixels = IntArray(w * h)
        out.getPixels(pixels, 0, w, 0, 0, w, h)
        for (i in pixels.indices) {
            val isObstacle = (maskBytes[i].toInt() and 0xFF) >= 128
            if (isObstacle) {
                val orig = pixels[i]
                val r = Color.red(orig)
                val g = Color.green(orig)
                val b = Color.blue(orig)
                val newR = (r * 0.7f + 255 * 0.3f).toInt().coerceIn(0, 255)
                val newG = (g * 0.7f).toInt().coerceIn(0, 255)
                val newB = (b * 0.7f).toInt().coerceIn(0, 255)
                pixels[i] = Color.argb(255, newR, newG, newB)
            }
        }
        out.setPixels(pixels, 0, w, 0, 0, w, h)

        // Capa 2: polilínea verde del horizonte por columna (pre-agregación).
        val horizonPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(255, 0, 255, 0)
            this.style = Paint.Style.STROKE
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
            this.style = Paint.Style.STROKE
            strokeWidth = 1f
        }
        for (b in 1 until numBands) {
            val x = (b * bandWidth).toFloat()
            canvas.drawLine(x, 0f, x, h.toFloat(), bandPaint)
        }

        return out
    }
}