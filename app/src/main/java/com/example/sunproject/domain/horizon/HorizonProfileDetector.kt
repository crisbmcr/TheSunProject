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
 *   1. Bitmap → HSV → canales V (luminancia) y S (saturación).
 *   2. Otsu adaptativo por bandas verticales sobre V (calibración).
 *   3. Sanity check: si una banda termina con fracción de cielo absurda
 *      (< 20% o > 97%), reemplazar su threshold por la mediana de las
 *      bandas sanas. Maneja el caso donde Otsu particiona el cielo
 *      gradiente en lugar de separar cielo de suelo.
 *   4. Máscara binaria combinando V y S:
 *        is_sky = (V >= T_banda AND S >= S_min) OR (V >= V_alto)
 *      - Cláusula AND: cielo azul saturado (caso típico).
 *      - Cláusula OR: sol, halo, nubes muy brillantes (V≈255, S bajo).
 *      Esto descarta correctamente metal/concreto brillante (V alto, S bajo).
 *   5. Cierre morfológico vertical.
 *   6. CCA: conservar solo componentes no-cielo conectadas al borde inferior.
 *   7. Búsqueda por columna desde arriba del primer obstáculo.
 *   8. Agregación a buckets de azimut + suavizado circular.
 *
 * Costo: ~30M ops sobre atlas 3600×900. ~300-500ms en teléfono moderno.
 */
object HorizonProfileDetector {

    private const val TAG = "HorizonDetector"

    data class Result(
        val profile: HorizonProfile,
        val debugInfo: DebugInfo?
    )

    data class DebugInfo(
        val perBandThresholds: List<Double>,
        val perBandSampleCounts: List<Int>,
        val perBandSkyFractions: List<Float>,
        val overlayBitmap: Bitmap,
        val altByColumnDeg: FloatArray,
        val totalElapsedMs: Long
    )

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
            "Bitmap (${atlasBitmap.width}×${atlasBitmap.height}) != " +
                    "config (${atlasConfig.widthPx}×${atlasConfig.heightPx})"
        }

        val w = atlasConfig.widthPx
        val h = atlasConfig.heightPx
        val numBuckets = HorizonProfile.DEFAULT_AZIMUTH_BUCKETS

        val (vChannel, sChannel) = extractVAndSChannels(atlasBitmap)
        try {
            // Bulk-read los canales una sola vez para reusar después.
            val vBuf = ByteArray(w * h)
            val sBuf = ByteArray(w * h)
            vChannel.get(0, 0, vBuf)
            sChannel.get(0, 0, sBuf)

            val calibrationYStart = AtlasMath.altitudeToY(params.calibrationMaxAltitudeDeg, atlasConfig)
            val bandWidth = w / params.azimuthBands
            val perBandThresholds = DoubleArray(params.azimuthBands)
            val perBandSampleCounts = IntArray(params.azimuthBands)

            // Paso 1: Otsu por banda sobre el ROI de calibración.
            for (b in 0 until params.azimuthBands) {
                val xStart = b * bandWidth
                val xEnd = if (b == params.azimuthBands - 1) w else (b + 1) * bandWidth
                val (thr, count) = calibrateOtsuOnRoi(
                    vChannel, xStart, xEnd, calibrationYStart, h
                )
                perBandThresholds[b] = thr
                perBandSampleCounts[b] = count
            }

            // Paso 2: sanity check + fallback a mediana de sanas.
            val perBandSkyFractions = applyPerBandSanityCheck(
                vBuf, perBandThresholds, params, w, h, bandWidth, params.azimuthBands
            )

            // Log post-fix para diagnóstico.
            for (b in 0 until params.azimuthBands) {
                Log.d(
                    TAG,
                    "band[$b] T=${"%.1f".format(perBandThresholds[b])} " +
                            "skyFrac=${"%.2f".format(perBandSkyFractions[b])}"
                )
            }

            // Paso 3: máscara con V + S combinados.
            val mask = buildSkyMaskWithSaturation(
                vBuf, sBuf, w, h,
                perBandThresholds, bandWidth, params.azimuthBands,
                params.saturationMinForSky, params.veryHighVForcedSky
            )
            try {
                val closedMask = morphClose(
                    mask, params.morphCloseKernelWidth, params.morphCloseKernelHeight
                )
                try {
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
                            val overlay = buildDebugOverlay(
                                atlasBitmap, obstacleMask, altByColumnDeg,
                                atlasConfig, params.azimuthBands, bandWidth
                            )
                            DebugInfo(
                                perBandThresholds = perBandThresholds.toList(),
                                perBandSampleCounts = perBandSampleCounts.toList(),
                                perBandSkyFractions = perBandSkyFractions.toList(),
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
                                    "mean=${"%.1f".format(meanV)}° elapsed=${elapsedMs}ms"
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
            sChannel.release()
        }
    }

    // -- Pipeline steps ------------------------------------------------------

    /** Bitmap RGBA → HSV → devuelve canales V y S clonados (CV_8UC1 cada uno). */
    private fun extractVAndSChannels(bitmap: Bitmap): Pair<Mat, Mat> {
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
                        // HSV → canal 0=H, 1=S, 2=V.
                        return channels[2].clone() to channels[1].clone()
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
     * Para cada banda, computa la fracción de pixels (sobre la banda completa)
     * que quedan clasificados como cielo SOLO mirando V vs T_b. Si esa
     * fracción cae fuera del rango sano, marca la banda como pathological y
     * reemplaza T_b por la mediana de las bandas sanas.
     *
     * Modifica perBandThresholds in-place. Devuelve la lista final de skyFracs
     * (re-evaluada después del reemplazo, para diagnóstico).
     */
    private fun applyPerBandSanityCheck(
        vBuf: ByteArray,
        perBandThresholds: DoubleArray,
        params: DetectionParams,
        w: Int, h: Int,
        bandWidth: Int,
        numBands: Int
    ): FloatArray {
        val skyFractions = FloatArray(numBands)
        for (b in 0 until numBands) {
            val xStart = b * bandWidth
            val xEnd = if (b == numBands - 1) w else (b + 1) * bandWidth
            val bandPixels = (xEnd - xStart) * h
            val T = perBandThresholds[b]
            var skyCount = 0
            for (y in 0 until h) {
                val rowOff = y * w
                for (x in xStart until xEnd) {
                    val v = vBuf[rowOff + x].toInt() and 0xFF
                    if (v >= T) skyCount++
                }
            }
            skyFractions[b] = skyCount.toFloat() / bandPixels
        }

        // Identificar bandas sanas.
        val healthyIndices = mutableListOf<Int>()
        for (b in 0 until numBands) {
            if (skyFractions[b] in params.healthyBandSkyFractionMin..params.healthyBandSkyFractionMax) {
                healthyIndices.add(b)
            }
        }

        if (healthyIndices.isEmpty()) {
            Log.w(TAG, "Todas las bandas fallaron sanity check. Sin fallback posible — dejando T_b originales.")
            return skyFractions
        }

        val healthyThresholds = healthyIndices.map { perBandThresholds[it] }.sorted()
        val medianT = healthyThresholds[healthyThresholds.size / 2]

        var replaced = 0
        for (b in 0 until numBands) {
            if (skyFractions[b] !in params.healthyBandSkyFractionMin..params.healthyBandSkyFractionMax) {
                Log.w(
                    TAG,
                    "band[$b] PATHOLOGICAL (skyFrac=${"%.2f".format(skyFractions[b])}, " +
                            "T=${"%.1f".format(perBandThresholds[b])}). Replacing with medianT=${"%.1f".format(medianT)}"
                )
                perBandThresholds[b] = medianT
                replaced++

                // Re-computar skyFraction para logging post-fix.
                val xStart = b * bandWidth
                val xEnd = if (b == numBands - 1) w else (b + 1) * bandWidth
                val bandPixels = (xEnd - xStart) * h
                var skyCount = 0
                for (y in 0 until h) {
                    val rowOff = y * w
                    for (x in xStart until xEnd) {
                        val v = vBuf[rowOff + x].toInt() and 0xFF
                        if (v >= medianT) skyCount++
                    }
                }
                skyFractions[b] = skyCount.toFloat() / bandPixels
            }
        }

        if (replaced > 0) {
            Log.i(TAG, "Sanity check: $replaced/$numBands bandas reemplazadas con medianT=${"%.1f".format(medianT)}")
        }

        return skyFractions
    }

    /**
     * Construye la máscara binaria de cielo (255=cielo, 0=no-cielo) usando
     * criterio combinado V+S:
     *   is_sky = (V >= T_banda AND S >= S_min) OR (V >= V_alto)
     */
    private fun buildSkyMaskWithSaturation(
        vBuf: ByteArray,
        sBuf: ByteArray,
        w: Int, h: Int,
        perBandThresholds: DoubleArray,
        bandWidth: Int,
        numBands: Int,
        sMin: Int,
        vHigh: Int
    ): Mat {
        val outBuf = ByteArray(w * h)
        for (b in 0 until numBands) {
            val xStart = b * bandWidth
            val xEnd = if (b == numBands - 1) w else (b + 1) * bandWidth
            val vT = perBandThresholds[b].toInt()
            for (y in 0 until h) {
                val rowOff = y * w
                for (x in xStart until xEnd) {
                    val i = rowOff + x
                    val v = vBuf[i].toInt() and 0xFF
                    val s = sBuf[i].toInt() and 0xFF
                    val isSky = (v >= vT && s >= sMin) || (v >= vHigh)
                    if (isSky) outBuf[i] = 255.toByte()
                }
            }
        }
        val mask = Mat(h, w, CvType.CV_8UC1)
        mask.put(0, 0, outBuf)
        return mask
    }

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

    private fun filterObstaclesConnectedToBottom(skyMask: Mat): Mat {
        val nonSky = Mat()
        Core.bitwise_not(skyMask, nonSky)
        try {
            val labels = Mat()
            Imgproc.connectedComponents(nonSky, labels, 8, CvType.CV_32S)
            try {
                val w = labels.cols()
                val h = labels.rows()

                val bottomRow = IntArray(w)
                labels.get(h - 1, 0, bottomRow)
                val bottomLabels = HashSet<Int>(16)
                for (label in bottomRow) {
                    if (label != 0) bottomLabels.add(label)
                }

                if (bottomLabels.isEmpty()) {
                    Log.w(TAG, "filterObstaclesConnectedToBottom: 0 componentes tocan el borde inferior")
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
                Log.d(TAG, "CCA: kept ${bottomLabels.size} component(s) connected to bottom row")
                return result
            } finally {
                labels.release()
            }
        } finally {
            nonSky.release()
        }
    }

    private fun findHorizonByColumn(obstacleMask: Mat, cfg: AtlasConfig): FloatArray {
        val w = obstacleMask.cols()
        val h = obstacleMask.rows()
        val result = FloatArray(w)
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
            result[x] = if (horizonY < 0) 0f
            else AtlasMath.yToAltitude(horizonY, cfg)
        }
        return result
    }

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