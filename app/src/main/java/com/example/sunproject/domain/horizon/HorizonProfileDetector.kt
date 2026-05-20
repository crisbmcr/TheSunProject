package com.example.sunproject.domain.horizon

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.domain.atlas.AtlasConfig
import com.example.sunproject.domain.atlas.AtlasMath
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

/**
 * Detector de perfil de obstrucción sobre un atlas equirectangular.
 *
 * Pipeline (v2 — segmentación semántica con TFLite):
 *   1. Bitmap del atlas → SkyAtlasSegmenter (tiling 513×513 + CNN DeepLab v3
 *      MobileNet V2 + stitching) → máscara binaria cielo/no-cielo Mat CV_8UC1.
 *   2. Filtro CCA sobre la máscara invertida: solo conservar componentes
 *      no-cielo conectadas al borde inferior. Defensivo contra artefactos
 *      del CNN (nubes mal clasificadas como obstáculo, ruido aislado).
 *   3. Búsqueda del horizonte por columna desde arriba sobre la máscara
 *      filtrada: primer pixel obstáculo define la cima.
 *   4. Agregación a buckets de azimut con la estrategia configurada (P75
 *      por default) y suavizado circular.
 *
 * Costo: ~4-5s con GPU delegate, ~25-30s en CPU fallback.
 * Llamar SIEMPRE desde background thread.
 */
object HorizonProfileDetector {

    private const val TAG = "HorizonDetector"

    data class Result(
        val profile: HorizonProfile,
        val debugInfo: DebugInfo?
    )

    data class DebugInfo(
        val overlayBitmap: Bitmap,
        val altByColumnDeg: FloatArray,
        val totalElapsedMs: Long,
        val segmentationElapsedMs: Long
    )

    /**
     * Detecta el perfil de obstrucción del atlas. Llamar siempre desde background.
     */
    fun detect(
        context: Context,
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

        val numBuckets = HorizonProfile.DEFAULT_AZIMUTH_BUCKETS

        // Paso 1: segmentación de cielo vía CNN.
        val segStartNs = System.nanoTime()
        val skyMask = SkyAtlasSegmenter(context).use { segmenter ->
            segmenter.segmentAtlas(atlasBitmap)
        }
        val segmentationElapsedMs = (System.nanoTime() - segStartNs) / 1_000_000L

        try {
            // Paso 2: filtro CCA — quedarse con obstáculos conectados al borde inferior.
            val obstacleMask = filterObstaclesConnectedToBottom(skyMask)
            try {
                // Paso 3: búsqueda del horizonte por columna.
                val altByColumnDeg = findHorizonByColumn(obstacleMask, atlasConfig)

                // Paso 4: agregación a buckets + suavizado.
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

                val totalElapsedMs = (System.nanoTime() - startNs) / 1_000_000L
                val debugInfo = if (generateDebugInfo) {
                    val overlay = buildDebugOverlay(
                        atlasBitmap, obstacleMask, altByColumnDeg, atlasConfig
                    )
                    DebugInfo(
                        overlayBitmap = overlay,
                        altByColumnDeg = altByColumnDeg,
                        totalElapsedMs = totalElapsedMs,
                        segmentationElapsedMs = segmentationElapsedMs
                    )
                } else null

                val (minV, maxV, meanV) = stats(altSmoothed)
                Log.i(
                    TAG,
                    "Profile detected: $numBuckets buckets, " +
                            "min=${"%.1f".format(minV)}° max=${"%.1f".format(maxV)}° " +
                            "mean=${"%.1f".format(meanV)}° " +
                            "segmentation=${segmentationElapsedMs}ms total=${totalElapsedMs}ms"
                )

                return Result(profile, debugInfo)
            } finally {
                obstacleMask.release()
            }
        } finally {
            skyMask.release()
        }
    }

    // -- Pipeline steps preservados del detector v1 -------------------------

    /**
     * Toma la máscara de cielo (255=cielo, 0=no-cielo) y devuelve una máscara
     * donde solo quedan los píxeles no-cielo pertenecientes a componentes
     * conectadas al borde inferior.
     */
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
            result[x] = if (horizonY < 0) 0f else AtlasMath.yToAltitude(horizonY, cfg)
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
        cfg: AtlasConfig
    ): Bitmap {
        val out = atlasBitmap.copy(Bitmap.Config.ARGB_8888, true)
        val w = out.width
        val h = out.height
        val canvas = Canvas(out)

        // Capa 1: tinte rojo translúcido en obstáculos.
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

        // Capa 2: polilínea verde del horizonte detectado.
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

        return out
    }
}