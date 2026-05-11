package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Color
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.FrameRecord
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Fase 2: Multi-band blending sobre el atlas equirectangular.
 *
 * Implementación manual de Burt-Adelson (1983) porque el módulo
 * cv::detail::MultiBandBlender no está expuesto en los bindings Java
 * del AAR de OpenCV Android.
 *
 * ============================================================
 * ALGORITMO
 * ============================================================
 * Para cada frame H0/H45:
 *   1. Warpear al espacio del atlas equirectangular completo
 *   2. Construir pirámide gaussiana de la máscara de peso
 *   3. Construir pirámide laplaciana de la imagen (premultiplicada
 *      por la máscara para que pyrDown no sangre desde el background
 *      negro)
 *   4. Acumular incremental en buffers globales banda por banda:
 *        accumImg[b]  += L_img[b] * G_mask[b]   (channel-wise)
 *        accumMask[b] += G_mask[b]
 *
 * Al final de todos los frames:
 *   5. Normalizar cada banda: blended[b] = accumImg[b] / accumMask[b]
 *   6. Reconstruir desde la cima:
 *        result = blended[N] + pyrUp(blended[N-1] + pyrUp(... + blended[0]))
 *   7. Copiar al atlas.pixels donde coverage > epsilon
 *
 * Resultado: detalles de altas frecuencias vienen mayoritariamente
 * de UN frame por zona (en vez de promediarse) → esconde ghost
 * geométrico residual.
 *
 * No toca el Z0 (esos quedan para ZenithMatrixProjector después).
 *
 * ============================================================
 * MEMORIA / TIEMPO
 * ============================================================
 * Pico estimado para atlas 3600x900, 20 frames, 4 bandas:
 *   - Acumuladores globales: ~67 MB persistentes
 *   - Pirámides por frame procesado: ~80 MB temp
 *   - Mat warped y multiplicaciones: ~60 MB temp
 *   Total pico: ~200 MB durante procesamiento.
 *
 * Si OOM, bajar DEFAULT_NUM_BANDS a 3.
 *
 * Tiempo estimado: 60-120s para 20 frames en Android moderno.
 */
object MultiBandAtlasBlender {
    // (al inicio del object MultiBandAtlasBlender)
    private const val H45_POLAR_FADE_START_ALT_DEG = 80.0
    private const val H45_POLAR_FADE_FULL_ALT_DEG = 88.0
    private const val TAG = "MultiBandAtlasBlender"

    // Menos bandas = menos memoria + menos detalle en suavizado de seams.
    // 4 es un compromiso razonable.
    private const val DEFAULT_NUM_BANDS = 4

    private const val MIN_QUALITY_WEIGHT_TO_FEED = 0.05f
    // 0.05 ≈ 5% de cobertura efectiva. Píxeles abajo de esto son típicamente
    // propagación gaussiana desde bordes de frames vecinos (sin info real
    // del frame), no deben escribirse al atlas porque contaminan la franja
    // de transición H45→Z0 con datos warpeados deformados.
    private const val MIN_COVERAGE_FOR_OUTPUT = 0.05f

    // Quality weight constants (mismo criterio H0/H45 que AtlasProjector)
    private const val QW_MAX_AZ_ERR = 8f
    private const val QW_MAX_PITCH_ERR = 6f
    private const val QW_MAX_ROLL_ERR = 8f

    fun blendH0H45Frames(
        frames: List<FrameRecord>,
        atlas: SkyAtlas,
        numBands: Int = DEFAULT_NUM_BANDS
    ): Boolean {
        return try {
            SunProjectApp.requireOpenCv()

            val validFrames = frames.filter {
                !isZenithFrame(it) &&
                        it.hfovDeg != null &&
                        it.vfovDeg != null &&
                        it.imageWidthPx != null &&
                        it.imageHeightPx != null
            }
            if (validFrames.isEmpty()) {
                Log.w(TAG, "no valid H0/H45 frames")
                return false
            }

            val effectiveBands = computeMaxBands(atlas.width, atlas.height, numBands)

            Log.i(
                TAG,
                "starting: frames=${validFrames.size} bands=$effectiveBands " +
                        "atlas=${atlas.width}x${atlas.height}"
            )

            val tStart = System.currentTimeMillis()
            val fedCount = doBlend(validFrames, atlas, effectiveBands)
            val elapsedMs = System.currentTimeMillis() - tStart

            Log.i(TAG, "finished: fedFrames=$fedCount elapsedMs=$elapsedMs")
            fedCount > 0
        } catch (t: Throwable) {
            Log.e(TAG, "multi-band crashed, falling back", t)
            false
        }
    }

    private fun computeMaxBands(w: Int, h: Int, requested: Int): Int {
        // Each pyrDown halves dimensions; stop when smaller side < 8 px
        var bands = requested
        var ww = w
        var hh = h
        for (i in 1..requested) {
            ww = (ww + 1) / 2
            hh = (hh + 1) / 2
            if (ww < 8 || hh < 8) {
                bands = i - 1
                break
            }
        }
        return bands.coerceAtLeast(1)
    }

    private fun doBlend(frames: List<FrameRecord>, atlas: SkyAtlas, numBands: Int): Int {
        val atlasW = atlas.width
        val atlasH = atlas.height

        // Precompute pyramid sizes
        val pyramidSizes = mutableListOf<Size>()
        var pw = atlasW
        var ph = atlasH
        pyramidSizes.add(Size(pw.toDouble(), ph.toDouble()))
        for (i in 1..numBands) {
            pw = (pw + 1) / 2
            ph = (ph + 1) / 2
            pyramidSizes.add(Size(pw.toDouble(), ph.toDouble()))
        }

        // Allocate global accumulators (numBands+1 levels)
        val accumImg = Array(numBands + 1) { i ->
            val s = pyramidSizes[i]
            Mat.zeros(s.height.toInt(), s.width.toInt(), CvType.CV_32FC3)
        }
        val accumMask = Array(numBands + 1) { i ->
            val s = pyramidSizes[i]
            Mat.zeros(s.height.toInt(), s.width.toInt(), CvType.CV_32FC1)
        }

        var fedCount = 0

        try {
            for (frame in frames) {
                val qw = qualityWeightForFrame(frame)
                if (qw < MIN_QUALITY_WEIGHT_TO_FEED) {
                    Log.d(TAG, "skip frame=${frame.frameId} qw=${"%.3f".format(qw)}")
                    continue
                }

                val srcBitmap = BitmapFactory.decodeFile(frame.originalPath) ?: continue
                try {
                    val warp = warpFrameToAtlas(frame, srcBitmap, atlas, qw)
                    if (warp == null) continue
                    try {
                        accumulateFrame(warp.imgF, warp.maskF, accumImg, accumMask, numBands)
                        fedCount++
                        Log.d(TAG, "fed frame=${frame.frameId} qw=${"%.3f".format(qw)}")
                    } finally {
                        warp.imgF.release()
                        warp.maskF.release()
                    }
                } finally {
                    srcBitmap.recycle()
                }
            }

            if (fedCount == 0) {
                Log.w(TAG, "no frames fed")
                return 0
            }

            // Save final coverage (level 0 mask) before normalizing
            val coverageMask = Mat()
            accumMask[0].copyTo(coverageMask)

            // Normalize each band
            val blended = Array(numBands + 1) { b ->
                normalizeBand(accumImg[b], accumMask[b])
            }

            // Reconstruct from top
            val result = reconstructFromPyramid(blended)

            blended.forEach { it.release() }

            try {
                copyResultToAtlas(result, coverageMask, atlas)
            } finally {
                result.release()
                coverageMask.release()
            }

            return fedCount
        } finally {
            accumImg.forEach { it.release() }
            accumMask.forEach { it.release() }
        }
    }

    private fun accumulateFrame(
        imgF: Mat,
        maskF: Mat,
        accumImg: Array<Mat>,
        accumMask: Array<Mat>,
        numBands: Int
    ) {
        // 1. Premultiply image by mask
        val premult = multiplyImage3CByMask1C(imgF, maskF)

        // 2. Gaussian pyramid of mask
        val gMask = Array(numBands + 1) { Mat() }
        maskF.copyTo(gMask[0])
        for (k in 1..numBands) {
            Imgproc.pyrDown(gMask[k - 1], gMask[k])
        }

        // 3. Gaussian pyramid of premultiplied image
        val gImg = Array(numBands + 1) { Mat() }
        premult.copyTo(gImg[0])
        premult.release()
        for (k in 1..numBands) {
            Imgproc.pyrDown(gImg[k - 1], gImg[k])
        }

        // 4. Laplacian pyramid: L[k] = G[k] - pyrUp(G[k+1]) for k<N, L[N]=G[N]
        val lImg = Array(numBands + 1) { Mat() }
        for (k in 0 until numBands) {
            val up = Mat()
            Imgproc.pyrUp(gImg[k + 1], up, gImg[k].size())
            Core.subtract(gImg[k], up, lImg[k])
            up.release()
        }
        gImg[numBands].copyTo(lImg[numBands])

        // Release G pyramid of image
        gImg.forEach { it.release() }

        // 5. Accumulate per band. lImg[b] viene de la pirámide de
        //    img*mask, así que la máscara ya está "pesada" en cada
        //    banda. NO se multiplica de nuevo por gMask[b] acá —
        //    eso sería doble premultiplicación y oscurece el atlas.
        for (b in 0..numBands) {
            Core.add(accumImg[b], lImg[b], accumImg[b])
            Core.add(accumMask[b], gMask[b], accumMask[b])
        }

        // Release frame's pyramids
        lImg.forEach { it.release() }
        gMask.forEach { it.release() }
    }

    private fun normalizeBand(accumImg: Mat, accumMask: Mat): Mat {
        // Add small epsilon to mask to avoid division by zero
        val safeMask = Mat()
        Core.add(accumMask, Scalar(1e-6), safeMask)

        val result = Mat()
        divideImage3CByMask1C(accumImg, safeMask, result)
        safeMask.release()
        return result
    }

    private fun reconstructFromPyramid(blended: Array<Mat>): Mat {
        var current = Mat()
        blended[blended.size - 1].copyTo(current)

        for (b in blended.size - 2 downTo 0) {
            val up = Mat()
            Imgproc.pyrUp(current, up, blended[b].size())
            current.release()
            current = Mat()
            Core.add(blended[b], up, current)
            up.release()
        }

        return current
    }

    private fun multiplyImage3CByMask1C(img3c: Mat, mask1c: Mat): Mat {
        val mask3c = Mat()
        val channels = listOf(mask1c, mask1c, mask1c)
        Core.merge(channels, mask3c)
        val result = Mat()
        Core.multiply(img3c, mask3c, result)
        mask3c.release()
        return result
    }

    private fun divideImage3CByMask1C(img3c: Mat, mask1c: Mat, dst: Mat) {
        val mask3c = Mat()
        val channels = listOf(mask1c, mask1c, mask1c)
        Core.merge(channels, mask3c)
        Core.divide(img3c, mask3c, dst)
        mask3c.release()
    }

    private data class WarpResult(val imgF: Mat, val maskF: Mat)

    private fun warpFrameToAtlas(
        frame: FrameRecord,
        srcBitmap: Bitmap,
        atlas: SkyAtlas,
        qualityWeight: Float
    ): WarpResult? {
        val atlasW = atlas.width
        val atlasH = atlas.height
        val srcW = srcBitmap.width
        val srcH = srcBitmap.height
        val srcPixels = IntArray(srcW * srcH)
        srcBitmap.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val yawDeg = frame.measuredAzimuthDeg.toDouble()
        val pitchDeg = (frame.absPitchDeg ?: frame.measuredPitchDeg).toDouble()
        val rollDeg = (frame.absRollDeg ?: frame.measuredRollDeg).toDouble()

        val basis = buildBasis(yawDeg, pitchDeg, rollDeg)
        val forward = basis.first
        val right = basis.second
        val up = basis.third

        val hfovDeg = frame.hfovDeg!!.toDouble()
        val vfovDeg = frame.vfovDeg!!.toDouble()
        val tanHalfH = tan(Math.toRadians(hfovDeg / 2.0))
        val tanHalfV = tan(Math.toRadians(vfovDeg / 2.0))

        val imgF = Mat.zeros(atlasH, atlasW, CvType.CV_32FC3)
        val maskF = Mat.zeros(atlasH, atlasW, CvType.CV_32FC1)

        val imgRow = FloatArray(atlasW * 3)
        val maskRow = FloatArray(atlasW)

        var coveredPx = 0L

        for (y in 0 until atlasH) {
            for (i in imgRow.indices) imgRow[i] = 0f
            for (i in maskRow.indices) maskRow[i] = 0f

            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            val altRad = Math.toRadians(altDeg.toDouble())
            val cosAlt = cos(altRad)
            val sinAlt = sin(altRad)

            // Polar fade idéntico al path píxel-a-píxel (Cambio A en AtlasProjector).
            // Apaga H45 al acercarse al polo para no contaminar la cap donde Z0 debe dominar.
            val polarFade: Double = run {
                val t = ((altDeg.toDouble() - H45_POLAR_FADE_START_ALT_DEG) /
                        (H45_POLAR_FADE_FULL_ALT_DEG - H45_POLAR_FADE_START_ALT_DEG))
                    .coerceIn(0.0, 1.0)
                // smoothstep inverso: peso=1 abajo de 80°, peso=0 arriba de 88°
                1.0 - (t * t * (3.0 - 2.0 * t))
            }
            if (polarFade <= 0.0) continue  // saltea la fila entera arriba de 88°

            for (x in 0 until atlasW) {
                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                val azRad = Math.toRadians(azDeg.toDouble())

                val dirX = cosAlt * sin(azRad)
                val dirY = cosAlt * cos(azRad)
                val dirZ = sinAlt

                val camX = dirX * right[0] + dirY * right[1] + dirZ * right[2]
                val camY = dirX * up[0] + dirY * up[1] + dirZ * up[2]
                val camZ = dirX * forward[0] + dirY * forward[1] + dirZ * forward[2]

                if (camZ <= 0.0) continue

                val nx = (camX / camZ) / tanHalfH
                val ny = (camY / camZ) / tanHalfV

                if (abs(nx) > 1.0 || abs(ny) > 1.0) continue

                val u = ((nx + 1.0) * 0.5) * (srcW - 1)
                val v = (1.0 - (ny + 1.0) * 0.5) * (srcH - 1)

                val color = bilinearSample(srcPixels, srcW, srcH, u.toFloat(), v.toFloat())

                val r = Color.red(color) / 255f
                val g = Color.green(color) / 255f
                val b = Color.blue(color) / 255f

                // OpenCV native order is BGR
                imgRow[x * 3] = b
                imgRow[x * 3 + 1] = g
                imgRow[x * 3 + 2] = r

                val wx = (1.0 - abs(nx)).coerceIn(0.0, 1.0)
                val wy = (1.0 - abs(ny)).coerceIn(0.0, 1.0)
                val localWeight = wx * wy

                val maskValue = (localWeight * qualityWeight.toDouble() * polarFade)
                    .toFloat().coerceIn(0f, 1f)
                if (maskValue <= 0f) continue
                maskRow[x] = maskValue
                coveredPx++
            }

            imgF.put(y, 0, imgRow)
            maskF.put(y, 0, maskRow)
        }

        if (coveredPx == 0L) {
            imgF.release()
            maskF.release()
            return null
        }

        return WarpResult(imgF, maskF)
    }

    private fun copyResultToAtlas(result: Mat, coverageMask: Mat, atlas: SkyAtlas) {
        val w = atlas.width
        val h = atlas.height

        // Convert result CV_32FC3 (range [0,1]) to CV_8UC3
        val result8U = Mat()
        result.convertTo(result8U, CvType.CV_8U, 255.0)

        val imgRow = ByteArray(w * 3)
        val maskRow = FloatArray(w)

        var written = 0L

        for (y in 0 until h) {
            result8U.get(y, 0, imgRow)
            coverageMask.get(y, 0, maskRow)

            for (x in 0 until w) {
                if (maskRow[x] < MIN_COVERAGE_FOR_OUTPUT) continue

                val b = imgRow[x * 3].toInt() and 0xFF
                val g = imgRow[x * 3 + 1].toInt() and 0xFF
                val r = imgRow[x * 3 + 2].toInt() and 0xFF

                val color = Color.argb(255, r, g, b)
                // Atlas weightSums=0 inicialmente → blendPixel setea directo
                atlas.blendPixel(x, y, color, 1.0f)
                written++
            }
        }

        result8U.release()
        Log.d(TAG, "copied result: writtenPx=$written")
    }

    // ============================================================
    // GEOMETRÍA — réplica de AtlasProjector.buildProjectionBasisFromAngles
    // ============================================================

    private fun buildBasis(
        yawDeg: Double, pitchDeg: Double, rollDeg: Double
    ): Triple<DoubleArray, DoubleArray, DoubleArray> {
        val yawRad = Math.toRadians(yawDeg)
        val pitchRad = Math.toRadians(pitchDeg)
        val rollRad = Math.toRadians(rollDeg)

        val cp = cos(pitchRad)
        val sp = sin(pitchRad)
        val forward = doubleArrayOf(cp * sin(yawRad), cp * cos(yawRad), sp)

        var rx = forward[1]
        var ry = -forward[0]
        var rz = 0.0
        var rNorm = sqrt(rx * rx + ry * ry + rz * rz)
        if (rNorm < 1e-4) {
            rx = 1.0; ry = 0.0; rz = 0.0; rNorm = 1.0
        }
        val right0 = doubleArrayOf(rx / rNorm, ry / rNorm, rz / rNorm)

        val u0x = right0[1] * forward[2] - right0[2] * forward[1]
        val u0y = right0[2] * forward[0] - right0[0] * forward[2]
        val u0z = right0[0] * forward[1] - right0[1] * forward[0]
        val u0Norm = sqrt(u0x * u0x + u0y * u0y + u0z * u0z).coerceAtLeast(1e-12)
        val up0 = doubleArrayOf(u0x / u0Norm, u0y / u0Norm, u0z / u0Norm)

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val rrx = right0[0] * cosR + up0[0] * sinR
        val rry = right0[1] * cosR + up0[1] * sinR
        val rrz = right0[2] * cosR + up0[2] * sinR
        val rrN = sqrt(rrx * rrx + rry * rry + rrz * rrz).coerceAtLeast(1e-12)
        val right = doubleArrayOf(rrx / rrN, rry / rrN, rrz / rrN)

        val uux = up0[0] * cosR - right0[0] * sinR
        val uuy = up0[1] * cosR - right0[1] * sinR
        val uuz = up0[2] * cosR - right0[2] * sinR
        val uuN = sqrt(uux * uux + uuy * uuy + uuz * uuz).coerceAtLeast(1e-12)
        val up = doubleArrayOf(uux / uuN, uuy / uuN, uuz / uuN)

        return Triple(forward, right, up)
    }

    private fun bilinearSample(pixels: IntArray, width: Int, height: Int, fx: Float, fy: Float): Int {
        val x0 = fx.toInt().coerceIn(0, width - 1)
        val y0 = fy.toInt().coerceIn(0, height - 1)
        val x1 = (x0 + 1).coerceIn(0, width - 1)
        val y1 = (y0 + 1).coerceIn(0, height - 1)

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val c00 = pixels[y0 * width + x0]
        val c10 = pixels[y0 * width + x1]
        val c01 = pixels[y1 * width + x0]
        val c11 = pixels[y1 * width + x1]

        val r = bilerp(
            Color.red(c00).toFloat(), Color.red(c10).toFloat(),
            Color.red(c01).toFloat(), Color.red(c11).toFloat(),
            dx, dy
        ).toInt().coerceIn(0, 255)
        val g = bilerp(
            Color.green(c00).toFloat(), Color.green(c10).toFloat(),
            Color.green(c01).toFloat(), Color.green(c11).toFloat(),
            dx, dy
        ).toInt().coerceIn(0, 255)
        val b = bilerp(
            Color.blue(c00).toFloat(), Color.blue(c10).toFloat(),
            Color.blue(c01).toFloat(), Color.blue(c11).toFloat(),
            dx, dy
        ).toInt().coerceIn(0, 255)

        return Color.argb(255, r, g, b)
    }

    private fun bilerp(v00: Float, v10: Float, v01: Float, v11: Float, dx: Float, dy: Float): Float {
        val top = v00 * (1f - dx) + v10 * dx
        val bottom = v01 * (1f - dx) + v11 * dx
        return top * (1f - dy) + bottom * dy
    }

    private fun qualityWeightForFrame(frame: FrameRecord): Float {
        val azErr = shortestAngleDeltaAbs(frame.measuredAzimuthDeg, frame.targetAzimuthDeg)
        val pitchBase = frame.absPitchDeg ?: frame.measuredPitchDeg
        val pitchErr = abs(frame.targetPitchDeg - pitchBase)
        val rollBase = frame.absRollDeg ?: frame.measuredRollDeg
        val rollAbs = abs(rollBase)

        if (azErr > QW_MAX_AZ_ERR) return 0f
        if (pitchErr > QW_MAX_PITCH_ERR) return 0f
        if (rollAbs > QW_MAX_ROLL_ERR) return 0f

        val azScore = (1f - azErr / QW_MAX_AZ_ERR).coerceIn(0f, 1f)
        val pitchScore = (1f - pitchErr / QW_MAX_PITCH_ERR).coerceIn(0f, 1f)
        val rollScore = (1f - rollAbs / QW_MAX_ROLL_ERR).coerceIn(0f, 1f)

        return 0.15f + 0.85f * azScore * pitchScore * rollScore
    }

    private fun shortestAngleDeltaAbs(a: Float, b: Float): Float {
        var d = (a - b) % 360f
        if (d > 180f) d -= 360f
        if (d < -180f) d += 360f
        return abs(d)
    }

    private fun isZenithFrame(frame: FrameRecord): Boolean {
        return frame.targetPitchDeg >= 80f || frame.ringId.contains("Z", ignoreCase = true)
    }
}