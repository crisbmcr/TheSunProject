package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Color
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.FrameRecord
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.stitching.Detail_MultiBandBlender
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Fase 2: Multi-band blending sobre el atlas equirectangular.
 *
 * ============================================================
 * QUÉ HACE
 * ============================================================
 * Reemplaza el path píxel-a-píxel para H0/H45 con un blending tipo
 * Burt-Adelson via cv::detail::MultiBandBlender de OpenCV:
 *
 *   1. Para cada frame, warpea la imagen al espacio del atlas
 *      equirectangular (proyección directa, no inversa).
 *   2. Genera máscara de peso por píxel con feathering radial lineal
 *      modulado por el quality weight del frame.
 *   3. Alimenta al blender con (imagen 16SC3, máscara 8UC1, top_left).
 *   4. Blend final → escribe al atlas.pixels.
 *
 * El blender descompone cada frame en una pirámide laplaciana y la
 * máscara en una pirámide gaussiana. Las bandas bajas se mezclan
 * con feathering amplio; las altas con feathering estrecho. Resultado:
 * los detalles de altas frecuencias vienen mayoritariamente de UN
 * frame por zona, en vez de promediarse → esconde geometric ghost.
 *
 * ============================================================
 * QUÉ NO HACE
 * ============================================================
 * - No toca el Z0 (lo escribe ZenithMatrixProjector después del blend).
 * - No corrige geometría real: si dos frames están misalineados 2°,
 *   el contenido sigue en lugares distintos, pero el seam visible es
 *   menos abrupto que con weighted average puro.
 * - No corrige white balance ni exposure global entre frames (eso lo
 *   sigue haciendo estimateFrameRgbGain en AtlasProjector si querés
 *   integrarlo después). El blender opera sobre los colores ya como
 *   vienen del frame.
 *
 * ============================================================
 * MEMORIA
 * ============================================================
 * Pico estimado para atlas 3600x900 con 20 frames y 5 bandas:
 *   - Warp por frame: 19.4 MB (CV_16SC3) + 3.2 MB (mask) = 22.6 MB
 *   - Pirámides internas del blender: ~30 MB (5 bandas, CV_16SC3)
 *   - Total pico: ~75 MB durante el blend
 * Aceptable en devices ≥ 4 GB RAM. Si OOM en devices low-end, hay
 * que optimizar usando bounding rects en lugar de atlas completo.
 */
object MultiBandAtlasBlender {

    private const val TAG = "MultiBandAtlasBlender"

    private const val DEFAULT_NUM_BANDS = 5
    private const val MIN_QUALITY_WEIGHT_TO_FEED = 0.05f

    // Quality weight constants — mismo criterio H0/H45 que AtlasProjector
    private const val QW_MAX_AZ_ERR = 8f
    private const val QW_MAX_PITCH_ERR = 6f
    private const val QW_MAX_ROLL_ERR = 8f

    /**
     * Renderiza los frames H0/H45 al atlas usando multi-band blending.
     * Retorna true si tuvo éxito; false para fallback al path píxel-a-píxel.
     */
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

            Log.i(
                TAG,
                "starting multi-band: frames=${validFrames.size} bands=$numBands " +
                        "atlas=${atlas.width}x${atlas.height}"
            )

            val tStart = System.currentTimeMillis()
            val fedCount = doBlend(validFrames, atlas, numBands)
            val elapsedMs = System.currentTimeMillis() - tStart

            Log.i(TAG, "multi-band finished: fedFrames=$fedCount elapsedMs=$elapsedMs")
            fedCount > 0
        } catch (t: Throwable) {
            Log.e(TAG, "multi-band blending crashed, falling back", t)
            false
        }
    }

    private fun doBlend(frames: List<FrameRecord>, atlas: SkyAtlas, numBands: Int): Int {
        val atlasW = atlas.width
        val atlasH = atlas.height

        // tryGpu=0, weight_type=CV_16S (más rápido y menos memoria que CV_32F)
        val blender = Detail_MultiBandBlender(0, numBands, CvType.CV_16S)
        blender.prepare(Rect(0, 0, atlasW, atlasH))

        var fedCount = 0

        for (frame in frames) {
            val qw = qualityWeightForFrame(frame)
            if (qw < MIN_QUALITY_WEIGHT_TO_FEED) {
                Log.d(TAG, "skip frame=${frame.frameId} qw=${"%.3f".format(qw)} below threshold")
                continue
            }

            val srcBitmap = BitmapFactory.decodeFile(frame.originalPath)
            if (srcBitmap == null) {
                Log.w(TAG, "decode failed: ${frame.originalPath}")
                continue
            }

            try {
                val warped = warpFrameToAtlas(frame, srcBitmap, atlas, qw)
                if (warped == null) {
                    Log.w(TAG, "warp produced no coverage for frame=${frame.frameId}")
                    continue
                }
                try {
                    blender.feed(warped.img16S, warped.mask8U, warped.topLeft)
                    fedCount++
                    Log.d(
                        TAG,
                        "fed frame=${frame.frameId} ring=${frame.ringId} " +
                                "qw=${"%.3f".format(qw)} covered=${warped.coveredPx}px"
                    )
                } finally {
                    warped.img16S.release()
                    warped.mask8U.release()
                }
            } finally {
                srcBitmap.recycle()
            }
        }

        if (fedCount == 0) {
            Log.w(TAG, "no frames fed to blender; aborting")
            return 0
        }

        val result16S = Mat()
        val resultMask = Mat()
        try {
            blender.blend(result16S, resultMask)
            val result8U = Mat()
            try {
                result16S.convertTo(result8U, CvType.CV_8U)
                copyBlendResultToAtlas(result8U, resultMask, atlas)
            } finally {
                result8U.release()
            }
        } finally {
            result16S.release()
            resultMask.release()
        }

        return fedCount
    }

    private data class WarpResult(
        val img16S: Mat,
        val mask8U: Mat,
        val topLeft: Point,
        val coveredPx: Long
    )

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

        // Misma convención de poses que AtlasProjector: yaw=measured, pitch/roll=abs
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

        val warpedImg8U = Mat(atlasH, atlasW, CvType.CV_8UC3, Scalar(0.0, 0.0, 0.0))
        val mask8U = Mat(atlasH, atlasW, CvType.CV_8UC1, Scalar(0.0))

        val rowBytes = atlasW * 3
        val imgBuffer = ByteArray(rowBytes)
        val maskBuffer = ByteArray(atlasW)

        var coveredPx = 0L

        for (y in 0 until atlasH) {
            // reset row buffers
            for (i in imgBuffer.indices) imgBuffer[i] = 0
            for (i in maskBuffer.indices) maskBuffer[i] = 0

            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            val altRad = Math.toRadians(altDeg.toDouble())
            val cosAlt = cos(altRad)
            val sinAlt = sin(altRad)

            for (x in 0 until atlasW) {
                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)
                val azRad = Math.toRadians(azDeg.toDouble())

                // World direction (ENU): same as AtlasProjector.worldDirectionRad
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

                val r = Color.red(color)
                val g = Color.green(color)
                val b = Color.blue(color)

                // OpenCV native order is BGR for CV_8UC3
                imgBuffer[x * 3] = b.toByte()
                imgBuffer[x * 3 + 1] = g.toByte()
                imgBuffer[x * 3 + 2] = r.toByte()

                // Feathering radial LINEAL (no cuadrático). El blender va
                // a propagar la máscara via pirámide gaussiana, así que el
                // perfil exacto importa menos — pero lineal da más zona
                // de overlap útil que cuadrático.
                val wx = (1.0 - abs(nx)).coerceIn(0.0, 1.0)
                val wy = (1.0 - abs(ny)).coerceIn(0.0, 1.0)
                val localWeight = wx * wy

                val maskVal = (localWeight * qualityWeight.toDouble() * 255.0)
                    .toInt().coerceIn(1, 255)
                maskBuffer[x] = maskVal.toByte()

                coveredPx++
            }

            warpedImg8U.put(y, 0, imgBuffer)
            mask8U.put(y, 0, maskBuffer)
        }

        if (coveredPx == 0L) {
            warpedImg8U.release()
            mask8U.release()
            return null
        }

        val img16S = Mat()
        warpedImg8U.convertTo(img16S, CvType.CV_16S)
        warpedImg8U.release()

        return WarpResult(img16S, mask8U, Point(0.0, 0.0), coveredPx)
    }

    private fun copyBlendResultToAtlas(result8U: Mat, mask: Mat, atlas: SkyAtlas) {
        val w = atlas.width
        val h = atlas.height

        val imgBuffer = ByteArray(w * 3)
        val maskBuffer = ByteArray(w)

        var written = 0L

        for (y in 0 until h) {
            result8U.get(y, 0, imgBuffer)
            mask.get(y, 0, maskBuffer)

            for (x in 0 until w) {
                val maskV = maskBuffer[x].toInt() and 0xFF
                if (maskV == 0) continue

                val b = imgBuffer[x * 3].toInt() and 0xFF
                val g = imgBuffer[x * 3 + 1].toInt() and 0xFF
                val r = imgBuffer[x * 3 + 2].toInt() and 0xFF

                val color = Color.argb(255, r, g, b)
                // Como el atlas todavía no tiene cobertura (weightSums=0),
                // blendPixel setea directamente pixel + weight=1.0.
                atlas.blendPixel(x, y, color, 1.0f)
                written++
            }
        }

        Log.d(TAG, "copied result to atlas: writtenPx=$written")
    }

    // ============================================================
    // GEOMETRÍA — réplica exacta de AtlasProjector.buildProjectionBasisFromAngles
    // (branch zenithLike=false, sin hard zenith)
    // ============================================================

    private fun buildBasis(
        yawDeg: Double, pitchDeg: Double, rollDeg: Double
    ): Triple<DoubleArray, DoubleArray, DoubleArray> {
        val yawRad = Math.toRadians(yawDeg)
        val pitchRad = Math.toRadians(pitchDeg)
        val rollRad = Math.toRadians(rollDeg)

        val cp = cos(pitchRad)
        val sp = sin(pitchRad)
        val forward = doubleArrayOf(
            cp * sin(yawRad),
            cp * cos(yawRad),
            sp
        )

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