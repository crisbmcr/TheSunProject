package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.FrameRecord
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * Proyecta el frame Z0 al atlas equirectangular usando la matriz 3x3 M
 * (R_world<-device) directamente, sin descomponer a Euler.
 *
 * Pipeline (inverse remap):
 *   atlas (u_eq, v_eq) -> rayo d_world en ENU
 *     -> rayo en device frame via M^T
 *     -> rayo en OpenCV cam frame via B (* R_disp)
 *     -> proyeccion pinhole a (u_src, v_src)
 *   Se arma un LUT CV_32FC2 y se llama a cv::remap una sola vez.
 *
 * Convenciones:
 *   - ENU: +X=Este, +Y=Norte, +Z=Arriba. Azimut CW desde el Norte.
 *   - Device (Android): +X=derecha, +Y=arriba pantalla, +Z=saliendo.
 *   - OpenCV cam: +X=derecha imagen, +Y=abajo imagen, +Z=hacia la escena.
 *   - Camara trasera en portrait: eje optico = -Z_device.
 *     => B = diag(+1, -1, -1) mapea device frame a OpenCV cam frame.
 *
 * Aca NO se usa absAzimuthDeg/absPitchDeg/absRollDeg del frame; solo la
 * matriz. Esa es la razon de ser de este archivo.
 */
object ZenithMatrixProjector {

    // Peso maximo que un Z0 puede aportar por pixel.
    //
    // Subido de 0.40 a 1.0 (sin cap efectivo). Razón: con 0.40 el Z0
    // contribuye solo ~28% del color cuando se solapa con H45 (peso 1.0)
    // en la zona alta del polo, lo que blanqueaba el Z0 al promediarse
    // con bordes deformados de H45 que también llegan a esa zona.
    //
    // El fade por altitud (ZENITH_FADE_START_ALT_DEG=74°, FULL=82°) se
    // mantiene: el Z0 sigue sin escribir abajo de 74° y entra suave
    // entre 74°-82°, así que no invade la zona donde H45 es la fuente
    // correcta. Por encima de 82° el Z0 ahora puede dominar totalmente.
    //
    // No se tocan los H45 ni el path projectBitmapToAtlas.
    private const val ZENITH_MAX_WEIGHT = 1.0f

    // Fade por altitud. Debajo de 74deg el Z0 no escribe nada; entre 74
    // y 82 entra suave; arriba de 82 escribe con peso pleno.
    private const val ZENITH_FADE_START_ALT_DEG = 74f
    private const val ZENITH_FADE_FULL_ALT_DEG = 82f

    /**
     * Construye el LUT inverso y la mascara de validez para todo el atlas.
     *
     * @param rWorldDevice matriz 3x3 row-major (tal como viene de FrameRecord).
     * @param displayRotation Surface.ROTATION_0..270. Solo 0 soportado por ahora.
     * @return Pair(lut CV_32FC2 [H x W x (u,v)], mask CV_8U [H x W], 255=valido)
     */
    fun buildEquirectLut(
        rWorldDevice: FloatArray,
        hfovDeg: Float,
        vfovDeg: Float,
        srcW: Int,
        srcH: Int,
        atlas: SkyAtlas,
        displayRotation: Int,
        yawCorrectionDeg: Float = 0f
    ): Pair<Mat, Mat> {
        require(rWorldDevice.size == 9) { "rWorldDevice debe tener 9 floats" }
        require(displayRotation == 0) {
            "ZenithMatrixProjector: solo ROTATION_0 soportado (llego $displayRotation)"
        }

        val w = atlas.width
        val h = atlas.height
        val lut = Mat(h, w, CvType.CV_32FC2)
        val mask = Mat.zeros(h, w, CvType.CV_8UC1)

        // Parametros intrinsecos pinhole
        val fx = 0.5f * srcW / tan(Math.toRadians(hfovDeg / 2.0)).toFloat()
        val fy = 0.5f * srcH / tan(Math.toRadians(vfovDeg / 2.0)).toFloat()
        val cx = srcW * 0.5f - 0.5f
        val cy = srcH * 0.5f - 0.5f

        // Precompute sin/cos de azimut una sola vez (compartido entre filas)
        val sinAz = FloatArray(w)
        val cosAz = FloatArray(w)
        for (x in 0 until w) {
            val azRad = Math.toRadians(AtlasMath.xToAzimuth(x, atlas.config).toDouble()).toFloat()
            sinAz[x] = sin(azRad)
            cosAz[x] = cos(azRad)
        }

        // Aliases para leer M con nombres (row-major: M[row*3+col])
        val m00 = rWorldDevice[0]; val m01 = rWorldDevice[1]; val m02 = rWorldDevice[2]
        val m10 = rWorldDevice[3]; val m11 = rWorldDevice[4]; val m12 = rWorldDevice[5]
        val m20 = rWorldDevice[6]; val m21 = rWorldDevice[7]; val m22 = rWorldDevice[8]

        val lutRow = FloatArray(w * 2)
        val maskRow = ByteArray(w)
        var validCount = 0
        // FIX DECLINACIÓN: precalcular rotación en azimut para alinear con true-N
        val correctionRad = Math.toRadians(yawCorrectionDeg.toDouble()).toFloat()
        val cosCorr = cos(correctionRad)
        val sinCorr = sin(correctionRad)

        Log.d(
            "ZenithMatrix",
            "buildEquirectLut yawCorrection=${"%.2f".format(yawCorrectionDeg)}° " +
                    "srcW=$srcW srcH=$srcH displayRotation=$displayRotation"
        )
        for (y in 0 until h) {
            val altRad = Math.toRadians(
                AtlasMath.yToAltitude(y, atlas.config).toDouble()
            ).toFloat()
            val cosAlt = cos(altRad)
            val sinAlt = sin(altRad)

            for (x in 0 until w) {
                // Rayo d_world en ENU (azimut CW desde el Norte).
                // Rayo d_world en ENU (azimut CW desde Norte).
                val dwxRaw = cosAlt * sinAz[x]
                val dwyRaw = cosAlt * cosAz[x]
                val dwz = sinAlt

// FIX DECLINACIÓN: rotar el rayo por -yawCorrection alrededor del eje Z
// para que coincida con H0/H45. Preserva altitud.
                val dwx =  dwxRaw * cosCorr + dwyRaw * sinCorr
                val dwy = -dwxRaw * sinCorr + dwyRaw * cosCorr

// d_device = M^T * d_world.
                val ddx = m00 * dwx + m10 * dwy + m20 * dwz
                val ddy = m01 * dwx + m11 * dwy + m21 * dwz
                val ddz = m02 * dwx + m12 * dwy + m22 * dwz

                // d_cam = B * d_device, con B = diag(+1, -1, -1).
                // TODO: R_disp para ROTATION_90/180/270 si alguna vez
                // se capturan Z0 con la pantalla no vertical.
                val xc = ddx
                val yc = -ddy
                val zc = -ddz

                val pairBase = x * 2

                if (zc <= 1e-6f) {
                    // Rayo por detras de la camara o tangente al plano
                    // imagen: invalido.
                    lutRow[pairBase] = -1f
                    lutRow[pairBase + 1] = -1f
                    maskRow[x] = 0
                    continue
                }

                val uSrc = fx * xc / zc + cx
                val vSrc = fy * yc / zc + cy

                if (uSrc < 0f || uSrc > srcW - 1f || vSrc < 0f || vSrc > srcH - 1f) {
                    lutRow[pairBase] = -1f
                    lutRow[pairBase + 1] = -1f
                    maskRow[x] = 0
                } else {
                    lutRow[pairBase] = uSrc
                    lutRow[pairBase + 1] = vSrc
                    maskRow[x] = 255.toByte()
                    validCount++
                }
            }

            lut.put(y, 0, lutRow)
            mask.put(y, 0, maskRow)
        }

        Log.d(
            "ZenithMatrix",
            "LUT built: valid=$validCount / ${w * h} " +
                    "(${"%.1f".format(100.0 * validCount / (w * h))}%) " +
                    "atlas=${w}x${h} src=${srcW}x${srcH} " +
                    "hfov=${"%.1f".format(hfovDeg)} vfov=${"%.1f".format(vfovDeg)}"
        )

        return lut to mask
    }

    /**
     * Proyecta un unico Z0 al atlas. Asume que frame es un Z0 valido con
     * matriz de rotacion almacenada; no hace validacion del ringId.
     */
    fun projectZ0ToAtlas(
        frame: FrameRecord,
        src: Bitmap,
        atlas: SkyAtlas,
        frameWeight: Float,
        displayRotation: Int
    ) {
        if (frameWeight <= 0f) return
        SunProjectApp.requireOpenCv()

        val rWorldDevice = storedMatrix(frame) ?: run {
            Log.w(
                "ZenithMatrix",
                "frame=${frame.frameId} sin matriz de rotacion; skip Z0"
            )
            return
        }

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)
        val srcW = src.width
        val srcH = src.height

        // Atlas vive en mag-N puro (estado de Fase 7). La matriz
        // rWorldDevice del Z0 viene del gyro transport heredando del
        // último H45, y vive en el mismo frame rotvec/mag-N que las
        // matrices de los H45. Como baseYawDeg de los H0/H45 también
        // proyecta en mag-N (sin sumar correctionDeg), los tres rings
        // quedan alineados automáticamente sin necesidad de aplicar
        // ninguna corrección acá.
        //
        // GyroCameraController de la vista 3D consume
        // cachedGyroToTrueNorthCorrectionDeg para llevar la cámara
        // virtual a true-N — eso afecta dónde caen los overlays solares
        // pero no toca el atlas en sí.
        val correctionDeg = 0f
        Log.d(
            "ZenithMatrix",
            "frame=${frame.frameId} applyingYawCorrection=${"%.2f".format(correctionDeg)}° " +
                    "(disabled: atlas vive en mag-N como en Fase 7)"
        )

        val (lut, mask) = buildEquirectLut(
            rWorldDevice = rWorldDevice,
            hfovDeg = hfov,
            vfovDeg = vfov,
            srcW = srcW,
            srcH = srcH,
            atlas = atlas,
            displayRotation = displayRotation,
            yawCorrectionDeg = correctionDeg
        )

        val srcMat = Mat()
        val dst = Mat()
        try {
            Utils.bitmapToMat(src, srcMat) // CV_8UC4, RGBA
            Imgproc.remap(
                srcMat, dst, lut, Mat(),
                Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT,
                Scalar(0.0, 0.0, 0.0, 0.0)
            )

            val w = atlas.width
            val h = atlas.height
            val rgba = ByteArray(w * h * 4)
            dst.get(0, 0, rgba)
            val maskBytes = ByteArray(w * h)
            mask.get(0, 0, maskBytes)

            val clampedFrameWeight = frameWeight.coerceAtMost(ZENITH_MAX_WEIGHT)
            var written = 0

            // Cap polar absoluta: en altitud >= ZENITH_FADE_FULL_ALT_DEG
            // el Z0 sobreescribe en lugar de blendear. Razón: aunque el
            // peso del Z0 (~0.7-1.0) domina en magnitud al peso de cualquier
            // H45 que llegó por overshoot pinhole (~0.01-0.05), el blend
            // canal-por-canal mezcla los tonos del H45 estirado y "tira" el
            // resultado hacia más blanco/frío. Para preservar el color
            // natural del Z0 sin retoques, usamos overwritePixel ahí.
            //
            // En la banda de fade (74°-82°) seguimos blendeando para no
            // crear un seam visible al borde del cap.
            for (y in 0 until h) {
                val altDeg = AtlasMath.yToAltitude(y, atlas.config)
                val altAlpha = smoothstep(
                    ZENITH_FADE_START_ALT_DEG,
                    ZENITH_FADE_FULL_ALT_DEG,
                    altDeg
                )
                if (altAlpha <= 0f) continue

                val rowWeight = clampedFrameWeight * altAlpha
                if (rowWeight <= 0f) continue

                val isHardZenithCap = altDeg >= ZENITH_FADE_FULL_ALT_DEG

                val rowBase = y * w
                for (x in 0 until w) {
                    if (maskBytes[rowBase + x] == 0.toByte()) continue
                    val i = (rowBase + x) * 4
                    val r = rgba[i].toInt() and 0xFF
                    val g = rgba[i + 1].toInt() and 0xFF
                    val b = rgba[i + 2].toInt() and 0xFF
                    val argb = (0xFF shl 24) or (r shl 16) or (g shl 8) or b
                    if (isHardZenithCap) {
                        atlas.overwritePixel(x, y, argb, rowWeight)
                    } else {
                        atlas.blendPixel(x, y, argb, rowWeight)
                    }
                    written++
                }
            }

            Log.i(
                "ZenithMatrix",
                "frame=${frame.frameId} written=$written " +
                        "frameWeight=${"%.3f".format(frameWeight)} " +
                        "clamped=${"%.3f".format(clampedFrameWeight)}"
            )
        } finally {
            lut.release()
            mask.release()
            srcMat.release()
            dst.release()
        }
    }

    private fun storedMatrix(frame: FrameRecord): FloatArray? {
        val m00 = frame.rotationM00 ?: return null
        val m01 = frame.rotationM01 ?: return null
        val m02 = frame.rotationM02 ?: return null
        val m10 = frame.rotationM10 ?: return null
        val m11 = frame.rotationM11 ?: return null
        val m12 = frame.rotationM12 ?: return null
        val m20 = frame.rotationM20 ?: return null
        val m21 = frame.rotationM21 ?: return null
        val m22 = frame.rotationM22 ?: return null
        return floatArrayOf(m00, m01, m02, m10, m11, m12, m20, m21, m22)
    }

    private fun smoothstep(edge0: Float, edge1: Float, x: Float): Float {
        if (edge1 <= edge0) return if (x >= edge1) 1f else 0f
        val t = ((x - edge0) / (edge1 - edge0)).coerceIn(0f, 1f)
        return t * t * (3f - 2f * t)
    }
}