package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.Color
import com.example.sunproject.data.model.FrameRecord
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

object ZenithTopFaceRefiner {

    private const val FACE_SIZE_PX = 512
    private const val ANNULUS_MIN_ALT_DEG = 72f
    private const val ANNULUS_MAX_ALT_DEG = 78f
    private const val ECC_MIN_ALT_DEG = 68f
    private const val ECC_MAX_ALT_DEG = 88f
    private const val BLEND_MIN_ALT_DEG = 68f

    data class TopFace(
        val rgba: Mat,
        val gray32: Mat,
        val validMask: Mat,
        val faceSizePx: Int
    )

    data class RefinementResult(
        val initialTwistDeg: Float,
        val finalTwistDeg: Float,
        val eccRotationDeg: Float,
        val eccTxPx: Float,
        val eccTyPx: Float,
        val eccScore: Double,
        val alignedTopFace: TopFace
    )

    fun refineAndBlendZenithIntoAtlas(
        atlas: SkyAtlas,
        frame: FrameRecord,
        srcBitmap: Bitmap,
        frameWeight: Float
    ): RefinementResult? {
        if (frameWeight <= 0f) return null

        val atlasTop = buildAtlasTopFace(atlas, FACE_SIZE_PX)
        val zenithTop = buildZenithTopFace(frame, srcBitmap, baseTwistDeg = 0f, faceSizePx = FACE_SIZE_PX)

        val initialTwistDeg = estimateTwistFromAnnulus(
            reference = atlasTop,
            moving = zenithTop,
            minAltitudeDeg = ANNULUS_MIN_ALT_DEG,
            maxAltitudeDeg = ANNULUS_MAX_ALT_DEG
        )

        val refined = refineWithEcc(
            reference = atlasTop,
            moving = zenithTop,
            initialTwistDeg = initialTwistDeg,
            eccMinAltitudeDeg = ECC_MIN_ALT_DEG,
            eccMaxAltitudeDeg = ECC_MAX_ALT_DEG
        )

        blendTopFaceIntoAtlas(
            aligned = refined.alignedTopFace,
            atlas = atlas,
            frameWeight = frameWeight,
            minAltitudeDeg = BLEND_MIN_ALT_DEG
        )

        releaseTopFace(atlasTop)
        releaseTopFace(zenithTop)

        return refined
    }

    fun buildAtlasTopFace(
        atlas: SkyAtlas,
        faceSizePx: Int = FACE_SIZE_PX
    ): TopFace {
        val argb = IntArray(faceSizePx * faceSizePx)
        val mask = ByteArray(faceSizePx * faceSizePx)

        val center = (faceSizePx - 1) * 0.5f
        val radius = center

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dir = topFaceDirection(x, y, center, radius)
                val azDeg = normalizeDeg(radToDeg(atan2(dir[1], dir[0])))
                val altDeg = radToDeg(atan2(dir[2], sqrt(dir[0] * dir[0] + dir[1] * dir[1])))

                val atlasX = AtlasMath.azimuthToX(azDeg, atlas.config).coerceIn(0, atlas.width - 1)
                val atlasY = AtlasMath.altitudeToY(altDeg, atlas.config).coerceIn(0, atlas.height - 1)

                val idx = y * faceSizePx + x
                if (atlas.hasCoverageAt(atlasX, atlasY)) {
                    argb[idx] = atlas.pixels[atlas.index(atlasX, atlasY)]
                    mask[idx] = 0xFF.toByte()
                } else {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                }
            }
        }

        return topFaceFromArgbAndMask(argb, mask, faceSizePx)
    }

    fun buildZenithTopFace(
        frame: FrameRecord,
        srcBitmap: Bitmap,
        baseTwistDeg: Float,
        faceSizePx: Int = FACE_SIZE_PX
    ): TopFace {
        val srcW = srcBitmap.width
        val srcH = srcBitmap.height
        val srcPixels = IntArray(srcW * srcH)
        srcBitmap.getPixels(srcPixels, 0, srcW, 0, 0, srcW, srcH)

        val hfov = (frame.hfovDeg ?: 65f).coerceIn(1f, 179f)
        val vfov = (frame.vfovDeg ?: 50f).coerceIn(1f, 179f)

        val tanHalfH = tan(Math.toRadians(hfov * 0.5).toFloat())
        val tanHalfV = tan(Math.toRadians(vfov * 0.5).toFloat())

        val yawDeg = normalizeDeg(frame.targetAzimuthDeg + baseTwistDeg)
        val pitchDeg = frame.measuredPitchDeg.coerceIn(84f, 90f)
        val rollDeg = frame.measuredRollDeg

        val yawRad = degToRad(yawDeg)
        val pitchRad = degToRad(pitchDeg)
        val rollRad = degToRad(-rollDeg)

        val forward = worldDirectionRad(yawRad, pitchRad)

        var right0 = cross(forward, floatArrayOf(0f, 0f, 1f))
        if (length(right0) < 1e-4f) {
            right0 = floatArrayOf(cos(yawRad), sin(yawRad), 0f)
        }
        right0 = normalize(right0)

        var up0 = cross(right0, forward)
        up0 = normalize(up0)

        val cosR = cos(rollRad)
        val sinR = sin(rollRad)

        val right = normalize(add(scale(right0, cosR), scale(up0, sinR)))
        val up = normalize(add(scale(up0, cosR), scale(right0, -sinR)))

        val argb = IntArray(faceSizePx * faceSizePx)
        val mask = ByteArray(faceSizePx * faceSizePx)

        val center = (faceSizePx - 1) * 0.5f
        val radius = center

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dir = topFaceDirection(x, y, center, radius)

                val camX = dot(dir, right)
                val camY = dot(dir, up)
                val camZ = dot(dir, forward)

                val idx = y * faceSizePx + x
                if (camZ <= 0f) {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                    continue
                }

                val nx = (camX / camZ) / tanHalfH
                val ny = (camY / camZ) / tanHalfV

                if (abs(nx) > 1f || abs(ny) > 1f) {
                    argb[idx] = Color.TRANSPARENT
                    mask[idx] = 0
                    continue
                }

                val u = (((nx + 1f) * 0.5f) * (srcW - 1)).roundToInt().coerceIn(0, srcW - 1)
                val v = ((1f - ((ny + 1f) * 0.5f)) * (srcH - 1)).roundToInt().coerceIn(0, srcH - 1)

                argb[idx] = srcPixels[v * srcW + u]
                mask[idx] = 0xFF.toByte()
            }
        }

        return topFaceFromArgbAndMask(argb, mask, faceSizePx)
    }

    fun estimateTwistFromAnnulus(
        reference: TopFace,
        moving: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int = 360
    ): Float {
        val refSig = buildAnnulusSignature(reference, minAltitudeDeg, maxAltitudeDeg, angleBins)
        val movSig = buildAnnulusSignature(moving, minAltitudeDeg, maxAltitudeDeg, angleBins)

        var bestShift = 0
        var bestScore = Float.NEGATIVE_INFINITY

        for (shift in 0 until angleBins) {
            var num = 0.0
            var denA = 0.0
            var denB = 0.0

            for (i in 0 until angleBins) {
                val a = refSig[i]
                val b = movSig[(i + shift) % angleBins]
                num += a * b
                denA += a * a
                denB += b * b
            }

            val score =
                if (denA > 1e-9 && denB > 1e-9) {
                    (num / sqrt(denA * denB)).toFloat()
                } else {
                    Float.NEGATIVE_INFINITY
                }

            if (score > bestScore) {
                bestScore = score
                bestShift = shift
            }
        }

        return normalizeDeg(bestShift.toFloat())
    }

    fun refineWithEcc(
        reference: TopFace,
        moving: TopFace,
        initialTwistDeg: Float,
        eccMinAltitudeDeg: Float,
        eccMaxAltitudeDeg: Float
    ): RefinementResult {
        val center = Point((moving.faceSizePx - 1) * 0.5, (moving.faceSizePx - 1) * 0.5)

        val rotM = Imgproc.getRotationMatrix2D(center, initialTwistDeg.toDouble(), 1.0)

        val rotatedRgba = Mat()
        val rotatedMask = Mat()
        val rotatedGray = Mat()

        Imgproc.warpAffine(
            moving.rgba,
            rotatedRgba,
            rotM,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_LINEAR,
            Core.BORDER_CONSTANT,
            Scalar(0.0, 0.0, 0.0, 0.0)
        )

        Imgproc.warpAffine(
            moving.validMask,
            rotatedMask,
            rotM,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_NEAREST,
            Core.BORDER_CONSTANT,
            Scalar(0.0)
        )

        Imgproc.cvtColor(rotatedRgba, rotatedGray, Imgproc.COLOR_RGBA2GRAY)
        rotatedGray.convertTo(rotatedGray, CvType.CV_32F, 1.0 / 255.0)

        val altitudeMask = buildAltitudeMask(
            faceSizePx = moving.faceSizePx,
            minAltitudeDeg = eccMinAltitudeDeg,
            maxAltitudeDeg = eccMaxAltitudeDeg
        )

        val eccMask = Mat()
        Core.bitwise_and(reference.validMask, rotatedMask, eccMask)
        Core.bitwise_and(eccMask, altitudeMask, eccMask)

        val warp = Mat.eye(2, 3, CvType.CV_32F)
        val criteria = TermCriteria(
            TermCriteria.COUNT + TermCriteria.EPS,
            50,
            1e-4
        )

        val eccScore = try {
            Video.findTransformECC(
                reference.gray32,
                rotatedGray,
                warp,
                Video.MOTION_EUCLIDEAN,
                criteria,
                eccMask,
                5
            )
        } catch (_: Throwable) {
            0.0
        }

        val alignedRgba = Mat()
        val alignedMask = Mat()
        val alignedGray = Mat()

        Imgproc.warpAffine(
            rotatedRgba,
            alignedRgba,
            warp,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_LINEAR + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0, 0.0, 0.0, 0.0)
        )

        Imgproc.warpAffine(
            rotatedMask,
            alignedMask,
            warp,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_NEAREST + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0)
        )

        Imgproc.cvtColor(alignedRgba, alignedGray, Imgproc.COLOR_RGBA2GRAY)
        alignedGray.convertTo(alignedGray, CvType.CV_32F, 1.0 / 255.0)

        val m00 = warp.get(0, 0)[0]
        val m01 = warp.get(0, 1)[0]
        val tx = warp.get(0, 2)[0]
        val ty = warp.get(1, 2)[0]
        val eccRotDeg = Math.toDegrees(atan2(m01, m00).toDouble()).toFloat()

        rotM.release()
        rotatedRgba.release()
        rotatedMask.release()
        rotatedGray.release()
        altitudeMask.release()
        eccMask.release()
        warp.release()

        return RefinementResult(
            initialTwistDeg = initialTwistDeg,
            finalTwistDeg = normalizeDeg(initialTwistDeg + eccRotDeg),
            eccRotationDeg = eccRotDeg,
            eccTxPx = tx.toFloat(),
            eccTyPx = ty.toFloat(),
            eccScore = eccScore,
            alignedTopFace = TopFace(
                rgba = alignedRgba,
                gray32 = alignedGray,
                validMask = alignedMask,
                faceSizePx = moving.faceSizePx
            )
        )
    }

    fun blendTopFaceIntoAtlas(
        aligned: TopFace,
        atlas: SkyAtlas,
        frameWeight: Float,
        minAltitudeDeg: Float
    ) {
        val pixels = rgbaMatToArgb(aligned.rgba)
        val center = (aligned.faceSizePx - 1) * 0.5f
        val radius = center

        for (y in 0 until aligned.faceSizePx) {
            for (x in 0 until aligned.faceSizePx) {
                val idx = y * aligned.faceSizePx + x
                val valid = aligned.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                if (valid <= 0) continue

                val dir = topFaceDirection(x, y, center, radius)
                val altDeg = radToDeg(atan2(dir[2], sqrt(dir[0] * dir[0] + dir[1] * dir[1])))
                if (altDeg < minAltitudeDeg) continue

                val azDeg = normalizeDeg(radToDeg(atan2(dir[1], dir[0])))

                val atlasX = AtlasMath.azimuthToX(azDeg, atlas.config).coerceIn(0, atlas.width - 1)
                val atlasY = AtlasMath.altitudeToY(altDeg, atlas.config).coerceIn(0, atlas.height - 1)

                atlas.blendPixel(
                    atlasX,
                    atlasY,
                    pixels[idx],
                    frameWeight
                )
            }
        }
    }

    fun releaseTopFace(face: TopFace) {
        face.rgba.release()
        face.gray32.release()
        face.validMask.release()
    }

    private fun buildAnnulusSignature(
        face: TopFace,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float,
        angleBins: Int
    ): FloatArray {
        val signature = FloatArray(angleBins)

        val center = (face.faceSizePx - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToTopRadiusPx(maxAltitudeDeg, maxRadius)
        val outerRadius = altitudeToTopRadiusPx(minAltitudeDeg, maxRadius)

        for (bin in 0 until angleBins) {
            val theta = (2.0 * PI * bin.toDouble()) / angleBins.toDouble()

            var sum = 0.0
            var count = 0

            var r = innerRadius
            while (r <= outerRadius + 1e-3f) {
                val x = (center + r * cos(theta).toFloat()).roundToInt().coerceIn(0, face.faceSizePx - 1)
                val y = (center + r * sin(theta).toFloat()).roundToInt().coerceIn(0, face.faceSizePx - 1)

                val valid = face.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                if (valid > 0) {
                    sum += face.gray32.get(y, x)[0]
                    count++
                }
                r += 2f
            }

            signature[bin] = if (count > 0) (sum / count).toFloat() else 0f
        }

        val mean = signature.average().toFloat()
        var varSum = 0f
        for (i in signature.indices) {
            signature[i] -= mean
            varSum += signature[i] * signature[i]
        }
        val std = sqrt((varSum / signature.size).coerceAtLeast(1e-9f))
        for (i in signature.indices) {
            signature[i] /= std
        }

        return signature
    }

    private fun buildAltitudeMask(
        faceSizePx: Int,
        minAltitudeDeg: Float,
        maxAltitudeDeg: Float
    ): Mat {
        val mask = Mat.zeros(faceSizePx, faceSizePx, CvType.CV_8UC1)

        val center = (faceSizePx - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToTopRadiusPx(maxAltitudeDeg, maxRadius)
        val outerRadius = altitudeToTopRadiusPx(minAltitudeDeg, maxRadius)

        for (y in 0 until faceSizePx) {
            for (x in 0 until faceSizePx) {
                val dx = x - center
                val dy = y - center
                val r = sqrt(dx * dx + dy * dy)
                if (r in innerRadius..outerRadius) {
                    mask.put(y, x, 255.0)
                }
            }
        }

        return mask
    }

    private fun topFaceFromArgbAndMask(
        argb: IntArray,
        mask: ByteArray,
        faceSizePx: Int
    ): TopFace {
        val bmp = Bitmap.createBitmap(faceSizePx, faceSizePx, Bitmap.Config.ARGB_8888)
        bmp.setPixels(argb, 0, faceSizePx, 0, 0, faceSizePx, faceSizePx)

        val rgba = Mat()
        Utils.bitmapToMat(bmp, rgba)

        val gray32 = Mat()
        Imgproc.cvtColor(rgba, gray32, Imgproc.COLOR_RGBA2GRAY)
        gray32.convertTo(gray32, CvType.CV_32F, 1.0 / 255.0)

        val validMask = Mat(faceSizePx, faceSizePx, CvType.CV_8UC1)
        validMask.put(0, 0, mask)

        return TopFace(
            rgba = rgba,
            gray32 = gray32,
            validMask = validMask,
            faceSizePx = faceSizePx
        )
    }

    private fun rgbaMatToArgb(rgba: Mat): IntArray {
        val width = rgba.cols()
        val height = rgba.rows()
        val raw = ByteArray(width * height * 4)
        rgba.get(0, 0, raw)

        val out = IntArray(width * height)
        var j = 0
        for (i in out.indices) {
            val r = raw[j].toInt() and 0xFF
            val g = raw[j + 1].toInt() and 0xFF
            val b = raw[j + 2].toInt() and 0xFF
            val a = raw[j + 3].toInt() and 0xFF
            out[i] = Color.argb(a, r, g, b)
            j += 4
        }
        return out
    }

    private fun altitudeToTopRadiusPx(altitudeDeg: Float, maxRadius: Float): Float {
        val zenithAngleDeg = 90f - altitudeDeg
        return tan(degToRad(zenithAngleDeg)) * maxRadius
    }

    private fun topFaceDirection(
        x: Int,
        y: Int,
        center: Float,
        radius: Float
    ): FloatArray {
        val px = (x - center) / radius
        val py = (y - center) / radius
        return normalize(floatArrayOf(px, py, 1f))
    }

    private fun degToRad(value: Float): Float = Math.toRadians(value.toDouble()).toFloat()
    private fun radToDeg(value: Float): Float = Math.toDegrees(value.toDouble()).toFloat()

    private fun normalizeDeg(value: Float): Float {
        var v = value % 360f
        if (v < 0f) v += 360f
        return v
    }

    private fun worldDirectionRad(yawRad: Float, pitchRad: Float): FloatArray {
        val cp = cos(pitchRad)
        val sp = sin(pitchRad)
        val cy = cos(yawRad)
        val sy = sin(yawRad)
        return floatArrayOf(cp * cy, cp * sy, sp)
    }

    private fun dot(a: FloatArray, b: FloatArray): Float =
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun cross(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        )

    private fun length(v: FloatArray): Float = sqrt(dot(v, v))

    private fun normalize(v: FloatArray): FloatArray {
        val n = length(v).coerceAtLeast(1e-6f)
        return floatArrayOf(v[0] / n, v[1] / n, v[2] / n)
    }

    private fun scale(v: FloatArray, s: Float): FloatArray =
        floatArrayOf(v[0] * s, v[1] * s, v[2] * s)

    private fun add(a: FloatArray, b: FloatArray): FloatArray =
        floatArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])
}