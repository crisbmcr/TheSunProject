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
import android.util.Log

object ZenithTopFaceRefiner {

    private const val FACE_SIZE_PX = 512
    private const val ANNULUS_MIN_ALT_DEG = 72f
    private const val ANNULUS_MAX_ALT_DEG = 78f
    private const val ECC_MIN_ALT_DEG = 68f
    private const val ECC_MAX_ALT_DEG = 88f
    private const val BLEND_MIN_ALT_DEG = 68f

    private const val MAX_ECC_TRANSLATION_RATIO = 0.02f
    private const val MIN_ECC_SCORE_TO_USE_TRANSLATION = 0.30

    private const val BLEND_FEATHER_START_ALT_DEG = 82f
    private const val BLEND_FEATHER_FULL_ALT_DEG = 89f

    private const val RGB_GAIN_MIN = 0.92f
    private const val RGB_GAIN_MAX = 1.08f

    private const val POLAR_CAP_FILL_MIN_ALT_DEG = 86f
    private const val POLAR_CAP_NEAREST_RADIUS_PX = 4
    private const val POLAR_CAP_FILL_WEIGHT = 2.0f

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
    private data class ColorCorrection(
        val rGain: Float,
        val gGain: Float,
        val bGain: Float,
        val overlapCount: Int
    )

    private data class TopFaceSample(
        val color: Int,
        val usedFallback: Boolean
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

        try {
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

            val colorCorrection = blendTopFaceIntoAtlas(
                aligned = refined.alignedTopFace,
                atlas = atlas,
                frameWeight = frameWeight,
                minAltitudeDeg = BLEND_MIN_ALT_DEG
            )

            fillUncoveredPolarCapFromTopFace(
                aligned = refined.alignedTopFace,
                atlas = atlas,
                frameWeight = frameWeight,
                minAltitudeDeg = POLAR_CAP_FILL_MIN_ALT_DEG,
                colorCorrection = colorCorrection
            )

            return refined
        } finally {
            releaseTopFace(atlasTop)
            releaseTopFace(zenithTop)
        }
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

        val m00 = warp.get(0, 0)[0]
        val m01 = warp.get(0, 1)[0]

        val tx = warp.get(0, 2)[0].toFloat()
        val ty = warp.get(1, 2)[0].toFloat()

        val eccRotDeg = Math.toDegrees(atan2(m01, m00).toDouble()).toFloat()

        val maxShiftPx = moving.faceSizePx.toFloat() * MAX_ECC_TRANSLATION_RATIO
        val allowTranslation = eccScore >= MIN_ECC_SCORE_TO_USE_TRANSLATION

        val txUsed: Float = if (allowTranslation) {
            tx.coerceIn(-maxShiftPx, maxShiftPx)
        } else {
            0f
        }

        val tyUsed: Float = if (allowTranslation) {
            ty.coerceIn(-maxShiftPx, maxShiftPx)
        } else {
            0f
        }

        val warpUsed = Mat.eye(2, 3, CvType.CV_32F)
        warpUsed.put(0, 0, m00)
        warpUsed.put(0, 1, m01)
        warpUsed.put(1, 0, warp.get(1, 0)[0])
        warpUsed.put(1, 1, warp.get(1, 1)[0])
        warpUsed.put(0, 2, txUsed.toDouble())
        warpUsed.put(1, 2, tyUsed.toDouble())

        val alignedRgba = Mat()
        val alignedMask = Mat()
        val alignedGray = Mat()

        Imgproc.warpAffine(
            rotatedRgba,
            alignedRgba,
            warpUsed,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_LINEAR + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0, 0.0, 0.0, 0.0)
        )

        Imgproc.warpAffine(
            rotatedMask,
            alignedMask,
            warpUsed,
            Size(moving.faceSizePx.toDouble(), moving.faceSizePx.toDouble()),
            Imgproc.INTER_NEAREST + Imgproc.WARP_INVERSE_MAP,
            Core.BORDER_CONSTANT,
            Scalar(0.0)
        )

        Imgproc.cvtColor(alignedRgba, alignedGray, Imgproc.COLOR_RGBA2GRAY)
        alignedGray.convertTo(alignedGray, CvType.CV_32F, 1.0 / 255.0)

        rotM.release()
        rotatedRgba.release()
        rotatedMask.release()
        rotatedGray.release()
        altitudeMask.release()
        eccMask.release()
        warp.release()
        warpUsed.release()

        return RefinementResult(
            initialTwistDeg = initialTwistDeg,
            finalTwistDeg = normalizeDeg(initialTwistDeg + eccRotDeg),
            eccRotationDeg = eccRotDeg,
            eccTxPx = txUsed,
            eccTyPx = tyUsed,
            eccScore = eccScore,
            alignedTopFace = TopFace(
                rgba = alignedRgba,
                gray32 = alignedGray,
                validMask = alignedMask,
                faceSizePx = moving.faceSizePx
            )
        )
    }

    private fun blendTopFaceIntoAtlas(
        aligned: TopFace,
        atlas: SkyAtlas,
        frameWeight: Float,
        minAltitudeDeg: Float
    ): ColorCorrection {
        val topPixels = rgbaMatToArgb(aligned.rgba)
        val correction = computeOverlapColorCorrection(
            aligned = aligned,
            atlas = atlas,
            facePixels = topPixels,
            minAltitudeDeg = minAltitudeDeg
        )

        var coveredWrites = 0
        var uncoveredWrites = 0
        var nullSamples = 0

        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg) continue

            for (x in 0 until atlas.width) {
                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceBilinear(
                    face = aligned,
                    facePixels = topPixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                )

                if (sampled == null) {
                    nullSamples++
                    continue
                }

                val hasBaseCoverage = atlas.hasCoverageAt(x, y)

                val altitudeAlpha = if (hasBaseCoverage) {
                    val t = ((altDeg - BLEND_FEATHER_START_ALT_DEG) /
                            (BLEND_FEATHER_FULL_ALT_DEG - BLEND_FEATHER_START_ALT_DEG))
                        .coerceIn(0f, 1f)
                    t * t * (3f - 2f * t)
                } else {
                    1f
                }

                val finalWeight = frameWeight * altitudeAlpha
                if (finalWeight <= 0f) continue

                val corrected = applyRgbGain(
                    sampled,
                    correction.rGain,
                    correction.gGain,
                    correction.bGain
                )

                atlas.blendPixel(x, y, corrected, finalWeight)

                if (hasBaseCoverage) coveredWrites++ else uncoveredWrites++
            }
        }

        Log.d(
            "AtlasZenithBlend",
            "minAlt=${"%.1f".format(minAltitudeDeg)} " +
                    "overlap=${correction.overlapCount} " +
                    "gainR=${"%.3f".format(correction.rGain)} " +
                    "gainG=${"%.3f".format(correction.gGain)} " +
                    "gainB=${"%.3f".format(correction.bGain)} " +
                    "coveredWrites=$coveredWrites " +
                    "uncoveredWrites=$uncoveredWrites " +
                    "nullSamples=$nullSamples"
        )

        return correction
    }

    private fun computeOverlapColorCorrection(
        aligned: TopFace,
        atlas: SkyAtlas,
        facePixels: IntArray,
        minAltitudeDeg: Float
    ): ColorCorrection {
        var refRSum = 0f
        var refGSum = 0f
        var refBSum = 0f

        var movRSum = 0f
        var movGSum = 0f
        var movBSum = 0f

        var overlapCount = 0

        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg) continue

            for (x in 0 until atlas.width) {
                if (!atlas.hasCoverageAt(x, y)) continue

                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceBilinear(
                    face = aligned,
                    facePixels = facePixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                ) ?: continue

                val refColor = atlas.pixels[atlas.index(x, y)]

                refRSum += Color.red(refColor)
                refGSum += Color.green(refColor)
                refBSum += Color.blue(refColor)

                movRSum += Color.red(sampled)
                movGSum += Color.green(sampled)
                movBSum += Color.blue(sampled)

                overlapCount++
            }
        }

        val rGain = if (overlapCount > 0 && movRSum > 1e-3f) {
            (refRSum / movRSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        val gGain = if (overlapCount > 0 && movGSum > 1e-3f) {
            (refGSum / movGSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        val bGain = if (overlapCount > 0 && movBSum > 1e-3f) {
            (refBSum / movBSum).coerceIn(RGB_GAIN_MIN, RGB_GAIN_MAX)
        } else 1f

        return ColorCorrection(
            rGain = rGain,
            gGain = gGain,
            bGain = bGain,
            overlapCount = overlapCount
        )
    }

    private fun fillUncoveredPolarCapFromTopFace(
        aligned: TopFace,
        atlas: SkyAtlas,
        frameWeight: Float,
        minAltitudeDeg: Float,
        colorCorrection: ColorCorrection
    ) {
        if (frameWeight <= 0f) return

        val topPixels = rgbaMatToArgb(aligned.rgba)
        val yBottom = AtlasMath.altitudeToY(minAltitudeDeg, atlas.config).coerceIn(0, atlas.height - 1)

        var candidateCount = 0
        var filledCount = 0
        var directCount = 0
        var fallbackCount = 0
        var missedCount = 0

        for (y in 0..yBottom) {
            val altDeg = AtlasMath.yToAltitude(y, atlas.config)
            if (altDeg < minAltitudeDeg) continue

            for (x in 0 until atlas.width) {
                if (atlas.hasCoverageAt(x, y)) continue

                candidateCount++

                val azDeg = AtlasMath.xToAzimuth(x, atlas.config)

                val sampled = sampleTopFaceWithFallback(
                    face = aligned,
                    facePixels = topPixels,
                    azDeg = azDeg,
                    altDeg = altDeg
                )

                if (sampled == null) {
                    missedCount++
                    continue
                }

                val corrected = applyRgbGain(
                    sampled.color,
                    colorCorrection.rGain,
                    colorCorrection.gGain,
                    colorCorrection.bGain
                )

                atlas.blendPixel(x, y, corrected, frameWeight)
                filledCount++

                if (sampled.usedFallback) fallbackCount++ else directCount++
            }
        }

        Log.d(
            "AtlasZenithFill",
            "minAlt=${"%.1f".format(minAltitudeDeg)} " +
                    "candidates=$candidateCount " +
                    "filled=$filledCount " +
                    "direct=$directCount " +
                    "fallback=$fallbackCount " +
                    "missed=$missedCount"
        )
    }

    private fun sampleTopFaceWithFallback(
        face: TopFace,
        facePixels: IntArray,
        azDeg: Float,
        altDeg: Float
    ): TopFaceSample? {
        val coords = projectWorldToTopFace(face, azDeg, altDeg) ?: return null

        val direct = sampleTopFaceBilinearAt(
            face = face,
            facePixels = facePixels,
            fx = coords.first,
            fy = coords.second
        )

        if (direct != null) {
            return TopFaceSample(
                color = direct,
                usedFallback = false
            )
        }

        val fallback = sampleNearestValidTopFace(
            face = face,
            facePixels = facePixels,
            fx = coords.first,
            fy = coords.second,
            maxRadiusPx = POLAR_CAP_NEAREST_RADIUS_PX
        ) ?: return null

        return TopFaceSample(
            color = fallback,
            usedFallback = true
        )
    }

    private fun projectWorldToTopFace(
        face: TopFace,
        azDeg: Float,
        altDeg: Float
    ): Pair<Float, Float>? {
        val dir = worldDirectionRad(
            degToRad(azDeg),
            degToRad(altDeg)
        )

        if (dir[2] <= 1e-6f) return null

        val center = (face.faceSizePx - 1) * 0.5f
        val radius = center

        val fx = center + radius * (dir[0] / dir[2])
        val fy = center + radius * (dir[1] / dir[2])

        if (fx < 0f || fy < 0f || fx > face.faceSizePx - 1f || fy > face.faceSizePx - 1f) {
            return null
        }

        return fx to fy
    }

    private fun sampleNearestValidTopFace(
        face: TopFace,
        facePixels: IntArray,
        fx: Float,
        fy: Float,
        maxRadiusPx: Int
    ): Int? {
        val cx = fx.roundToInt().coerceIn(0, face.faceSizePx - 1)
        val cy = fy.roundToInt().coerceIn(0, face.faceSizePx - 1)

        for (radius in 1..maxRadiusPx) {
            var bestColor: Int? = null
            var bestD2 = Float.POSITIVE_INFINITY

            val left = (cx - radius).coerceAtLeast(0)
            val right = (cx + radius).coerceAtMost(face.faceSizePx - 1)
            val top = (cy - radius).coerceAtLeast(0)
            val bottom = (cy + radius).coerceAtMost(face.faceSizePx - 1)

            for (y in top..bottom) {
                for (x in left..right) {
                    if (x != left && x != right && y != top && y != bottom) continue

                    val valid = face.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                    if (valid <= 0) continue

                    val dx = x - fx
                    val dy = y - fy
                    val d2 = dx * dx + dy * dy

                    if (d2 < bestD2) {
                        bestD2 = d2
                        bestColor = facePixels[y * face.faceSizePx + x]
                    }
                }
            }

            if (bestColor != null) return bestColor
        }

        return null
    }

    private fun sampleTopFaceBilinear(
        face: TopFace,
        facePixels: IntArray,
        azDeg: Float,
        altDeg: Float
    ): Int? {
        val coords = projectWorldToTopFace(face, azDeg, altDeg) ?: return null

        return sampleTopFaceBilinearAt(
            face = face,
            facePixels = facePixels,
            fx = coords.first,
            fy = coords.second
        )
    }

    private fun sampleTopFaceBilinearAt(
        face: TopFace,
        facePixels: IntArray,
        fx: Float,
        fy: Float
    ): Int? {
        val x0 = fx.toInt().coerceIn(0, face.faceSizePx - 1)
        val y0 = fy.toInt().coerceIn(0, face.faceSizePx - 1)
        val x1 = (x0 + 1).coerceIn(0, face.faceSizePx - 1)
        val y1 = (y0 + 1).coerceIn(0, face.faceSizePx - 1)

        val m00 = face.validMask.get(y0, x0)?.firstOrNull()?.toInt() ?: 0
        val m10 = face.validMask.get(y0, x1)?.firstOrNull()?.toInt() ?: 0
        val m01 = face.validMask.get(y1, x0)?.firstOrNull()?.toInt() ?: 0
        val m11 = face.validMask.get(y1, x1)?.firstOrNull()?.toInt() ?: 0

        val dx = (fx - x0).coerceIn(0f, 1f)
        val dy = (fy - y0).coerceIn(0f, 1f)

        val w00 = (1f - dx) * (1f - dy)
        val w10 = dx * (1f - dy)
        val w01 = (1f - dx) * dy
        val w11 = dx * dy

        val c00 = facePixels[y0 * face.faceSizePx + x0]
        val c10 = facePixels[y0 * face.faceSizePx + x1]
        val c01 = facePixels[y1 * face.faceSizePx + x0]
        val c11 = facePixels[y1 * face.faceSizePx + x1]

        var sumW = 0f
        var sumA = 0f
        var sumR = 0f
        var sumG = 0f
        var sumB = 0f

        fun acc(mask: Int, weight: Float, color: Int) {
            if (mask <= 0 || weight <= 0f) return
            sumW += weight
            sumA += Color.alpha(color) * weight
            sumR += Color.red(color) * weight
            sumG += Color.green(color) * weight
            sumB += Color.blue(color) * weight
        }

        acc(m00, w00, c00)
        acc(m10, w10, c10)
        acc(m01, w01, c01)
        acc(m11, w11, c11)

        if (sumW <= 1e-6f) return null

        val a = (sumA / sumW).roundToInt().coerceIn(0, 255)
        val r = (sumR / sumW).roundToInt().coerceIn(0, 255)
        val g = (sumG / sumW).roundToInt().coerceIn(0, 255)
        val b = (sumB / sumW).roundToInt().coerceIn(0, 255)

        return Color.argb(a, r, g, b)
    }

    private fun applyRgbGain(
        color: Int,
        rGain: Float,
        gGain: Float,
        bGain: Float
    ): Int {
        val a = Color.alpha(color)
        val r = (Color.red(color) * rGain).roundToInt().coerceIn(0, 255)
        val g = (Color.green(color) * gGain).roundToInt().coerceIn(0, 255)
        val b = (Color.blue(color) * bGain).roundToInt().coerceIn(0, 255)
        return Color.argb(a, r, g, b)
    }

    private fun colorLuma(color: Int): Float {
        val r = Color.red(color).toFloat()
        val g = Color.green(color).toFloat()
        val b = Color.blue(color).toFloat()
        return 0.299f * r + 0.587f * g + 0.114f * b
    }

    private fun applyLumaGain(color: Int, gain: Float): Int {
        val a = Color.alpha(color)
        val r = (Color.red(color) * gain).roundToInt().coerceIn(0, 255)
        val g = (Color.green(color) * gain).roundToInt().coerceIn(0, 255)
        val b = (Color.blue(color) * gain).roundToInt().coerceIn(0, 255)
        return Color.argb(a, r, g, b)
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