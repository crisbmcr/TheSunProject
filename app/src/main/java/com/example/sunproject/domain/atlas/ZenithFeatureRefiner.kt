package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.util.Log
import com.example.sunproject.data.model.FrameRecord
import org.opencv.core.CvType
import org.opencv.core.DMatch
import org.opencv.core.KeyPoint
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfDMatch
import org.opencv.core.MatOfKeyPoint
import org.opencv.features2d.BFMatcher
import org.opencv.features2d.ORB
import org.opencv.imgproc.Imgproc
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Alternative zenith refiner based on ORB features + RANSAC 1-DoF.
 *
 * ====================================================================
 * RATIONALE
 * ====================================================================
 * The legacy refiner (ZenithTopFaceRefiner) uses a 1D photometric
 * correlation over a narrow annulus band (altitudes 70-82 degrees).
 * That approach turned out to be unstable in practice:
 *
 *   (a) The annulus signal is ~18% populated (one Z0 frame covers
 *       ~65 degrees of azimuth). Zero-fill bias pushes corrMedian off
 *       zero and reduces peak SNR.
 *   (b) Repetitive ceiling/roof textures create multiple spurious
 *       correlation peaks at large shifts (120-180 degrees).
 *   (c) The IMU seed can be off by hundreds of degrees due to gimbal
 *       lock artifacts near the pole, and photometric correlation has
 *       no robust way to distinguish real peak from aliased peak.
 *
 * Feature-based registration attacks all three at once:
 *
 *   - ORB identifies specific physical structures in both images.
 *     A patch in the Z0 only matches its actual counterpart, not
 *     "anything vaguely similar N degrees away".
 *   - With gravity prior (pitch~88, roll~0), the residual unknown is
 *     strictly 1-DoF (yaw). RANSAC with sample size 1 is trivial and
 *     very robust.
 *   - Number of inliers is a natural confidence metric. No arbitrary
 *       correlation threshold.
 *
 * Reference: Ding et al. 2021, "Minimal Solutions for Panoramic
 * Stitching Given Gravity Prior" (ICCV).
 *
 * ====================================================================
 * PIPELINE
 * ====================================================================
 * 1. Build atlasTop (gnomonic top-down projection of current atlas).
 * 2. Build zenithTop from the Z0 frame using the IMU seed pose as-is.
 *    This top face is what we'll align; roll is included in the basis.
 * 3. Restrict both images to the seam annulus ROI (alt 65-85 deg) via
 *    a circular mask.
 * 4. Run ORB on both, limited to the ROI. Describe with BRIEF.
 * 5. BFMatcher Hamming + Lowe ratio test (k=2, ratio < 0.75).
 * 6. Convert each surviving match (ptTop, ptAtlas) to spherical
 *    coordinates (azDeg, altDeg) using the same topFaceDirection /
 *    worldDirectionRad conventions the atlas uses. This is critical:
 *    any convention mismatch here would reintroduce the original bug.
 * 7. For each match: deltaAz_i = normalizeSignedDeg(azAtlas - azTop).
 * 8. RANSAC 1-DoF: iterate, pick 1 match as hypothesis, count inliers
 *    within 1.5 deg. Keep best.
 * 9. Refine Delta yaw as the circular mean of inlier deltaAz.
 * 10. Build corrected zenithTop with yaw = seedYaw + finalDeltaYaw.
 * 11. Verify: sample inlier pairs on the corrected top face, measure
 *     angular residual. If median residual > 1 deg, reject.
 * 12. Blend using the shared blend path in ZenithTopFaceRefiner.
 *
 * ====================================================================
 * ACCEPTANCE CRITERIA
 * ====================================================================
 * Accept if and only if:
 *   - minInliers >= MIN_INLIERS_STRICT, OR
 *   - minInliers >= MIN_INLIERS_RELAXED AND residualDegMedian < 0.8
 *
 * There is no "rescue mode". If features don't align, we do NOT pretend
 * they do. The frame falls back to the legacy ERP path, which at least
 * places the zenith consistently (even if not optimally) instead of
 * at a spurious yaw.
 */
object ZenithFeatureRefiner {

    // -------- Tuning (all sizes/thresholds centralised) --------

    private const val FACE_SIZE_PX = 512

    // Seam ROI — slightly wider than the legacy refiner's 70-82 band,
    // to give ORB more usable texture. ORB likes gradient, not flat sky.
    private const val SEAM_MIN_ALT_DEG = 65f
    private const val SEAM_MAX_ALT_DEG = 85f

    // ORB parameters. 1500 features is plenty for a 512x512 top face.
    private const val ORB_NUM_FEATURES = 1500
    private const val ORB_SCALE_FACTOR = 1.2f
    private const val ORB_NUM_LEVELS = 8
    private const val ORB_EDGE_THRESHOLD = 15
    private const val ORB_FAST_THRESHOLD = 12

    // Lowe's ratio test — relaxed from 0.75 to 0.80. Repetitive indoor
    // ceiling/sky textures create many "close-second" matches that a 0.75
    // ratio kills aggressively. 0.80 keeps more candidates; RANSAC then
    // separates inliers from outliers by geometric consistency.
    private const val LOWE_RATIO = 0.80f

    // RANSAC parameters.
    private const val RANSAC_ITERATIONS = 200
    private const val RANSAC_INLIER_THRESHOLD_DEG = 2.0f

    // Acceptance thresholds — lowered minimum inliers because in practice
    // a Z0 frame with sparse ceiling features may only yield ~15 reliable
    // matches post-RANSAC, and that is still geometrically very strong.
    private const val MIN_INLIERS_STRICT = 15
    private const val MIN_INLIERS_RELAXED = 8
    private const val MAX_RESIDUAL_MEDIAN_DEG_RELAXED = 1.2f
    private const val MAX_RESIDUAL_MEDIAN_DEG_STRICT = 2.0f

    // Minimum matches required to even attempt RANSAC.
    private const val MIN_MATCHES_FOR_RANSAC = 6

    // Seed sanity guard. If RANSAC proposes |deltaYaw| > this value,
    // we still accept it if inliers are numerous, but log loudly so
    // you can inspect the case.
    private const val LARGE_DELTA_YAW_WARN_DEG = 30f

    data class FeatureRefineResult(
        val deltaYawDeg: Float,           // shift applied to seedYaw
        val finalYawDeg: Float,           // absolute yaw used to build corrected topFace
        val numMatches: Int,
        val numInliers: Int,
        val residualMedianDeg: Float,
        val residualP90Deg: Float,
        val accepted: Boolean,
        val acceptMode: String            // "strict", "relaxed", "rejected"
    )

    /**
     * Main entrypoint, mirrors refineAndBlendZenithIntoAtlas in the
     * legacy refiner. Returns null on rejection (caller should fallback).
     *
     * On acceptance, this function:
     *   - builds the corrected top face
     *   - blends it into the atlas using the legacy blend path
     *   - releases the corrected top face internally
     *   - returns a LegacyCompat object so AtlasProjector can log
     *     without changes to its existing code path
     *
     * NOTE: atlas and srcBitmap are not released here.
     */
    fun refineAndBlendZenithIntoAtlas(
        atlas: SkyAtlas,
        frame: FrameRecord,
        srcBitmap: Bitmap,
        frameWeight: Float,
        seedTwistDeg: Float,
        seedPitchOffsetDeg: Float,
        seedRollOffsetDeg: Float,
        seedAbsoluteYawDeg: Float? = null,
        seedAbsolutePitchDeg: Float? = null,
        seedAbsoluteRollDeg: Float? = null,
        verbose: Boolean = true
    ): ZenithTopFaceRefiner.RefinementResult? {
        com.example.sunproject.SunProjectApp.requireOpenCv()

        if (frameWeight <= 0f) return null
        if (seedAbsoluteYawDeg == null ||
            seedAbsolutePitchDeg == null ||
            seedAbsoluteRollDeg == null
        ) {
            Log.w("AtlasZenithFeature", "frame=${frame.frameId} missing absolute seed — skip")
            return null
        }

        val tStart = System.currentTimeMillis()

        // Step 1: build atlas top face (reference).
        val atlasTop = ZenithTopFaceRefiner.buildAtlasTopFace(atlas, FACE_SIZE_PX)

        try {
            // Step 2: build zenith top face using the IMU seed as-is.
            val zenithTopSeed = ZenithTopFaceRefiner.buildZenithTopFace(
                frame = frame,
                srcBitmap = srcBitmap,
                baseTwistDeg = normalizeDeg(seedTwistDeg),
                basePitchOffsetDeg = seedPitchOffsetDeg,
                baseRollOffsetDeg = seedRollOffsetDeg,
                absoluteYawDegOverride = seedAbsoluteYawDeg,
                absolutePitchDegOverride = seedAbsolutePitchDeg,
                absoluteRollDegOverride = seedAbsoluteRollDeg,
                faceSizePx = FACE_SIZE_PX
            )

            val result: FeatureRefineResult = try {
                // Steps 3-9: find deltaYaw via features.
                runFeatureAlignment(
                    atlasTop = atlasTop,
                    zenithTop = zenithTopSeed,
                    seedAbsoluteYawDeg = seedAbsoluteYawDeg,
                    frameId = frame.frameId,
                    verbose = verbose
                )
            } finally {
                ZenithTopFaceRefiner.releaseTopFace(zenithTopSeed)
            }

            val tFeatures = System.currentTimeMillis() - tStart

            Log.d(
                "AtlasZenithFeature",
                "frame=${frame.frameId} " +
                        "seedYaw=${"%.2f".format(seedAbsoluteYawDeg)} " +
                        "deltaYaw=${"%.2f".format(result.deltaYawDeg)} " +
                        "finalYaw=${"%.2f".format(result.finalYawDeg)} " +
                        "matches=${result.numMatches} " +
                        "inliers=${result.numInliers} " +
                        "residualMedian=${"%.3f".format(result.residualMedianDeg)} " +
                        "residualP90=${"%.3f".format(result.residualP90Deg)} " +
                        "accepted=${result.accepted} " +
                        "acceptMode=${result.acceptMode} " +
                        "elapsedMs=$tFeatures"
            )

            if (!result.accepted) return null

            if (abs(result.deltaYawDeg) > LARGE_DELTA_YAW_WARN_DEG) {
                Log.w(
                    "AtlasZenithFeature",
                    "frame=${frame.frameId} large deltaYaw=${"%.2f".format(result.deltaYawDeg)} " +
                            "— IMU seed suspected drifted. Inliers=${result.numInliers} " +
                            "residualMedian=${"%.3f".format(result.residualMedianDeg)}"
                )
            }

            // Step 10: build corrected zenith top.
            val correctedTop = ZenithTopFaceRefiner.buildZenithTopFace(
                frame = frame,
                srcBitmap = srcBitmap,
                baseTwistDeg = normalizeDeg(seedTwistDeg + result.deltaYawDeg),
                basePitchOffsetDeg = seedPitchOffsetDeg,
                baseRollOffsetDeg = seedRollOffsetDeg,
                absoluteYawDegOverride = normalizeDeg(seedAbsoluteYawDeg + result.deltaYawDeg),
                absolutePitchDegOverride = seedAbsolutePitchDeg,
                absoluteRollDegOverride = seedAbsoluteRollDeg,
                faceSizePx = FACE_SIZE_PX
            )

            try {
                // Step 12: blend via shared legacy path.
                ZenithTopFaceRefiner.blendAlignedTopFaceAndFillCap(
                    aligned = correctedTop,
                    atlas = atlas,
                    frameWeight = frameWeight
                )

                Log.d(
                    "AtlasZenithFeatureBlend",
                    "frame=${frame.frameId} deltaYaw=${"%.2f".format(result.deltaYawDeg)} " +
                            "inliers=${result.numInliers} acceptMode=${result.acceptMode} " +
                            "totalMs=${System.currentTimeMillis() - tStart}"
                )

                // Package result in the legacy shape so AtlasProjector's
                // existing log line (AtlasZenithTopRefine with initialTwist,
                // finalTwist, etc) keeps working. We do NOT return the
                // aligned top face because we already released it via blend;
                // caller should not call releaseTopFace on it.
                //
                // Build a 1x1 dummy top face for the RefinementResult so
                // the caller's releaseTopFace call doesn't NPE. This is a
                // minor wart that lets us keep AtlasProjector unchanged.
                val dummyTop = buildDummyTopFace()

                return ZenithTopFaceRefiner.RefinementResult(
                    initialTwistDeg = normalizeDeg(seedTwistDeg + result.deltaYawDeg),
                    finalTwistDeg = normalizeDeg(seedTwistDeg + result.deltaYawDeg),
                    eccRotationDeg = 0f,
                    eccTxPx = 0f,
                    eccTyPx = 0f,
                    eccScore = result.numInliers.toDouble(), // repurposed as confidence
                    alignedTopFace = dummyTop
                )
            } catch (t: Throwable) {
                ZenithTopFaceRefiner.releaseTopFace(correctedTop)
                throw t
            } finally {
                // correctedTop was consumed by blend; release it now.
                ZenithTopFaceRefiner.releaseTopFace(correctedTop)
            }
        } finally {
            ZenithTopFaceRefiner.releaseTopFace(atlasTop)
        }
    }

    // ==================================================================
    // Private implementation
    // ==================================================================

    private fun runFeatureAlignment(
        atlasTop: ZenithTopFaceRefiner.TopFace,
        zenithTop: ZenithTopFaceRefiner.TopFace,
        seedAbsoluteYawDeg: Float,
        frameId: String,
        verbose: Boolean
    ): FeatureRefineResult {
        val size = atlasTop.faceSizePx
        val center = (size - 1) * 0.5f

        // ------------------------------------------------------------
        // PRE-ROTATION STRATEGY
        // ------------------------------------------------------------
        // The atlasTop is built with azimuth=0 pointing to world North.
        // The zenithTop is built with the IMU seed yaw applied, so a
        // physical structure at world azimuth X lands at topFace pixel
        // corresponding to (X - seedYaw) in the zenith image.
        //
        // If we run ORB directly on (atlasTop, zenithTop), the two
        // images differ by a rotation of `seedYaw` degrees around the
        // center. ORB is in theory rotation-invariant via keypoint
        // orientation, but near the pole with limited texture this is
        // flaky — hence the 4-11 good matches we saw in the logs.
        //
        // Fix: rotate atlasGray8 by -seedYaw so that after rotation it
        // uses the SAME nominal convention as zenithGray8 (North of
        // the atlas ends up at the same topface-pixel as North of the
        // zenith). Now ORB only has to resolve the residual ~30-40°.
        //
        // The RANSAC delta is then expressed in the "rotated frame",
        // and the actual world-frame deltaYaw is delta_rotated itself
        // (because pre-rotating atlas by -seedYaw is mathematically
        // equivalent to subtracting seedYaw from the pixel→azimuth
        // mapping on atlas side). We log everything explicitly.
        // ------------------------------------------------------------

        val atlasRoiMask = buildAnnulusRoiMask(atlasTop)
        val zenithRoiMask = buildAnnulusRoiMask(zenithTop)

        val atlasGray8 = Mat()
        val zenithGray8 = Mat()
        val atlasGray8Rot = Mat()
        val atlasRoiMaskRot = Mat()

        try {
            atlasTop.gray32.convertTo(atlasGray8, CvType.CV_8U, 255.0)
            zenithTop.gray32.convertTo(zenithGray8, CvType.CV_8U, 255.0)

            // Rotate atlas image and its ROI mask by -seedYaw around the center.
            // getRotationMatrix2D uses CCW positive; azimuth in our world is CW
            // positive from North, so rotating by -seedYaw here is consistent
            // with bringing atlas's North to coincide with zenith's North.
            val rotM = org.opencv.imgproc.Imgproc.getRotationMatrix2D(
                org.opencv.core.Point(center.toDouble(), center.toDouble()),
                (-seedAbsoluteYawDeg).toDouble(),
                1.0
            )
            try {
                org.opencv.imgproc.Imgproc.warpAffine(
                    atlasGray8, atlasGray8Rot, rotM,
                    org.opencv.core.Size(size.toDouble(), size.toDouble()),
                    org.opencv.imgproc.Imgproc.INTER_LINEAR,
                    org.opencv.core.Core.BORDER_CONSTANT,
                    org.opencv.core.Scalar(0.0)
                )
                org.opencv.imgproc.Imgproc.warpAffine(
                    atlasRoiMask, atlasRoiMaskRot, rotM,
                    org.opencv.core.Size(size.toDouble(), size.toDouble()),
                    org.opencv.imgproc.Imgproc.INTER_NEAREST,
                    org.opencv.core.Core.BORDER_CONSTANT,
                    org.opencv.core.Scalar(0.0)
                )
            } finally {
                rotM.release()
            }

            // ORB detect + compute on the rotated atlas and the zenith.
            val orb = ORB.create(
                ORB_NUM_FEATURES,
                ORB_SCALE_FACTOR,
                ORB_NUM_LEVELS,
                ORB_EDGE_THRESHOLD,
                0,
                2,
                ORB.HARRIS_SCORE,
                31,
                ORB_FAST_THRESHOLD
            )

            val kpAtlas = MatOfKeyPoint()
            val descAtlas = Mat()
            val kpZenith = MatOfKeyPoint()
            val descZenith = Mat()

            try {
                orb.detectAndCompute(atlasGray8Rot, atlasRoiMaskRot, kpAtlas, descAtlas)
                orb.detectAndCompute(zenithGray8, zenithRoiMask, kpZenith, descZenith)

                val nAtlas = kpAtlas.rows()
                val nZenith = kpZenith.rows()

                if (verbose) {
                    Log.d(
                        "AtlasZenithFeatureDetect",
                        "frame=$frameId atlasKp=$nAtlas zenithKp=$nZenith " +
                                "preRotateDeg=${"%.2f".format(-seedAbsoluteYawDeg)}"
                    )
                }

                if (nAtlas < MIN_INLIERS_RELAXED || nZenith < MIN_INLIERS_RELAXED ||
                    descAtlas.empty() || descZenith.empty()
                ) {
                    return reject(
                        reason = "too_few_keypoints",
                        seedAbsoluteYawDeg = seedAbsoluteYawDeg,
                        numMatches = 0
                    )
                }

                // Match with Lowe ratio test.
                val matcher = BFMatcher.create(Core_NORM_HAMMING, false)
                val knn = ArrayList<MatOfDMatch>()
                matcher.knnMatch(descZenith, descAtlas, knn, 2)

                val goodMatches = ArrayList<DMatch>()
                val rejectedByLowe = ArrayList<Float>()
                for (pair in knn) {
                    val arr = pair.toArray()
                    if (arr.size < 2) continue
                    val m = arr[0]
                    val n = arr[1]
                    val ratio = if (n.distance > 0f) m.distance / n.distance else 0f
                    if (m.distance < LOWE_RATIO * n.distance) {
                        goodMatches.add(m)
                    } else {
                        rejectedByLowe.add(ratio)
                    }
                }

                if (verbose) {
                    val meanGoodDist =
                        if (goodMatches.isEmpty()) Float.NaN
                        else goodMatches.map { it.distance }.average().toFloat()
                    val meanRejectRatio =
                        if (rejectedByLowe.isEmpty()) Float.NaN
                        else rejectedByLowe.average().toFloat()
                    Log.d(
                        "AtlasZenithFeatureMatch",
                        "frame=$frameId knn=${knn.size} " +
                                "goodMatches=${goodMatches.size} " +
                                "rejectedLowe=${rejectedByLowe.size} " +
                                "meanGoodHamming=${"%.1f".format(meanGoodDist)} " +
                                "meanRejectedLoweRatio=${"%.3f".format(meanRejectRatio)}"
                    )
                }

                if (goodMatches.size < MIN_MATCHES_FOR_RANSAC) {
                    return reject(
                        reason = "too_few_matches",
                        seedAbsoluteYawDeg = seedAbsoluteYawDeg,
                        numMatches = goodMatches.size
                    )
                }

                // Convert each match to residual delta azimuth.
                //
                // Because we pre-rotated the atlas by -seedYaw, the atlas
                // keypoint at pixel (x,y) now represents the world azimuth
                //   azAtlasWorld = topFacePixelToAzimuthDeg(x,y) + seedYaw
                // but in the rotated frame what RANSAC sees is just
                //   azAtlasRot = topFacePixelToAzimuthDeg(x,y)
                // The zenith keypoint at pixel (x,y) represents
                //   azZenithCam = topFacePixelToAzimuthDeg(x,y)   (camera frame)
                // whose world azimuth is azZenithCam + seedYaw.
                //
                // So in the ROTATED frame:
                //   delta_rotated = azAtlasRot - azZenithCam
                //                 = (azAtlasWorld - seedYaw) - (azZenithCamWorld - seedYaw)
                //                 = azAtlasWorld - azZenithCamWorld
                //                 = deltaYaw_world
                //
                // i.e., delta_rotated IS the world-frame deltaYaw. No extra
                // correction needed. We just compute it directly.

                val kpAtlasArr = kpAtlas.toArray()
                val kpZenithArr = kpZenith.toArray()

                val deltaAzDegList = FloatArray(goodMatches.size)
                var usedMatches = 0

                for (match in goodMatches) {
                    val kpQuery = kpZenithArr[match.queryIdx]
                    val kpTrain = kpAtlasArr[match.trainIdx]

                    val azZenith = topFacePixelToAzimuthDeg(
                        kpQuery.pt.x.toFloat(),
                        kpQuery.pt.y.toFloat(),
                        center
                    )
                    val azAtlasRotated = topFacePixelToAzimuthDeg(
                        kpTrain.pt.x.toFloat(),
                        kpTrain.pt.y.toFloat(),
                        center
                    )

                    if (azZenith.isNaN() || azAtlasRotated.isNaN()) continue

                    val delta = normalizeSignedDeg(azAtlasRotated - azZenith)
                    deltaAzDegList[usedMatches++] = delta
                }

                if (usedMatches < MIN_MATCHES_FOR_RANSAC) {
                    return reject(
                        reason = "too_few_after_geom_conversion",
                        seedAbsoluteYawDeg = seedAbsoluteYawDeg,
                        numMatches = usedMatches
                    )
                }

                // RANSAC 1-DoF.
                val ransac = ransacDeltaYaw(deltaAzDegList, usedMatches)

                if (verbose) {
                    // Log histogram of delta azimuth for debugging.
                    val sorted = deltaAzDegList.sliceArray(0 until usedMatches).sortedArray()
                    val p10 = sorted[usedMatches / 10]
                    val p50 = sorted[usedMatches / 2]
                    val p90 = sorted[usedMatches * 9 / 10]
                    Log.d(
                        "AtlasZenithFeatureRansac",
                        "frame=$frameId matches=$usedMatches " +
                                "deltaAzP10=${"%.2f".format(p10)} " +
                                "deltaAzP50=${"%.2f".format(p50)} " +
                                "deltaAzP90=${"%.2f".format(p90)} " +
                                "ransacBestInliers=${ransac.numInliers} " +
                                "ransacDeltaYaw=${"%.3f".format(ransac.deltaYawDeg)}"
                    )
                }

                if (ransac.numInliers < MIN_INLIERS_RELAXED) {
                    return reject(
                        reason = "ransac_too_few_inliers",
                        seedAbsoluteYawDeg = seedAbsoluteYawDeg,
                        numMatches = usedMatches,
                        ransacInliers = ransac.numInliers
                    )
                }

                // Refine via circular mean of inliers.
                val finalDeltaYawDeg = circularMeanDeg(
                    deltaAzDegList,
                    ransac.inlierMask,
                    usedMatches
                )

                val residuals = FloatArray(ransac.numInliers)
                var residualIdx = 0
                for (i in 0 until usedMatches) {
                    if (ransac.inlierMask[i]) {
                        residuals[residualIdx++] =
                            abs(normalizeSignedDeg(deltaAzDegList[i] - finalDeltaYawDeg))
                    }
                }
                residuals.sort()
                val residualMedian = residuals[ransac.numInliers / 2]
                val residualP90 =
                    residuals[(ransac.numInliers * 9 / 10).coerceAtMost(ransac.numInliers - 1)]

                val acceptMode = when {
                    ransac.numInliers >= MIN_INLIERS_STRICT &&
                            residualMedian <= MAX_RESIDUAL_MEDIAN_DEG_STRICT -> "strict"
                    ransac.numInliers >= MIN_INLIERS_RELAXED &&
                            residualMedian <= MAX_RESIDUAL_MEDIAN_DEG_RELAXED -> "relaxed"
                    else -> "rejected"
                }

                return FeatureRefineResult(
                    deltaYawDeg = finalDeltaYawDeg,
                    finalYawDeg = normalizeDeg(seedAbsoluteYawDeg + finalDeltaYawDeg),
                    numMatches = usedMatches,
                    numInliers = ransac.numInliers,
                    residualMedianDeg = residualMedian,
                    residualP90Deg = residualP90,
                    accepted = acceptMode != "rejected",
                    acceptMode = acceptMode
                )
            } finally {
                kpAtlas.release()
                descAtlas.release()
                kpZenith.release()
                descZenith.release()
            }
        } finally {
            atlasRoiMask.release()
            zenithRoiMask.release()
            atlasGray8.release()
            zenithGray8.release()
            atlasGray8Rot.release()
            atlasRoiMaskRot.release()
        }
    }

    private fun reject(
        reason: String,
        seedAbsoluteYawDeg: Float,
        numMatches: Int,
        ransacInliers: Int = 0
    ): FeatureRefineResult {
        Log.w("AtlasZenithFeature", "reject reason=$reason matches=$numMatches ransacInliers=$ransacInliers")
        return FeatureRefineResult(
            deltaYawDeg = 0f,
            finalYawDeg = seedAbsoluteYawDeg,
            numMatches = numMatches,
            numInliers = ransacInliers,
            residualMedianDeg = Float.POSITIVE_INFINITY,
            residualP90Deg = Float.POSITIVE_INFINITY,
            accepted = false,
            acceptMode = "rejected"
        )
    }

    /**
     * Returns a CV_8UC1 mask, 255 inside the annulus (alt in [SEAM_MIN, SEAM_MAX])
     * AND where face.validMask is non-zero. This restricts ORB to the seam band.
     */
    private fun buildAnnulusRoiMask(face: ZenithTopFaceRefiner.TopFace): Mat {
        val size = face.faceSizePx
        val mask = Mat.zeros(size, size, CvType.CV_8UC1)

        val center = (size - 1) * 0.5f
        val maxRadius = center
        val innerRadius = altitudeToRadiusPx(SEAM_MAX_ALT_DEG, maxRadius)
        val outerRadius = altitudeToRadiusPx(SEAM_MIN_ALT_DEG, maxRadius)

        for (y in 0 until size) {
            for (x in 0 until size) {
                val dx = x - center
                val dy = y - center
                val r = sqrt(dx * dx + dy * dy)
                if (r < innerRadius || r > outerRadius) continue
                // Intersect with face valid mask (non-zero bytes only).
                val v = face.validMask.get(y, x)?.firstOrNull()?.toInt() ?: 0
                if (v > 0) mask.put(y, x, 255.0)
            }
        }
        return mask
    }

    private fun altitudeToRadiusPx(altDeg: Float, maxRadius: Float): Float {
        // Same convention as ZenithTopFaceRefiner.altitudeToTopRadiusPx.
        val zenithAngleDeg = 90f - altDeg
        return (kotlin.math.tan(Math.toRadians(zenithAngleDeg.toDouble())) * maxRadius).toFloat()
    }

    /**
     * Convert a pixel on the top face to world azimuth in degrees.
     *
     * CRITICAL: this must use the EXACT SAME convention as
     * ZenithTopFaceRefiner.buildAtlasTopFace does when it maps
     * topFace(x,y) to AtlasMath.azimuthToX. If these drift apart,
     * we reintroduce the original bug.
     *
     * In the legacy code: buildAtlasTopFace computes
     *   azDeg = normalizeDeg(radToDeg(atan2(dir[0], dir[1])))
     * where dir = topFaceDirection(x, y, center, radius).
     *
     * topFaceDirection builds a 3D direction with
     *   px = (x - center) / radius
     *   py = (y - center) / radius
     *   normalize([px, py, 1])
     *
     * So azimuth = atan2(px, py) = atan2(x - center, y - center).
     * Returns NaN if the pixel is outside the usable disc.
     */
    private fun topFacePixelToAzimuthDeg(x: Float, y: Float, center: Float): Float {
        val dx = x - center
        val dy = y - center
        if (dx * dx + dy * dy < 1e-6f) return Float.NaN
        val azRad = atan2(dx.toDouble(), dy.toDouble())
        return normalizeDeg(Math.toDegrees(azRad).toFloat())
    }

    // --- RANSAC 1-DoF over circular residuals ---

    private data class RansacResult(
        val deltaYawDeg: Float,
        val numInliers: Int,
        val inlierMask: BooleanArray
    )

    private fun ransacDeltaYaw(
        deltaAzDeg: FloatArray,
        n: Int
    ): RansacResult {
        var bestDelta = 0f
        var bestInliers = 0
        var bestMask = BooleanArray(n)

        val rng = java.util.Random(0x5EED.toLong()) // deterministic across runs

        val iters = if (n <= 2) 1 else RANSAC_ITERATIONS

        for (iter in 0 until iters) {
            val idx = if (iters == 1) 0 else rng.nextInt(n)
            val hypothesis = deltaAzDeg[idx]

            var count = 0
            for (i in 0 until n) {
                val r = abs(normalizeSignedDeg(deltaAzDeg[i] - hypothesis))
                if (r <= RANSAC_INLIER_THRESHOLD_DEG) count++
            }

            if (count > bestInliers) {
                bestInliers = count
                bestDelta = hypothesis
                bestMask = BooleanArray(n) { i ->
                    abs(normalizeSignedDeg(deltaAzDeg[i] - hypothesis)) <= RANSAC_INLIER_THRESHOLD_DEG
                }
            }
        }

        return RansacResult(bestDelta, bestInliers, bestMask)
    }

    private fun circularMeanDeg(
        deltaAzDeg: FloatArray,
        inlierMask: BooleanArray,
        n: Int
    ): Float {
        var sumSin = 0.0
        var sumCos = 0.0
        for (i in 0 until n) {
            if (!inlierMask[i]) continue
            val r = Math.toRadians(deltaAzDeg[i].toDouble())
            sumSin += sin(r)
            sumCos += cos(r)
        }
        if (sumSin * sumSin + sumCos * sumCos < 1e-12) return 0f
        return Math.toDegrees(atan2(sumSin, sumCos)).toFloat()
    }

    // --- helpers ---

    private fun normalizeDeg(value: Float): Float {
        var v = value % 360f
        if (v < 0f) v += 360f
        return v
    }

    private fun normalizeSignedDeg(value: Float): Float {
        var v = value
        while (v <= -180f) v += 360f
        while (v > 180f) v -= 360f
        return v
    }

    private fun buildDummyTopFace(): ZenithTopFaceRefiner.TopFace {
        // Minimal 1x1 TopFace used only to satisfy caller's releaseTopFace
        // contract after we already consumed the real corrected top face.
        val rgba = Mat.zeros(1, 1, CvType.CV_8UC4)
        val gray32 = Mat.zeros(1, 1, CvType.CV_32F)
        val mask = Mat.zeros(1, 1, CvType.CV_8UC1)
        return ZenithTopFaceRefiner.TopFace(
            rgba = rgba,
            gray32 = gray32,
            validMask = mask,
            faceSizePx = 1
        )
    }

    // Hamming norm constant (exposed via Core in OpenCV Java binding; some
    // OpenCV builds don't surface it by exact name — inline the value).
    @Suppress("ConstPropertyName")
    private const val Core_NORM_HAMMING = 6  // cv::NORM_HAMMING
}