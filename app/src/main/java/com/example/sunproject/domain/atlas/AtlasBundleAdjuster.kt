package com.example.sunproject.domain.atlas

import android.graphics.BitmapFactory
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.FrameRecord
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector
import org.apache.commons.math3.util.Pair as Math3Pair
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.DMatch
import org.opencv.core.KeyPoint
import org.opencv.core.Mat
import org.opencv.core.MatOfDMatch
import org.opencv.core.MatOfKeyPoint
import org.opencv.core.Size
import org.opencv.features2d.BFMatcher
import org.opencv.features2d.ORB
import org.opencv.imgproc.Imgproc
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Refinamiento global de poses H0/H45 via Bundle Adjustment con seed IMU.
 *
 * ============================================================
 * DISEÑO (refactorizado: BA basado en ángulos)
 * ============================================================
 * El AtlasProjector consume ÁNGULOS por frame, no matrices:
 *   - yaw   = measuredAzimuthDeg                         (game rotation, gyro-only)
 *   - pitch = absPitchDeg ?: measuredPitchDeg            (pitch absoluto)
 *   - roll  = absRollDeg  ?: measuredRollDeg             (roll absoluto)
 *
 * Por eso el BA opera y escribe ÁNGULOS, no matrices. Las matrices
 * rotationM** del FrameRecord (que vienen del rotation_vector con
 * magnetómetro) NO se modifican aquí; quedan intactas como datos
 * crudos para Z0/ZenithMatrixProjector y para análisis.
 *
 * El BA antes-de-este-refactor escribía matrices rotationM** que
 * el projector ignoraba para H0/H45 — los refinamientos no llegaban
 * al atlas. Esta versión arregla esa desconexión.
 *
 * ============================================================
 * GEOMETRÍA (idéntica al AtlasProjector.buildProjectionBasisFromAngles)
 * ============================================================
 * Para cada frame con ángulos (yawDeg, pitchDeg, rollDeg):
 *   forward = (cos(p)·sin(y), cos(p)·cos(y), sin(p))     (ENU)
 *   right0  = normalize(cross(forward, [0,0,1]))         (horizontal)
 *   up0     = normalize(cross(right0, forward))
 *   right   = right0·cos(roll) + up0·sin(roll)
 *   up      = up0·cos(roll) - right0·sin(roll)
 *
 * Pixel del frame → rayo world:
 *   nx = (px/W)·2 - 1
 *   ny = (py/H)·2 - 1
 *   camX = nx · tan(hfov/2)
 *   camY = -ny · tan(vfov/2)        (signo negativo, igual al projector)
 *   ray  = normalize(camX·right + camY·up + forward)
 *
 * ============================================================
 * REGULARIZACIÓN
 * ============================================================
 * El LM minimiza la suma de:
 *   - 3 residuales por match (diferencia de rayos world entre frames vecinos)
 *   - 3 residuales de regularización por frame (delta del seed en yaw/pitch/roll)
 *
 * Lambda gobierna fortaleza del ancla al seed. Garantiza que:
 *   - el atlas no derive en su conjunto (gauge global fijado por seed)
 *   - frames con pocos matches se queden cerca del seed
 *   - peor caso = baseline IMU (BA nunca empeora si está bien regularizado)
 */
object AtlasBundleAdjuster {

    // ============================================================
    // CONSTANTES
    // ============================================================

    private const val MAX_PAIR_ANGLE_DEG = 60f
    private const val DEFAULT_LAMBDA = 5.0f
    private const val MAX_FEATURE_DIM = 1000

    private const val ORB_NUM_FEATURES = 2000
    private const val ORB_SCALE_FACTOR = 1.2f
    private const val ORB_NUM_LEVELS = 8
    private const val ORB_EDGE_THRESHOLD = 10
    private const val ORB_FAST_THRESHOLD = 10
    private const val LOWE_RATIO = 0.75f

    private const val MIN_MATCHES_PER_PAIR = 8
    private const val RANSAC_INLIER_THRESHOLD_DEG = 2.5f
    private const val MAX_PAIR_MEDIAN_RESIDUAL_DEG = 1.5f

    private const val LM_MAX_ITERATIONS = 30
    private const val LM_RELATIVE_TOLERANCE = 1e-6

    private const val MAX_GLOBAL_DRIFT_DEG = 0.5f
    private const val MAX_PER_FRAME_DRIFT_DEG = 2.0f
    private const val MIN_FRAME_COVERAGE_RATIO = 0.5f

    // ============================================================
    // DATA CLASSES
    // ============================================================

    data class PairStat(
        val frameI: String,
        val frameJ: String,
        val matchesRaw: Int,
        val matchesFiltered: Int,
        val medianResidualDeg: Float
    )

    data class Result(
        val refinedFrames: List<FrameRecord>,
        val accepted: Boolean,
        val reason: String,
        val iterations: Int,
        val initialCost: Double,
        val finalCost: Double,
        val globalDriftDeg: Float,
        val medianPerFrameDriftDeg: Float,
        val maxPerFrameDriftDeg: Float,
        val pairStats: List<PairStat>
    )

    private data class FrameAngles(val yawDeg: Float, val pitchDeg: Float, val rollDeg: Float)

    private data class FrameMeta(val hfovDeg: Float, val vfovDeg: Float, val imgW: Int, val imgH: Int)

    private data class FrameFeatures(
        val keypoints: List<KeyPoint>,
        val descriptors: Mat,
        val scale: Float,
        val origW: Int,
        val origH: Int
    )

    private data class PairMatch(
        val pxI: Float, val pyI: Float,
        val pxJ: Float, val pyJ: Float
    )

    private data class PairData(
        val i: Int,
        val j: Int,
        val matches: List<PairMatch>,
        val matchesRaw: Int,
        val medianResidualDeg: Float
    )

    private data class DriftStats(
        val globalDeg: Float,
        val medianPerFrameDeg: Float,
        val maxPerFrameDeg: Float,
        val perFrameDeg: List<Float>
    )

    // ============================================================
    // API PÚBLICA
    // ============================================================

    fun refineOrFallback(
        frames: List<FrameRecord>,
        lambda: Float = DEFAULT_LAMBDA
    ): List<FrameRecord> {
        return try {
            val result = refine(frames, lambda)
            if (result.accepted) {
                Log.i(
                    TAG,
                    "result accepted=true iterations=${result.iterations} " +
                            "initialCost=${"%.6f".format(result.initialCost)} " +
                            "finalCost=${"%.6f".format(result.finalCost)} " +
                            "globalDriftDeg=${"%.3f".format(result.globalDriftDeg)} " +
                            "medianPerFrameDriftDeg=${"%.3f".format(result.medianPerFrameDriftDeg)}"
                )
                result.refinedFrames
            } else {
                Log.w(TAG, "result accepted=false reason=${result.reason} — fallback to seed IMU")
                frames
            }
        } catch (t: Throwable) {
            Log.e(TAG, "BA crashed, falling back to seed IMU", t)
            frames
        }
    }

    fun refine(frames: List<FrameRecord>, lambda: Float = DEFAULT_LAMBDA): Result {
        SunProjectApp.requireOpenCv()

        // 1. Filtrar frames válidos
        val validFrames = frames.filter { f ->
            !isZenithFrame(f) && f.hfovDeg != null && f.vfovDeg != null
        }
        if (validFrames.size < 4) {
            return failedResult(frames, "too_few_valid_frames=${validFrames.size}")
        }

        Log.i(TAG, "starting BA on ${validFrames.size} non-zenith frames, lambda=$lambda (angle-based)")

        // 2. Leer ángulos seed (los que el projector consume)
        val seedAngles = validFrames.map { extractAngles(it) }
        val frameMetas = validFrames.map { f ->
            FrameMeta(
                hfovDeg = f.hfovDeg!!,
                vfovDeg = f.vfovDeg!!,
                imgW = f.imageWidthPx ?: 1,
                imgH = f.imageHeightPx ?: 1
            )
        }

        // 3. Vecindades por forward direction
        val pairs = computeNeighborPairs(seedAngles)
        Log.i(TAG, "neighbors: ${pairs.size} pairs from ${validFrames.size} frames")

        // 4. Extraer matches por par
        val pairData = extractAllPairMatches(validFrames, frameMetas, seedAngles, pairs)
        val pairStats = pairData.map { pd ->
            PairStat(
                frameI = validFrames[pd.i].frameId,
                frameJ = validFrames[pd.j].frameId,
                matchesRaw = pd.matchesRaw,
                matchesFiltered = pd.matches.size,
                medianResidualDeg = pd.medianResidualDeg
            )
        }
        pairStats.forEach { ps ->
            Log.d(
                TAG,
                "pair ${ps.frameI}↔${ps.frameJ} matchesRaw=${ps.matchesRaw} " +
                        "matchesFilt=${ps.matchesFiltered} medResDeg=${"%.2f".format(ps.medianResidualDeg)}"
            )
        }

        // 5. Filtrar pares por count y medRes
        val usablePairs = pairData.filter { pd ->
            pd.matches.size >= MIN_MATCHES_PER_PAIR &&
                    pd.medianResidualDeg <= MAX_PAIR_MEDIAN_RESIDUAL_DEG
        }
        if (usablePairs.isEmpty()) {
            return failedResult(frames, "no_usable_pairs_after_filter", pairStats)
        }
        val rejectedByMedRes = pairData.count { pd ->
            pd.matches.size >= MIN_MATCHES_PER_PAIR &&
                    pd.medianResidualDeg > MAX_PAIR_MEDIAN_RESIDUAL_DEG
        }
        Log.i(
            TAG,
            "pair filter: usable=${usablePairs.size} rejected_by_medres=$rejectedByMedRes " +
                    "rejected_by_count=${pairData.size - usablePairs.size - rejectedByMedRes}"
        )

        // 6. Cobertura mínima
        val framesWithConstraints = mutableSetOf<Int>()
        usablePairs.forEach { pd ->
            framesWithConstraints.add(pd.i)
            framesWithConstraints.add(pd.j)
        }
        val coverage = framesWithConstraints.size.toFloat() / validFrames.size
        if (coverage < MIN_FRAME_COVERAGE_RATIO) {
            return failedResult(
                frames,
                "low_frame_coverage=${"%.2f".format(coverage)} " +
                        "framesWithConstraints=${framesWithConstraints.size}/${validFrames.size}",
                pairStats
            )
        }

        // 7. Initial point: 60 parámetros = 20 frames × (yaw, pitch, roll) en grados
        val nFrames = validFrames.size
        val initialPoint = DoubleArray(nFrames * 3)
        for (i in 0 until nFrames) {
            initialPoint[i * 3] = seedAngles[i].yawDeg.toDouble()
            initialPoint[i * 3 + 1] = seedAngles[i].pitchDeg.toDouble()
            initialPoint[i * 3 + 2] = seedAngles[i].rollDeg.toDouble()
        }

        // 8. LM
        val problem = buildLeastSquaresProblem(
            usablePairs = usablePairs,
            frameMetas = frameMetas,
            seedAngles = seedAngles,
            initialPoint = initialPoint,
            lambda = lambda.toDouble()
        )

        val initialCost = computeProblemCost(problem, ArrayRealVector(initialPoint))
        val optimizer = LevenbergMarquardtOptimizer()

        val optimum = try {
            optimizer.optimize(problem)
        } catch (t: Throwable) {
            Log.e(TAG, "LM optimizer threw", t)
            return failedResult(frames, "lm_threw=${t.message}", pairStats)
        }

        val finalCost = optimum.cost * optimum.cost
        val iterations = optimum.iterations

        Log.i(
            TAG,
            "lm done iterations=$iterations initialCost=${"%.6f".format(initialCost)} " +
                    "finalCost=${"%.6f".format(finalCost)} relGain=${"%.4f".format(1.0 - finalCost / initialCost)}"
        )

        if (finalCost > initialCost * 1.01) {
            return failedResult(
                frames,
                "cost_increased initial=${"%.6f".format(initialCost)} final=${"%.6f".format(finalCost)}",
                pairStats
            )
        }

        // 9. Extraer ángulos refinados
        val solution = optimum.point.toArray()
        val refinedAngles = (0 until nFrames).map { idx ->
            FrameAngles(
                yawDeg = normalizeYawDeg(solution[idx * 3].toFloat()),
                pitchDeg = solution[idx * 3 + 1].toFloat(),
                rollDeg = solution[idx * 3 + 2].toFloat()
            )
        }

        // 10. Drift checks
        val driftStats = computeDriftStats(seedAngles, refinedAngles)
        Log.i(
            TAG,
            "drift globalDeg=${"%.3f".format(driftStats.globalDeg)} " +
                    "medianFrameDeg=${"%.3f".format(driftStats.medianPerFrameDeg)} " +
                    "maxFrameDeg=${"%.3f".format(driftStats.maxPerFrameDeg)}"
        )
        validFrames.forEachIndexed { idx, f ->
            val before = seedAngles[idx]
            val after = refinedAngles[idx]
            Log.d(
                TAG,
                "frame_drift ${f.frameId} angleDeg=${"%.3f".format(driftStats.perFrameDeg[idx])} " +
                        "yaw ${"%.2f".format(before.yawDeg)}→${"%.2f".format(after.yawDeg)} " +
                        "pitch ${"%.2f".format(before.pitchDeg)}→${"%.2f".format(after.pitchDeg)} " +
                        "roll ${"%.2f".format(before.rollDeg)}→${"%.2f".format(after.rollDeg)}"
            )
        }

        if (driftStats.globalDeg > MAX_GLOBAL_DRIFT_DEG) {
            return failedResult(
                frames,
                "global_drift_exceeded=${"%.3f".format(driftStats.globalDeg)} max=$MAX_GLOBAL_DRIFT_DEG",
                pairStats
            )
        }
        if (driftStats.maxPerFrameDeg > MAX_PER_FRAME_DRIFT_DEG) {
            return failedResult(
                frames,
                "per_frame_drift_exceeded max=${"%.3f".format(driftStats.maxPerFrameDeg)} threshold=$MAX_PER_FRAME_DRIFT_DEG",
                pairStats
            )
        }

        // 11. Escribir ángulos refinados al FrameRecord. NO tocar rotationM**.
        val refinedById = validFrames.zip(refinedAngles).associate { (frame, ang) ->
            frame.frameId to ang
        }
        val outFrames = frames.map { f ->
            val ang = refinedById[f.frameId] ?: return@map f
            f.copy(
                measuredAzimuthDeg = ang.yawDeg,
                absPitchDeg = ang.pitchDeg,
                absRollDeg = ang.rollDeg
            )
        }

        return Result(
            refinedFrames = outFrames,
            accepted = true,
            reason = "ok",
            iterations = iterations,
            initialCost = initialCost,
            finalCost = finalCost,
            globalDriftDeg = driftStats.globalDeg,
            medianPerFrameDriftDeg = driftStats.medianPerFrameDeg,
            maxPerFrameDriftDeg = driftStats.maxPerFrameDeg,
            pairStats = pairStats
        )
    }

    // ============================================================
    // GEOMETRÍA (réplica exacta de AtlasProjector.buildProjectionBasisFromAngles)
    // ============================================================

    private fun extractAngles(f: FrameRecord): FrameAngles {
        val yaw = f.measuredAzimuthDeg
        val pitch = f.absPitchDeg ?: f.measuredPitchDeg
        val roll = f.absRollDeg ?: f.measuredRollDeg
        return FrameAngles(yaw, pitch, roll)
    }

    private fun worldDirection(yawRad: Double, pitchRad: Double): DoubleArray {
        val cp = cos(pitchRad)
        return doubleArrayOf(
            cp * sin(yawRad),
            cp * cos(yawRad),
            sin(pitchRad)
        )
    }

    /**
     * Mismo algoritmo y orden de operaciones que
     * AtlasProjector.buildProjectionBasisFromAngles para la rama
     * zenithLike=false. Lo mantenemos aquí para no exponer la función
     * privada del projector.
     */
    private fun buildBasis(
        yawDeg: Double, pitchDeg: Double, rollDeg: Double
    ): Triple<DoubleArray, DoubleArray, DoubleArray> {
        val yawRad = Math.toRadians(yawDeg)
        val pitchRad = Math.toRadians(pitchDeg)
        val rollRad = Math.toRadians(rollDeg)

        val forward = worldDirection(yawRad, pitchRad)

        // right0 = cross(forward, [0,0,1]) = (forward[1], -forward[0], 0)
        var rx = forward[1]
        var ry = -forward[0]
        var rz = 0.0
        var rNorm = sqrt(rx * rx + ry * ry + rz * rz)
        if (rNorm < 1e-4) {
            rx = 1.0; ry = 0.0; rz = 0.0; rNorm = 1.0
        }
        val right0 = doubleArrayOf(rx / rNorm, ry / rNorm, rz / rNorm)

        // up0 = cross(right0, forward), normalizado
        val up0x = right0[1] * forward[2] - right0[2] * forward[1]
        val up0y = right0[2] * forward[0] - right0[0] * forward[2]
        val up0z = right0[0] * forward[1] - right0[1] * forward[0]
        val u0Norm = sqrt(up0x * up0x + up0y * up0y + up0z * up0z).coerceAtLeast(1e-12)
        val up0 = doubleArrayOf(up0x / u0Norm, up0y / u0Norm, up0z / u0Norm)

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

    private fun pixelToWorldRay(
        px: Float, py: Float,
        meta: FrameMeta,
        yawDeg: Double, pitchDeg: Double, rollDeg: Double
    ): DoubleArray {
        val (forward, right, up) = buildBasis(yawDeg, pitchDeg, rollDeg)
        val tanHalfH = tan(Math.toRadians(meta.hfovDeg.toDouble()) / 2.0)
        val tanHalfV = tan(Math.toRadians(meta.vfovDeg.toDouble()) / 2.0)
        val nx = (px / meta.imgW.toDouble()) * 2.0 - 1.0
        val ny = (py / meta.imgH.toDouble()) * 2.0 - 1.0
        val camX = nx * tanHalfH
        val camY = -ny * tanHalfV
        val rx = camX * right[0] + camY * up[0] + forward[0]
        val ry = camX * right[1] + camY * up[1] + forward[1]
        val rz = camX * right[2] + camY * up[2] + forward[2]
        val n = sqrt(rx * rx + ry * ry + rz * rz).coerceAtLeast(1e-12)
        return doubleArrayOf(rx / n, ry / n, rz / n)
    }

    // ============================================================
    // VECINDADES
    // ============================================================

    private fun computeNeighborPairs(seedAngles: List<FrameAngles>): List<Pair<Int, Int>> {
        val n = seedAngles.size
        val forwards = seedAngles.map {
            worldDirection(
                Math.toRadians(it.yawDeg.toDouble()),
                Math.toRadians(it.pitchDeg.toDouble())
            )
        }
        val cosThr = cos(Math.toRadians(MAX_PAIR_ANGLE_DEG.toDouble()))
        val out = mutableListOf<Pair<Int, Int>>()
        for (i in 0 until n) {
            for (j in (i + 1) until n) {
                val d = forwards[i][0] * forwards[j][0] +
                        forwards[i][1] * forwards[j][1] +
                        forwards[i][2] * forwards[j][2]
                if (d >= cosThr) out.add(i to j)
            }
        }
        return out
    }

    // ============================================================
    // FEATURE EXTRACTION & MATCHING
    // ============================================================

    private fun extractAllPairMatches(
        frames: List<FrameRecord>,
        frameMetas: List<FrameMeta>,
        seedAngles: List<FrameAngles>,
        pairs: List<Pair<Int, Int>>
    ): List<PairData> {
        val features = frames.indices.map { idx ->
            detectFeatures(frames[idx])
        }
        val out = mutableListOf<PairData>()
        for ((i, j) in pairs) {
            val pd = matchPair(
                featI = features[i], metaI = frameMetas[i], anglesI = seedAngles[i], indexI = i,
                featJ = features[j], metaJ = frameMetas[j], anglesJ = seedAngles[j], indexJ = j
            )
            out.add(pd)
        }
        features.forEach { it?.descriptors?.release() }
        return out
    }

    private fun detectFeatures(frame: FrameRecord): FrameFeatures? {
        val srcBitmap = BitmapFactory.decodeFile(frame.originalPath) ?: return null
        try {
            val origW = srcBitmap.width
            val origH = srcBitmap.height
            val maxDim = max(origW, origH)
            val scale = if (maxDim > MAX_FEATURE_DIM) MAX_FEATURE_DIM.toFloat() / maxDim else 1f
            val w = (origW * scale).toInt()
            val h = (origH * scale).toInt()

            val rgba = Mat()
            Utils.bitmapToMat(srcBitmap, rgba)
            val gray = Mat()
            Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_RGBA2GRAY)
            rgba.release()

            val grayDown = if (scale < 1f) {
                val tmp = Mat()
                Imgproc.resize(gray, tmp, Size(w.toDouble(), h.toDouble()), 0.0, 0.0, Imgproc.INTER_AREA)
                gray.release()
                tmp
            } else gray

            val orb = ORB.create(
                ORB_NUM_FEATURES, ORB_SCALE_FACTOR, ORB_NUM_LEVELS,
                ORB_EDGE_THRESHOLD, 0, 2,
                ORB.HARRIS_SCORE, 31, ORB_FAST_THRESHOLD
            )
            val kpMat = MatOfKeyPoint()
            val descriptors = Mat()
            orb.detectAndCompute(grayDown, Mat(), kpMat, descriptors)

            val kp = kpMat.toList()
            kpMat.release()
            grayDown.release()

            return FrameFeatures(kp, descriptors, scale, origW, origH)
        } finally {
            srcBitmap.recycle()
        }
    }

    private fun matchPair(
        featI: FrameFeatures?, metaI: FrameMeta, anglesI: FrameAngles, indexI: Int,
        featJ: FrameFeatures?, metaJ: FrameMeta, anglesJ: FrameAngles, indexJ: Int
    ): PairData {
        if (featI == null || featJ == null ||
            featI.keypoints.size < MIN_MATCHES_PER_PAIR ||
            featJ.keypoints.size < MIN_MATCHES_PER_PAIR
        ) {
            return PairData(indexI, indexJ, emptyList(), 0, 0f)
        }

        val matcher = BFMatcher.create(org.opencv.core.Core.NORM_HAMMING, false)
        val knn = mutableListOf<MatOfDMatch>()
        matcher.knnMatch(featI.descriptors, featJ.descriptors, knn, 2)

        val rawMatches = mutableListOf<DMatch>()
        for (knnPair in knn) {
            val arr = knnPair.toArray()
            if (arr.size >= 2 && arr[0].distance < LOWE_RATIO * arr[1].distance) {
                rawMatches.add(arr[0])
            }
            knnPair.release()
        }

        val matchesRaw = rawMatches.size
        if (matchesRaw < MIN_MATCHES_PER_PAIR) {
            return PairData(indexI, indexJ, emptyList(), matchesRaw, 0f)
        }

        // Filtrar contra seed: para cada match, computar rayos en world con
        // los ángulos seed de cada frame. Si el ángulo entre rayos > threshold,
        // descartar (mismatch geométrico).
        val cosThr = cos(Math.toRadians(RANSAC_INLIER_THRESHOLD_DEG.toDouble()))
        val inliers = mutableListOf<PairMatch>()
        val residualsDeg = mutableListOf<Float>()

        for (m in rawMatches) {
            val kpI = featI.keypoints[m.queryIdx]
            val kpJ = featJ.keypoints[m.trainIdx]

            val pxI = (kpI.pt.x / featI.scale).toFloat()
            val pyI = (kpI.pt.y / featI.scale).toFloat()
            val pxJ = (kpJ.pt.x / featJ.scale).toFloat()
            val pyJ = (kpJ.pt.y / featJ.scale).toFloat()

            val rayI = pixelToWorldRay(
                pxI, pyI, metaI,
                anglesI.yawDeg.toDouble(), anglesI.pitchDeg.toDouble(), anglesI.rollDeg.toDouble()
            )
            val rayJ = pixelToWorldRay(
                pxJ, pyJ, metaJ,
                anglesJ.yawDeg.toDouble(), anglesJ.pitchDeg.toDouble(), anglesJ.rollDeg.toDouble()
            )

            val dot = rayI[0] * rayJ[0] + rayI[1] * rayJ[1] + rayI[2] * rayJ[2]
            if (dot >= cosThr) {
                inliers.add(PairMatch(pxI, pyI, pxJ, pyJ))
                residualsDeg.add(Math.toDegrees(acos(dot.coerceIn(-1.0, 1.0))).toFloat())
            }
        }

        val medRes = if (residualsDeg.isNotEmpty()) {
            residualsDeg.sorted()[residualsDeg.size / 2]
        } else 0f

        return PairData(indexI, indexJ, inliers, matchesRaw, medRes)
    }

    // ============================================================
    // LEAST SQUARES PROBLEM
    // ============================================================

    private fun buildLeastSquaresProblem(
        usablePairs: List<PairData>,
        frameMetas: List<FrameMeta>,
        seedAngles: List<FrameAngles>,
        initialPoint: DoubleArray,
        lambda: Double
    ): LeastSquaresProblem {
        val nFrames = seedAngles.size
        val totalMatches = usablePairs.sumOf { it.matches.size }
        val nResiduals = totalMatches * 3 + nFrames * 3
        val target = ArrayRealVector(DoubleArray(nResiduals))
        val sqrtLambda = sqrt(lambda)

        val model = MultivariateJacobianFunction { params ->
            val p = params.toArray()
            val residual = evaluateResiduals(p, usablePairs, frameMetas, seedAngles, sqrtLambda, nFrames, nResiduals)

            val jacobian = Array2DRowRealMatrix(nResiduals, p.size)
            val eps = 1e-5
            for (k in 0 until p.size) {
                val saved = p[k]
                p[k] = saved + eps
                val resPlus = evaluateResiduals(p, usablePairs, frameMetas, seedAngles, sqrtLambda, nFrames, nResiduals)
                p[k] = saved
                for (r in 0 until nResiduals) {
                    jacobian.setEntry(r, k, (resPlus[r] - residual[r]) / eps)
                }
            }

            Math3Pair(ArrayRealVector(residual) as RealVector, jacobian as RealMatrix)
        }

        return LeastSquaresBuilder()
            .start(initialPoint)
            .target(target)
            .model(model)
            .maxEvaluations(LM_MAX_ITERATIONS * 100)
            .maxIterations(LM_MAX_ITERATIONS)
            .checker { _, prev, curr ->
                val rel = kotlin.math.abs(curr.cost - prev.cost) / max(1e-12, prev.cost)
                rel < LM_RELATIVE_TOLERANCE
            }
            .build()
    }

    private fun evaluateResiduals(
        p: DoubleArray,
        usablePairs: List<PairData>,
        frameMetas: List<FrameMeta>,
        seedAngles: List<FrameAngles>,
        sqrtLambda: Double,
        nFrames: Int,
        nResiduals: Int
    ): DoubleArray {
        val residual = DoubleArray(nResiduals)
        var rIdx = 0

        // Match residuals (3 por match: diferencia de rayos world unitarios)
        for (pair in usablePairs) {
            val yawI = p[pair.i * 3]
            val pitchI = p[pair.i * 3 + 1]
            val rollI = p[pair.i * 3 + 2]
            val yawJ = p[pair.j * 3]
            val pitchJ = p[pair.j * 3 + 1]
            val rollJ = p[pair.j * 3 + 2]
            val mI = frameMetas[pair.i]
            val mJ = frameMetas[pair.j]

            for (m in pair.matches) {
                val rI = pixelToWorldRay(m.pxI, m.pyI, mI, yawI, pitchI, rollI)
                val rJ = pixelToWorldRay(m.pxJ, m.pyJ, mJ, yawJ, pitchJ, rollJ)
                residual[rIdx++] = rI[0] - rJ[0]
                residual[rIdx++] = rI[1] - rJ[1]
                residual[rIdx++] = rI[2] - rJ[2]
            }
        }

        // Regularization residuals (3 por frame, en radianes para que la unidad
        // sea consistente con la del residual de match — chord ≈ angle para
        // ángulos chicos)
        for (frameIdx in 0 until nFrames) {
            val seed = seedAngles[frameIdx]
            val dYawDeg = shortestAngleDeltaDeg(seed.yawDeg.toDouble(), p[frameIdx * 3])
            val dPitchDeg = p[frameIdx * 3 + 1] - seed.pitchDeg.toDouble()
            val dRollDeg = p[frameIdx * 3 + 2] - seed.rollDeg.toDouble()
            residual[rIdx++] = sqrtLambda * Math.toRadians(dYawDeg)
            residual[rIdx++] = sqrtLambda * Math.toRadians(dPitchDeg)
            residual[rIdx++] = sqrtLambda * Math.toRadians(dRollDeg)
        }

        return residual
    }

    private fun computeProblemCost(problem: LeastSquaresProblem, params: RealVector): Double {
        val eval = problem.evaluate(params)
        val r = eval.residuals
        var sum = 0.0
        for (i in 0 until r.dimension) {
            val v = r.getEntry(i)
            sum += v * v
        }
        return sum
    }

    // ============================================================
    // DRIFT METRICS
    // ============================================================

    private fun computeDriftStats(
        seed: List<FrameAngles>,
        refined: List<FrameAngles>
    ): DriftStats {
        require(seed.size == refined.size)
        val perFrame = mutableListOf<Float>()
        var sumYaw = 0.0
        var sumPitch = 0.0
        var sumRoll = 0.0
        for (i in seed.indices) {
            val dy = shortestAngleDeltaDeg(seed[i].yawDeg.toDouble(), refined[i].yawDeg.toDouble()).toFloat()
            val dp = refined[i].pitchDeg - seed[i].pitchDeg
            val dr = refined[i].rollDeg - seed[i].rollDeg
            perFrame.add(sqrt(dy * dy + dp * dp + dr * dr))
            sumYaw += dy
            sumPitch += dp
            sumRoll += dr
        }
        val n = seed.size.toDouble()
        val meanY = sumYaw / n
        val meanP = sumPitch / n
        val meanR = sumRoll / n
        val globalDeg = sqrt(meanY * meanY + meanP * meanP + meanR * meanR).toFloat()
        val sortedPerFrame = perFrame.sorted()
        return DriftStats(
            globalDeg = globalDeg,
            medianPerFrameDeg = sortedPerFrame[sortedPerFrame.size / 2],
            maxPerFrameDeg = sortedPerFrame.last(),
            perFrameDeg = perFrame
        )
    }

    // ============================================================
    // HELPERS
    // ============================================================

    private fun isZenithFrame(f: FrameRecord): Boolean {
        return f.targetPitchDeg >= 80f || f.ringId.contains("Z", ignoreCase = true)
    }

    private fun normalizeYawDeg(deg: Float): Float {
        var v = deg % 360f
        if (v < 0f) v += 360f
        return v
    }

    private fun shortestAngleDeltaDeg(a: Double, b: Double): Double {
        var d = (b - a) % 360.0
        if (d > 180.0) d -= 360.0
        if (d < -180.0) d += 360.0
        return d
    }

    private fun failedResult(
        frames: List<FrameRecord>,
        reason: String,
        pairStats: List<PairStat> = emptyList()
    ): Result {
        return Result(
            refinedFrames = frames,
            accepted = false,
            reason = reason,
            iterations = 0,
            initialCost = 0.0,
            finalCost = 0.0,
            globalDriftDeg = 0f,
            medianPerFrameDriftDeg = 0f,
            maxPerFrameDriftDeg = 0f,
            pairStats = pairStats
        )
    }

    private const val TAG = "AtlasBundleAdjuster"
}