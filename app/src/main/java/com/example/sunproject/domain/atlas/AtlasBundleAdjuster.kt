package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Log
import com.example.sunproject.SunProjectApp
import com.example.sunproject.data.model.FrameRecord
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector
import org.apache.commons.math3.util.Pair as Math3Pair
import org.opencv.android.Utils
import org.opencv.calib3d.Calib3d
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
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Refinamiento global de poses H0/H45 via Bundle Adjustment con seed IMU.
 *
 * ============================================================
 * RATIONAL
 * ============================================================
 * El seed IMU (rotation_vector de Android) tiene jitter típico de 1-2°
 * por frame, lo cual produce ghost geométrico visible en el atlas:
 * cables/objetos duplicados con offset de 10-20 px.
 *
 * Este módulo refina las 20 matrices de rotación (12 H0 + 8 H45,
 * Z0 NO se incluye) resolviendo un único problema de minimización
 * que combina:
 *   (a) error de reproyección de matches ORB entre frames vecinos
 *   (b) regularización al seed IMU (penaliza alejarse del seed)
 *
 * El término (b) tiene tres funciones críticas:
 *   - Fija el gauge global: una rotación común a todos los frames no
 *     reduce (a) pero aumenta (b), entonces el solver no deriva el
 *     atlas en su conjunto. La exactitud absoluta respecto del norte
 *     verdadero se preserva.
 *   - Protege frames con pocas matches: si un par H45-H45 sobre cielo
 *     plano no tiene features, ese frame se queda en el seed.
 *   - Garantiza monotonía: en peor caso, BA devuelve algo equivalente
 *     al baseline IMU. Nunca empeora.
 *
 * ============================================================
 * PIPELINE
 * ============================================================
 * 1. Filtrar frames: descartar Z0, exigir matrices presentes.
 * 2. Determinar vecindades: pares (i,j) con ángulo entre forwards <60°.
 * 3. Para cada par, extraer matches con ORB+BFMatcher+Lowe ratio.
 *    Filtrar por consistencia geométrica con seed IMU.
 * 4. Parametrizar las N rotaciones con Rodrigues (3 floats c/u).
 * 5. Resolver LM con función de costo combinada matches + regularización.
 * 6. Validar: costo no aumenta, drift global <0.5°, %frames cubiertos OK.
 * 7. Devolver FrameRecord con rotationM** actualizadas (o seed si falla).
 *
 * Logs clave (filtrar por tag "AtlasBundleAdjuster"):
 *   - "pair frame_i frame_j matchesRaw=X matchesFilt=Y medResDeg=Z"
 *   - "lm iter=N cost=X resCount=Y paramCount=Z"
 *   - "drift globalDeg=X medianFrameDeg=Y maxFrameDeg=Z"
 *   - "result accepted=true|false reason=..."
 */
object AtlasBundleAdjuster {

    // ============================================================
    // CONSTANTES DE CONFIGURACIÓN
    // ============================================================

    /** Ángulo máximo entre forwards de un par para considerarlo overlap. */
    private const val MAX_PAIR_ANGLE_DEG = 60f

    /**
     * Lambda de regularización al seed IMU (radianes²).
     *
     * Subido de 0.5 a 5.0 tras observar drift per-frame excesivo
     * (mediana 2.3°, max 9.4°) en sesión interior. Con lambda=0.5 los
     * miles de residuales de match aplastaban los 60 residuales de
     * regularización por factor ~800x, permitiendo que el LM moviera
     * frames muy lejos del seed para encajar matches falsos de patrones
     * repetitivos del entorno (techo, cortinas, panels A/C).
     *
     * Con lambda=5.0 una desviación de 1° del seed cuesta ~0.05 unidades
     * (10% del costo típico de matches en óptimo), suficiente para
     * mantener el seed como ancla pero permitir refinamiento de <1°.
     */
    private const val DEFAULT_LAMBDA = 5.0f

    /** Resolución máxima para detección de features (downsample). */
    private const val MAX_FEATURE_DIM = 1000

    /** Configuración ORB. */
    private const val ORB_NUM_FEATURES = 2000
    private const val ORB_SCALE_FACTOR = 1.2f
    private const val ORB_NUM_LEVELS = 8
    private const val ORB_EDGE_THRESHOLD = 10
    private const val ORB_FAST_THRESHOLD = 10

    /** Lowe ratio test. */
    private const val LOWE_RATIO = 0.75f

    /** Mínimo matches por par para considerar la constraint. */
    private const val MIN_MATCHES_PER_PAIR = 8

    /**
     * Threshold del residual angular para descartar matches inconsistentes
     * con el seed IMU.
     *
     * Bajado de 5° a 2.5° tras observar pares con cientos de matches
     * pasando con medResDeg ~4° (típico de patrones repetitivos del
     * entorno interior). El seed IMU tiene jitter ~1-2°, así que matches
     * genuinos tienen residuales <2°. Threshold a 2.5° corta los outliers
     * sistemáticos sin perder matches reales.
     */
    private const val RANSAC_INLIER_THRESHOLD_DEG = 2.5f

    /** Tope de iteraciones LM. */
    private const val LM_MAX_ITERATIONS = 30

    /** Tolerancia relativa de convergencia LM. */
    private const val LM_RELATIVE_TOLERANCE = 1e-6

    /** Drift global aceptable en grados. Si supera, se desconfía y se rechaza. */
    private const val MAX_GLOBAL_DRIFT_DEG = 0.5f

    /**
     * Drift máximo aceptable de un frame individual. Si algún frame se
     * mueve más que esto del seed, se rechaza el resultado completo.
     * Un BA sano refina sub-grado; >2° per frame indica overfitting a
     * matches falsos.
     */
    private const val MAX_PER_FRAME_DRIFT_DEG = 2.0f

    /**
     * Mediana máxima de residuales angulares por par para incluirlo en
     * el problema. Pares con medianResidual alto suelen ser dominados
     * por matches falsos de patrones repetitivos (rejillas, paños,
     * paneles). Filtrarlos pre-LM evita que contaminen la solución.
     */
    private const val MAX_PAIR_MEDIAN_RESIDUAL_DEG = 1.5f

    /** Mínimo % de frames con al menos una constraint válida. */
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

    // ============================================================
    // API PÚBLICA
    // ============================================================

    /**
     * API delgada para uso desde el pipeline. Si el BA falla por cualquier
     * motivo, devuelve los frames sin modificar. Nunca lanza excepción.
     */
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

    /**
     * API completa para diagnóstico. Devuelve todos los detalles para
     * inspección y A/B comparativo.
     */
    fun refine(frames: List<FrameRecord>, lambda: Float = DEFAULT_LAMBDA): Result {
        SunProjectApp.requireOpenCv()

        // Etapa 1: filtrar frames válidos (no zenith, con matriz)
        val validFrames = frames.filter { f ->
            !isZenithFrame(f) && getRotationMatrix(f) != null && f.hfovDeg != null && f.vfovDeg != null
        }
        if (validFrames.size < 4) {
            return failedResult(frames, "too_few_valid_frames=${validFrames.size}")
        }

        Log.i(TAG, "starting BA on ${validFrames.size} non-zenith frames, lambda=$lambda")

        // Etapa 2: determinar vecindades (pares con overlap esperado)
        val seedMatrices = validFrames.map { getRotationMatrix(it)!! }
        val pairs = computeNeighborPairs(seedMatrices)
        Log.i(TAG, "neighbors: ${pairs.size} pairs from ${validFrames.size} frames")

        // Etapa 3: extraer matches por par
        val pairData = extractAllPairMatches(validFrames, seedMatrices, pairs)
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

        val usablePairs = pairData.filter { pd ->
            pd.matches.size >= MIN_MATCHES_PER_PAIR &&
                    pd.medianResidualDeg <= MAX_PAIR_MEDIAN_RESIDUAL_DEG
        }
        if (usablePairs.isEmpty()) {
            return failedResult(frames, "no_usable_pairs_after_medres_filter", pairStats)
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

        // Validación de cobertura: ¿qué fracción de frames tiene al menos un par usable?
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

        // Etapa 4: parametrizar con Rodrigues. 3 floats por frame.
        val nFrames = validFrames.size
        val seedRvecs = seedMatrices.map { matrixToRotvec(it) }
        val initialPoint = DoubleArray(nFrames * 3) { idx ->
            val frameIdx = idx / 3
            val coordIdx = idx % 3
            seedRvecs[frameIdx][coordIdx]
        }

        // Etapa 5: armar el problema y resolverlo
        val problem = buildLeastSquaresProblem(
            usablePairs = usablePairs,
            seedRvecs = seedRvecs,
            initialPoint = initialPoint,
            lambda = lambda.toDouble()
        )

        val optimizer = LevenbergMarquardtOptimizer()
        val initialCost = computeProblemCost(problem, ArrayRealVector(initialPoint))

        val optimum = try {
            optimizer.optimize(problem)
        } catch (t: Throwable) {
            Log.e(TAG, "LM optimizer threw", t)
            return failedResult(frames, "lm_threw=${t.message}", pairStats)
        }

        val finalCost = optimum.cost * optimum.cost  // optimum.cost es RMS, no sum-squared
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

        // Etapa 6: extraer rvecs refinados y convertir a matrices
        val solution = optimum.point.toArray()
        val refinedMatrices = (0 until nFrames).map { frameIdx ->
            val rvec = doubleArrayOf(
                solution[frameIdx * 3],
                solution[frameIdx * 3 + 1],
                solution[frameIdx * 3 + 2]
            )
            rotvecToMatrix(rvec)
        }

        // Drift check (red de seguridad explícita)
        val driftStats = computeDriftStats(seedMatrices, refinedMatrices)
        Log.i(
            TAG,
            "drift globalDeg=${"%.3f".format(driftStats.globalDeg)} " +
                    "medianFrameDeg=${"%.3f".format(driftStats.medianPerFrameDeg)} " +
                    "maxFrameDeg=${"%.3f".format(driftStats.maxPerFrameDeg)}"
        )

        // Log per-frame drift para diagnóstico (qué frame se movió cuánto).
        validFrames.forEachIndexed { idx, f ->
            val seedT = transpose3x3F(seedMatrices[idx])
            val resid = mulMat3Mat3F(refinedMatrices[idx], seedT)
            val rvec = matrixToRotvec(resid)
            val angDeg = Math.toDegrees(
                sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2])
            ).toFloat()
            Log.d(TAG, "frame_drift ${f.frameId} angleDeg=${"%.3f".format(angDeg)}")
        }

        if (driftStats.globalDeg > MAX_GLOBAL_DRIFT_DEG) {
            return failedResult(
                frames,
                "global_drift_exceeded=${"%.3f".format(driftStats.globalDeg)} " +
                        "max=$MAX_GLOBAL_DRIFT_DEG",
                pairStats
            )
        }

        if (driftStats.maxPerFrameDeg > MAX_PER_FRAME_DRIFT_DEG) {
            return failedResult(
                frames,
                "per_frame_drift_exceeded max=${"%.3f".format(driftStats.maxPerFrameDeg)} " +
                        "threshold=$MAX_PER_FRAME_DRIFT_DEG",
                pairStats
            )
        }

        // Etapa 7: construir lista de FrameRecord refinados.
        // Frames no incluidos en el BA (Z0) se devuelven sin tocar.
        val refinedById = validFrames.zip(refinedMatrices).associate { (frame, m) ->
            frame.frameId to m
        }

        val outFrames = frames.map { f ->
            val newM = refinedById[f.frameId]
            if (newM == null) {
                f
            } else {
                f.copy(
                    rotationM00 = newM[0], rotationM01 = newM[1], rotationM02 = newM[2],
                    rotationM10 = newM[3], rotationM11 = newM[4], rotationM12 = newM[5],
                    rotationM20 = newM[6], rotationM21 = newM[7], rotationM22 = newM[8]
                )
            }
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
    // HELPERS - VECINDADES
    // ============================================================

    /** Forward de un frame en world: tercera columna de R (camera looking at -Z_dev → forward = -R*ẑ_dev = -col2 negada) */
    private fun frameForwardInWorld(R: FloatArray): FloatArray {
        // El "forward" de la cámara back-portrait apunta en -Z_dev del device.
        // En world: forward_world = R * (0, 0, -1) = -col2(R)
        return floatArrayOf(-R[2], -R[5], -R[8])
    }

    private fun computeNeighborPairs(seedMatrices: List<FloatArray>): List<Pair<Int, Int>> {
        val n = seedMatrices.size
        val forwards = seedMatrices.map { frameForwardInWorld(it) }
        val cosThreshold = cos(Math.toRadians(MAX_PAIR_ANGLE_DEG.toDouble())).toFloat()
        val out = mutableListOf<Pair<Int, Int>>()
        for (i in 0 until n) {
            for (j in (i + 1) until n) {
                val dot = forwards[i][0] * forwards[j][0] +
                        forwards[i][1] * forwards[j][1] +
                        forwards[i][2] * forwards[j][2]
                if (dot >= cosThreshold) {
                    out.add(i to j)
                }
            }
        }
        return out
    }

    // ============================================================
    // HELPERS - FEATURE EXTRACTION & MATCHING
    // ============================================================

    private data class PairData(
        val i: Int,
        val j: Int,
        /** Matches finales: cada elemento es (ray_i_dev, ray_j_dev), unitarios. */
        val matches: List<Pair<FloatArray, FloatArray>>,
        val matchesRaw: Int,
        val medianResidualDeg: Float
    )

    private fun extractAllPairMatches(
        frames: List<FrameRecord>,
        seedMatrices: List<FloatArray>,
        pairs: List<Pair<Int, Int>>
    ): List<PairData> {
        // Detectar features una sola vez por frame (un mismo frame puede aparecer en N pares).
        val features = frames.indices.map { idx ->
            detectFeatures(frames[idx])
        }

        val out = mutableListOf<PairData>()
        for ((i, j) in pairs) {
            val pd = matchPair(
                frameI = frames[i], featI = features[i], rotI = seedMatrices[i], indexI = i,
                frameJ = frames[j], featJ = features[j], rotJ = seedMatrices[j], indexJ = j
            )
            out.add(pd)
        }

        // Liberar Mats de descriptors (los keypoints son data clases, GC se encarga)
        features.forEach { it?.descriptors?.release() }

        return out
    }

    private data class FrameFeatures(
        val keypoints: List<KeyPoint>,
        val descriptors: Mat,
        /** Escala aplicada al downsample (para mapear KeyPoint.pt → coord original). */
        val scale: Float,
        /** Dimensiones originales de la imagen. */
        val origW: Int,
        val origH: Int
    )

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
        frameI: FrameRecord, featI: FrameFeatures?, rotI: FloatArray, indexI: Int,
        frameJ: FrameRecord, featJ: FrameFeatures?, rotJ: FloatArray, indexJ: Int
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

        // Lowe ratio
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

        // Filtro geométrico contra seed IMU. Para cada match, computamos el
        // ángulo entre ray_i_world (con R_imu_i) y ray_j_world (con R_imu_j).
        // Si supera RANSAC_INLIER_THRESHOLD_DEG, es probable mismatch.
        val hfovI = frameI.hfovDeg!!
        val vfovI = frameI.vfovDeg!!
        val hfovJ = frameJ.hfovDeg!!
        val vfovJ = frameJ.vfovDeg!!

        val inliers = mutableListOf<Pair<FloatArray, FloatArray>>()
        val residualsDeg = mutableListOf<Float>()
        val cosThr = cos(Math.toRadians(RANSAC_INLIER_THRESHOLD_DEG.toDouble())).toFloat()

        for (m in rawMatches) {
            val kpI = featI.keypoints[m.queryIdx]
            val kpJ = featJ.keypoints[m.trainIdx]

            val pxI = (kpI.pt.x / featI.scale).toFloat()
            val pyI = (kpI.pt.y / featI.scale).toFloat()
            val pxJ = (kpJ.pt.x / featJ.scale).toFloat()
            val pyJ = (kpJ.pt.y / featJ.scale).toFloat()

            val rayIDev = pixelToDeviceRay(pxI, pyI, featI.origW, featI.origH, hfovI, vfovI)
            val rayJDev = pixelToDeviceRay(pxJ, pyJ, featJ.origW, featJ.origH, hfovJ, vfovJ)

            val rayIWorld = mulMat3Vec(rotI, rayIDev)
            val rayJWorld = mulMat3Vec(rotJ, rayJDev)

            val dot = rayIWorld[0] * rayJWorld[0] +
                    rayIWorld[1] * rayJWorld[1] +
                    rayIWorld[2] * rayJWorld[2]

            if (dot >= cosThr) {
                inliers.add(rayIDev to rayJDev)
                residualsDeg.add(Math.toDegrees(acos(dot.coerceIn(-1f, 1f).toDouble())).toFloat())
            }
        }

        val medRes = if (residualsDeg.isNotEmpty()) {
            residualsDeg.sorted()[residualsDeg.size / 2]
        } else 0f

        return PairData(indexI, indexJ, inliers, matchesRaw, medRes)
    }

    /**
     * Pixel del frame original → rayo unitario en device frame.
     * Asume back camera portrait: B = diag(1, -1, -1) device→cam OpenCV.
     */
    private fun pixelToDeviceRay(
        px: Float, py: Float,
        imgW: Int, imgH: Int,
        hfovDeg: Float, vfovDeg: Float
    ): FloatArray {
        val tanHalfH = kotlin.math.tan(Math.toRadians(hfovDeg / 2.0)).toFloat()
        val tanHalfV = kotlin.math.tan(Math.toRadians(vfovDeg / 2.0)).toFloat()
        val nx = (px / imgW.toFloat()) * 2f - 1f          // [-1, 1] left→right
        val ny = (py / imgH.toFloat()) * 2f - 1f          // [-1, 1] top→bottom
        // En cam OpenCV: x = nx * tanH, y = ny * tanV, z = 1
        val xCam = nx * tanHalfH
        val yCam = ny * tanHalfV
        val zCam = 1f
        // device frame: B = diag(1, -1, -1). x_dev=x_cam, y_dev=-y_cam, z_dev=-z_cam
        val xDev = xCam
        val yDev = -yCam
        val zDev = -zCam
        // normalizar
        val n = sqrt(xDev * xDev + yDev * yDev + zDev * zDev)
        return floatArrayOf(xDev / n, yDev / n, zDev / n)
    }

    // ============================================================
    // HELPERS - LEAST SQUARES PROBLEM
    // ============================================================

    private fun buildLeastSquaresProblem(
        usablePairs: List<PairData>,
        seedRvecs: List<DoubleArray>,
        initialPoint: DoubleArray,
        lambda: Double
    ): org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem {
        val nFrames = seedRvecs.size
        val totalMatches = usablePairs.sumOf { it.matches.size }

        // Residuales: 3 por match + 3 por frame (regularización)
        val nResiduals = totalMatches * 3 + nFrames * 3
        val target = ArrayRealVector(DoubleArray(nResiduals)) // todo cero

        val sqrtLambda = sqrt(lambda)

        val model = MultivariateJacobianFunction { params ->
            val p = params.toArray()
            val residual = DoubleArray(nResiduals)

            // Cachear matrices R_i y derivadas para esta evaluación
            val rotMatrices = Array(nFrames) { idx ->
                rotvecToMatrixD(p[idx * 3], p[idx * 3 + 1], p[idx * 3 + 2])
            }

            // Bloque 1: residuales de match
            var rIdx = 0
            for (pair in usablePairs) {
                val Ri = rotMatrices[pair.i]
                val Rj = rotMatrices[pair.j]
                for ((rayIDev, rayJDev) in pair.matches) {
                    val rIw = mulMat3VecD(Ri, rayIDev)
                    val rJw = mulMat3VecD(Rj, rayJDev)
                    residual[rIdx++] = rIw[0] - rJw[0]
                    residual[rIdx++] = rIw[1] - rJw[1]
                    residual[rIdx++] = rIw[2] - rJw[2]
                }
            }

            // Bloque 2: regularización al seed IMU
            for (frameIdx in 0 until nFrames) {
                val seed = seedRvecs[frameIdx]
                residual[rIdx++] = sqrtLambda * (p[frameIdx * 3] - seed[0])
                residual[rIdx++] = sqrtLambda * (p[frameIdx * 3 + 1] - seed[1])
                residual[rIdx++] = sqrtLambda * (p[frameIdx * 3 + 2] - seed[2])
            }

            // Jacobiano numérico via diferencias finitas. Apache Commons
            // soporta delegar esto al optimizer si retornamos solo residual,
            // pero requiere un wrapper. Más simple: lo computamos in-place
            // usando central differences ligeras (fwd diff con paso 1e-5).
            val nParams = p.size
            val jacobian = Array2DRowRealMatrix(nResiduals, nParams)
            val eps = 1e-5
            for (k in 0 until nParams) {
                val savedVal = p[k]
                p[k] = savedVal + eps
                val resPlus = evaluateResiduals(p, usablePairs, seedRvecs, sqrtLambda, nFrames, nResiduals)
                p[k] = savedVal
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
        seedRvecs: List<DoubleArray>,
        sqrtLambda: Double,
        nFrames: Int,
        nResiduals: Int
    ): DoubleArray {
        val residual = DoubleArray(nResiduals)
        val rotMatrices = Array(nFrames) { idx ->
            rotvecToMatrixD(p[idx * 3], p[idx * 3 + 1], p[idx * 3 + 2])
        }
        var rIdx = 0
        for (pair in usablePairs) {
            val Ri = rotMatrices[pair.i]
            val Rj = rotMatrices[pair.j]
            for ((rayIDev, rayJDev) in pair.matches) {
                val rIw = mulMat3VecD(Ri, rayIDev)
                val rJw = mulMat3VecD(Rj, rayJDev)
                residual[rIdx++] = rIw[0] - rJw[0]
                residual[rIdx++] = rIw[1] - rJw[1]
                residual[rIdx++] = rIw[2] - rJw[2]
            }
        }
        for (frameIdx in 0 until nFrames) {
            val seed = seedRvecs[frameIdx]
            residual[rIdx++] = sqrtLambda * (p[frameIdx * 3] - seed[0])
            residual[rIdx++] = sqrtLambda * (p[frameIdx * 3 + 1] - seed[1])
            residual[rIdx++] = sqrtLambda * (p[frameIdx * 3 + 2] - seed[2])
        }
        return residual
    }

    private fun computeProblemCost(
        problem: org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem,
        params: RealVector
    ): Double {
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
    // HELPERS - DRIFT METRICS
    // ============================================================

    private data class DriftStats(
        val globalDeg: Float,
        val medianPerFrameDeg: Float,
        val maxPerFrameDeg: Float
    )

    /**
     * Drift global: rotación promedio entre seed y refined. Computado como
     * el ángulo de la rotación residual (R_refined_i · R_seed_i^T) promediado
     * sobre todos los frames. Si las correcciones son aleatorias entre frames,
     * el promedio es ~0; si todos los frames se rotan en la misma dirección
     * (=BA derivó la geometría absoluta), el promedio es grande.
     *
     * Drift per-frame: ángulo de rotación residual de cada frame individual.
     */
    private fun computeDriftStats(
        seed: List<FloatArray>,
        refined: List<FloatArray>
    ): DriftStats {
        require(seed.size == refined.size)
        val perFrameAnglesDeg = mutableListOf<Float>()
        // Sumamos los rvecs de la rotación residual para sacar el "promedio direccional"
        var sumRx = 0.0
        var sumRy = 0.0
        var sumRz = 0.0
        for (i in seed.indices) {
            val seedT = transpose3x3F(seed[i])
            val resid = mulMat3Mat3F(refined[i], seedT)
            val rvecD = matrixToRotvec(resid)
            val angleDeg = Math.toDegrees(
                sqrt(rvecD[0] * rvecD[0] + rvecD[1] * rvecD[1] + rvecD[2] * rvecD[2])
            ).toFloat()
            perFrameAnglesDeg.add(angleDeg)
            sumRx += rvecD[0]
            sumRy += rvecD[1]
            sumRz += rvecD[2]
        }
        val n = seed.size
        val meanRvecMag = sqrt(
            (sumRx / n) * (sumRx / n) + (sumRy / n) * (sumRy / n) + (sumRz / n) * (sumRz / n)
        )
        val globalDeg = Math.toDegrees(meanRvecMag).toFloat()
        val sortedAngles = perFrameAnglesDeg.sorted()
        val median = sortedAngles[sortedAngles.size / 2]
        val maxA = sortedAngles.last()
        return DriftStats(globalDeg, median, maxA)
    }

    // ============================================================
    // HELPERS - GEOMETRÍA / RODRIGUES
    // ============================================================

    /** Matriz row-major 3x3 (Float) → rotvec (3 doubles). */
    private fun matrixToRotvec(R: FloatArray): DoubleArray {
        val src = Mat(3, 3, CvType.CV_64F)
        val arr = DoubleArray(9) { R[it].toDouble() }
        src.put(0, 0, *arr)
        val dst = Mat()
        Calib3d.Rodrigues(src, dst)
        val out = DoubleArray(3)
        dst.get(0, 0, out)
        src.release()
        dst.release()
        return out
    }

    /** rotvec (3 doubles) → matriz row-major 3x3 (Float). */
    private fun rotvecToMatrix(rvec: DoubleArray): FloatArray {
        val matD = rotvecToMatrixD(rvec[0], rvec[1], rvec[2])
        return FloatArray(9) { matD[it].toFloat() }
    }

    /** rotvec → matriz row-major 3x3 (DoubleArray) — versión inline rápida. */
    private fun rotvecToMatrixD(rx: Double, ry: Double, rz: Double): DoubleArray {
        val theta = sqrt(rx * rx + ry * ry + rz * rz)
        if (theta < 1e-12) {
            return doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        }
        val kx = rx / theta
        val ky = ry / theta
        val kz = rz / theta
        val s = sin(theta)
        val c = cos(theta)
        val v = 1 - c
        return doubleArrayOf(
            kx * kx * v + c,        kx * ky * v - kz * s,   kx * kz * v + ky * s,
            ky * kx * v + kz * s,   ky * ky * v + c,        ky * kz * v - kx * s,
            kz * kx * v - ky * s,   kz * ky * v + kx * s,   kz * kz * v + c
        )
    }

    private fun mulMat3Vec(R: FloatArray, v: FloatArray): FloatArray {
        return floatArrayOf(
            R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
            R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]
        )
    }

    private fun mulMat3VecD(R: DoubleArray, v: FloatArray): DoubleArray {
        return doubleArrayOf(
            R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
            R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]
        )
    }

    private fun transpose3x3F(R: FloatArray): FloatArray = floatArrayOf(
        R[0], R[3], R[6],
        R[1], R[4], R[7],
        R[2], R[5], R[8]
    )

    private fun mulMat3Mat3F(A: FloatArray, B: FloatArray): FloatArray = floatArrayOf(
        A[0] * B[0] + A[1] * B[3] + A[2] * B[6],
        A[0] * B[1] + A[1] * B[4] + A[2] * B[7],
        A[0] * B[2] + A[1] * B[5] + A[2] * B[8],
        A[3] * B[0] + A[4] * B[3] + A[5] * B[6],
        A[3] * B[1] + A[4] * B[4] + A[5] * B[7],
        A[3] * B[2] + A[4] * B[5] + A[5] * B[8],
        A[6] * B[0] + A[7] * B[3] + A[8] * B[6],
        A[6] * B[1] + A[7] * B[4] + A[8] * B[7],
        A[6] * B[2] + A[7] * B[5] + A[8] * B[8]
    )

    // ============================================================
    // HELPERS - FRAMERECORD
    // ============================================================

    private fun isZenithFrame(f: FrameRecord): Boolean {
        return f.targetPitchDeg >= 80f || f.ringId.contains("Z", ignoreCase = true)
    }

    private fun getRotationMatrix(f: FrameRecord): FloatArray? {
        val m00 = f.rotationM00 ?: return null
        val m01 = f.rotationM01 ?: return null
        val m02 = f.rotationM02 ?: return null
        val m10 = f.rotationM10 ?: return null
        val m11 = f.rotationM11 ?: return null
        val m12 = f.rotationM12 ?: return null
        val m20 = f.rotationM20 ?: return null
        val m21 = f.rotationM21 ?: return null
        val m22 = f.rotationM22 ?: return null
        return floatArrayOf(m00, m01, m02, m10, m11, m12, m20, m21, m22)
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