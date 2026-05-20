package com.example.sunproject.domain.horizon

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Rect
import android.util.Log
import org.opencv.core.CvType
import org.opencv.core.Mat

/**
 * Coordina la segmentación de cielo sobre el atlas completo, tileando para
 * adaptarse al tamaño fijo del modelo TFLite (513×513).
 *
 * Estrategia de tiling para atlas equirectangular ~3600×900:
 *
 *   Horizontal:
 *     - 7 tiles estándar sin overlap, en x = 0, 513, 1026, 1539, 2052, 2565, 3078.
 *       Cubren x=[0..3591). Los últimos 9 píxeles (x=[3591..3600)) quedan
 *       afuera con esta tanda.
 *     - 1 tile "seam" con wrap horizontal centrado en el borde x=0/x=W-1.
 *       Toma TILE/2 columnas del lado derecho + TILE/2 del izquierdo y las
 *       concatena. Cubre el borde derecho faltante Y le da contexto continuo
 *       al modelo para features que cruzan el seam (mástiles en el N).
 *     - En la zona donde el seam tile overlapa con tiles estándar (x<256 y
 *       x>=3344), el seam tile escribe ÚLTIMO y por lo tanto pisa al estándar.
 *       Es intencional: el seam tile ve ambos lados del wrap, mejor contexto.
 *
 *   Vertical:
 *     - 2 tiles: y=0 (cubre y=[0..513)) y y=H-TILE (cubre y=[H-TILE..H)).
 *     - Con H<2*TILE (caso atlas 900), overlap forzado de TILE - (H-TILE)
 *       filas. Para H=900, overlap = 126 filas en y=[387..513).
 *     - En el overlap el tile inferior escribe último → gana. Es intencional:
 *       el tile inferior cae sobre la zona del horizonte, queremos su voto.
 *
 * Total para atlas 3600×900: 8 X × 2 Y = 16 inferencias TFLite.
 * Tiempo estimado: ~4s con GPU delegate, ~30s en CPU fallback.
 *
 * Llamar SIEMPRE desde background thread. La construcción es costosa (~600ms
 * de carga del modelo) — reusar la instancia si se hacen varias detecciones.
 */
class SkyAtlasSegmenter(context: Context) : AutoCloseable {

    companion object {
        private const val TAG = "SkyAtlasSegmenter"
        private val TILE_SIZE = SkySegmenterTFLite.INPUT_SIZE  // 513
    }

    private val segmenter = SkySegmenterTFLite(context)
    val usingGpu: Boolean get() = segmenter.usingGpu

    /**
     * Segmenta el atlas completo. Devuelve una Mat CV_8UC1 del mismo tamaño
     * que el bitmap, con 255=cielo y 0=no-cielo. La Mat es responsabilidad
     * del caller — debe liberarla con release().
     */
    fun segmentAtlas(atlas: Bitmap): Mat {
        val w = atlas.width
        val h = atlas.height
        require(w >= TILE_SIZE && h >= TILE_SIZE) {
            "Atlas ${w}×${h} más chico que el tile ${TILE_SIZE}×${TILE_SIZE}"
        }

        val startNs = System.nanoTime()
        val skyMask = Mat.zeros(h, w, CvType.CV_8UC1)

        val yPositions = listOf(0, h - TILE_SIZE)
        val xPositionsStandard = (0 until w step TILE_SIZE)
            .filter { it + TILE_SIZE <= w }
            .toList()
        val seamX = w - TILE_SIZE / 2  // centrado en el wrap

        var tileCount = 0
        for (y in yPositions) {
            // Tiles estándar (sin wrap)
            for (x in xPositionsStandard) {
                processTile(atlas, x, y, wrap = false, skyMask, w)
                tileCount++
            }
            // Tile seam (con wrap) — se escribe último, pisa a los estándar en su overlap
            processTile(atlas, seamX, y, wrap = true, skyMask, w)
            tileCount++
        }

        val elapsedMs = (System.nanoTime() - startNs) / 1_000_000L
        Log.i(
            TAG,
            "Atlas ${w}×${h} segmentado en $tileCount tiles, total ${elapsedMs}ms (GPU=$usingGpu)"
        )
        return skyMask
    }

    /**
     * Extrae el tile en (x, y) del atlas, lo procesa con el segmenter, y
     * escribe la máscara resultante de vuelta en skyMask.
     *
     * Si wrap=true, el tile se construye combinando el lado derecho del atlas
     * (desde x hasta el borde) con el lado izquierdo (desde 0 hasta completar
     * TILE_SIZE columnas), simulando la continuidad equirectangular.
     */
    private fun processTile(
        atlas: Bitmap,
        x: Int,
        y: Int,
        wrap: Boolean,
        skyMask: Mat,
        atlasW: Int
    ) {
        // Construir bitmap del tile.
        val tileBitmap = Bitmap.createBitmap(TILE_SIZE, TILE_SIZE, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(tileBitmap)

        if (wrap) {
            val rightWidth = atlasW - x
            val leftWidth = TILE_SIZE - rightWidth

            // Lado derecho del atlas → parte izquierda del tile
            canvas.drawBitmap(
                atlas,
                Rect(x, y, atlasW, y + TILE_SIZE),
                Rect(0, 0, rightWidth, TILE_SIZE),
                null
            )
            // Lado izquierdo del atlas → parte derecha del tile
            canvas.drawBitmap(
                atlas,
                Rect(0, y, leftWidth, y + TILE_SIZE),
                Rect(rightWidth, 0, TILE_SIZE, TILE_SIZE),
                null
            )
        } else {
            canvas.drawBitmap(
                atlas,
                Rect(x, y, x + TILE_SIZE, y + TILE_SIZE),
                Rect(0, 0, TILE_SIZE, TILE_SIZE),
                null
            )
        }

        // Inferencia.
        val mask = segmenter.segmentSky(tileBitmap)
        tileBitmap.recycle()

        // Escribir la máscara resultante en skyMask.
        writeTileMaskToAtlas(mask, x, y, wrap, skyMask, atlasW)
    }

    /**
     * Escribe la máscara booleana del tile en (x, y) del skyMask. Si wrap,
     * parte la escritura en dos para respetar la continuidad equirectangular.
     */
    private fun writeTileMaskToAtlas(
        tileMask: Array<BooleanArray>,
        x: Int,
        y: Int,
        wrap: Boolean,
        skyMask: Mat,
        atlasW: Int
    ) {
        val rowBytes = ByteArray(TILE_SIZE)
        for (ty in 0 until TILE_SIZE) {
            // Bool → byte por fila (255 si sky, 0 si no).
            for (tx in 0 until TILE_SIZE) {
                rowBytes[tx] = if (tileMask[ty][tx]) 255.toByte() else 0
            }

            val atlasY = y + ty
            if (wrap) {
                val rightWidth = atlasW - x
                // Parte izquierda del tile → columnas [x..atlasW) del atlas
                skyMask.put(atlasY, x, rowBytes.copyOfRange(0, rightWidth))
                // Parte derecha del tile → columnas [0..leftWidth) del atlas
                skyMask.put(atlasY, 0, rowBytes.copyOfRange(rightWidth, TILE_SIZE))
            } else {
                skyMask.put(atlasY, x, rowBytes)
            }
        }
    }

    override fun close() {
        segmenter.close()
    }
}