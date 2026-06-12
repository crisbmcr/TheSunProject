package com.example.sunproject.domain.atlas

import android.graphics.Bitmap
import android.graphics.Color
import java.io.File
import java.io.FileOutputStream
import kotlin.math.roundToInt

class SkyAtlas(val config: AtlasConfig) {
    val width: Int = config.widthPx
    val height: Int = config.heightPx

    val pixels: IntArray = IntArray(width * height) { Color.DKGRAY }
    private val weightSums: FloatArray = FloatArray(width * height)

    fun index(x: Int, y: Int): Int = y * width + x

    fun setPixel(x: Int, y: Int, color: Int) {
        if (x !in 0 until width || y !in 0 until height) return
        pixels[index(x, y)] = color
    }

    companion object {
        /**
         * Seam-sharpness exponent for winner-take-most compositing.
         * 1.0 = legacy-like weighted average (wide blend across overlaps,
         * produces ghosting when residual pose error > ~0.2 deg).
         * 6..10 = near-Voronoi seam selection: the frame with the highest
         * local weight owns the pixel, transition band collapses to the
         * locus where adjacent frame weights are equal (mid-overlap).
         * Converts residual misalignment from a double-edge ghost spanning
         * the whole overlap into a sub-degree geometric step at the seam,
         * which is what HorizonProfileDetector needs (sharp horizon edge).
         */
        const val SEAM_SHARPNESS = 8f
        var seamSharpnessOverride: Float? = null   // runtime A/B sobre sesiones históricas
    }

    fun blendPixel(x: Int, y: Int, color: Int, weight: Float) {
        if (x !in 0 until width || y !in 0 until height) return
        if (weight <= 0f) return

        val idx = index(x, y)
        val prevWeight = weightSums[idx]

        if (prevWeight <= 0f) {
            pixels[idx] = color
            weightSums[idx] = weight
            return
        }

        val p = seamSharpnessOverride ?: SEAM_SHARPNESS
        // Pairwise sharpened alpha: alpha -> 1 when weight >> prevWeight,
        // alpha -> 0 when weight << prevWeight, narrow blend near ties.
        val wp = Math.pow(weight.toDouble(), p.toDouble())
        val pp = Math.pow(prevWeight.toDouble(), p.toDouble())
        val alpha = (wp / (wp + pp)).toFloat()

        val prev = pixels[idx]
        val r = (Color.red(prev) * (1f - alpha) + Color.red(color) * alpha)
            .roundToInt().coerceIn(0, 255)
        val g = (Color.green(prev) * (1f - alpha) + Color.green(color) * alpha)
            .roundToInt().coerceIn(0, 255)
        val b = (Color.blue(prev) * (1f - alpha) + Color.blue(color) * alpha)
            .roundToInt().coerceIn(0, 255)

        pixels[idx] = Color.argb(255, r, g, b)
        // Keep the winner's weight, not the sum: weightSums now means
        // "strongest claim so far", which is what max-weight compositing needs.
        weightSums[idx] = max(prevWeight, weight)
    }

    /**
     * Sobreescribe el pixel y reemplaza el peso acumulado, descartando
     * lo escrito previamente en esa posición. Usado por el Z0 en la cap
     * polar absoluta (altitud >= ZENITH_FADE_FULL_ALT_DEG) donde el Z0
     * es la única fuente correcta y la mezcla con H45 sólo introduce
     * ruido de las esquinas pinhole estiradas.
     *
     * Cualquier blendPixel posterior va a promediar contra este color
     * como si overwritePixel hubiera sido el primer escritor con ese
     * peso. En el pipeline actual el Z0 es el último frame, así que no
     * hay writes posteriores.
     */
    fun overwritePixel(x: Int, y: Int, color: Int, weight: Float) {
        if (x !in 0 until width || y !in 0 until height) return
        if (weight <= 0f) return
        val idx = index(x, y)
        pixels[idx] = color
        weightSums[idx] = weight
    }

    fun hasCoverageAt(x: Int, y: Int): Boolean {
        if (x !in 0 until width || y !in 0 until height) return false
        return weightSums[index(x, y)] > 0f
    }

    fun weightAt(x: Int, y: Int): Float {
        if (x !in 0 until width || y !in 0 until height) return 0f
        return weightSums[index(x, y)]
    }
    fun toBitmap(): Bitmap {
        val bmp = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        bmp.setPixels(pixels, 0, width, 0, 0, width, height)
        return bmp
    }

    fun writePng(outFile: File) {
        outFile.parentFile?.mkdirs()
        FileOutputStream(outFile).use { stream ->
            toBitmap().compress(Bitmap.CompressFormat.PNG, 100, stream)
        }
    }
}