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

        val total = prevWeight + weight
        val prev = pixels[idx]

        val r = ((Color.red(prev) * prevWeight + Color.red(color) * weight) / total)
            .roundToInt().coerceIn(0, 255)
        val g = ((Color.green(prev) * prevWeight + Color.green(color) * weight) / total)
            .roundToInt().coerceIn(0, 255)
        val b = ((Color.blue(prev) * prevWeight + Color.blue(color) * weight) / total)
            .roundToInt().coerceIn(0, 255)

        pixels[idx] = Color.argb(255, r, g, b)
        weightSums[idx] = total
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