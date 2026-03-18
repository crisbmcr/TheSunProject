package com.example.sunproject.domain.atlas

class SkyAtlas(val config: AtlasConfig) {
    val width: Int = config.widthPx
    val height: Int = config.heightPx
    val pixels: IntArray = IntArray(width * height) { 0xFF202020.toInt() }

    fun index(x: Int, y: Int): Int = y * width + x
}