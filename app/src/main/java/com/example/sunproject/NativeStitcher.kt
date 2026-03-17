package com.example.sunproject

object NativeStitcher {
    init { System.loadLibrary("panorama-lib") }

    external fun checkStitcherAvailable(): Boolean
    external fun selfTestStitch(width: Int = 1280, height: Int = 720, jpegQuality: Int = 90): ByteArray?

    external fun stitchFromFiles(
        paths: Array<String>,
        jpegQuality: Int,
        maxDim: Int,
        closeLoop: Boolean
    ): ByteArray?
}