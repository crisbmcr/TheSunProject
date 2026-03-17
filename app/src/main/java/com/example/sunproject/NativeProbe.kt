package com.example.sunproject

object NativeProbe {
    init {
        System.loadLibrary("panorama-lib")
    }

    @JvmStatic
    external fun getOpenCvVersion(): String
}