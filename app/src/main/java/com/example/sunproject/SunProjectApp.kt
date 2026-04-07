package com.example.sunproject

import android.app.Application
import android.util.Log
import org.opencv.android.OpenCVLoader

class SunProjectApp : Application() {

    override fun onCreate() {
        super.onCreate()
        ensureOpenCvReady()
    }

    companion object {
        @Volatile
        private var initialized = false

        fun ensureOpenCvReady(): Boolean {
            if (initialized) return true

            synchronized(this) {
                if (initialized) return true

                // Primero intentamos cargar la lib nativa empaquetada en jniLibs.
                try {
                    System.loadLibrary("opencv_java4")
                    initialized = true
                    Log.i("OpenCV", "opencv_java4 loaded with System.loadLibrary")
                    return true
                } catch (t: Throwable) {
                    Log.w("OpenCV", "System.loadLibrary(opencv_java4) failed, trying OpenCVLoader.initLocal()", t)
                }

                return try {
                    initialized = OpenCVLoader.initLocal()
                    if (initialized) {
                        Log.i("OpenCV", "OpenCV loaded with OpenCVLoader.initLocal")
                    } else {
                        Log.e("OpenCV", "OpenCV initialization failed")
                    }
                    initialized
                } catch (t: Throwable) {
                    Log.e("OpenCV", "OpenCV initialization crashed", t)
                    false
                }
            }
        }

        fun requireOpenCv() {
            check(ensureOpenCvReady()) { "OpenCV no pudo inicializarse" }
        }
    }
}