package com.example.sunproject

import android.graphics.BitmapFactory
import android.net.Uri
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.example.sunproject.domain.render3d.PanoramaRenderer
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import androidx.lifecycle.lifecycleScope
import java.io.File

private const val TAG = "PanoramaViewActivity"

class PanoramaViewActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_PANORAMA_PATH = "panorama_path"
    }

    private lateinit var glView: GLSurfaceView
    private lateinit var hudText: TextView
    private var renderer: PanoramaRenderer? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_panorama_view)

        glView = findViewById(R.id.glSurfaceView)
        hudText = findViewById(R.id.hudText)

        val panoramaPath = intent.getStringExtra(EXTRA_PANORAMA_PATH)
        if (panoramaPath.isNullOrBlank()) {
            Log.e(TAG, "Falta $EXTRA_PANORAMA_PATH en el Intent — cerrando.")
            Toast.makeText(this, "Ruta de atlas vacía", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        glView.setEGLContextClientVersion(2)

        lifecycleScope.launch {
            val bitmap = withContext(Dispatchers.IO) {
                runCatching {
                    BitmapFactory.decodeFile(panoramaPath)
                }.getOrNull()
            }

            if (bitmap == null) {
                Log.e(TAG, "No se pudo decodificar el atlas en $panoramaPath")
                hudText.text = "Error cargando atlas"
                Toast.makeText(this@PanoramaViewActivity,
                    "No se pudo abrir $panoramaPath", Toast.LENGTH_LONG).show()
                return@launch
            }

            Log.i(TAG, "Atlas cargado ${bitmap.width}x${bitmap.height} de ${File(panoramaPath).name}")
            hudText.text = "Panorama 3D — ${bitmap.width}x${bitmap.height}\n" +
                    "v1: cámara fija mirando al Norte"

            renderer = PanoramaRenderer(bitmap).also {
                glView.setRenderer(it)
                glView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
            }
        }
    }

    override fun onResume() {
        super.onResume()
        if (renderer != null) glView.onResume()
    }

    override fun onPause() {
        super.onPause()
        if (renderer != null) glView.onPause()
    }
}