package com.example.sunproject

import android.content.Context
import android.graphics.BitmapFactory
import android.hardware.SensorManager
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.example.sunproject.domain.atlas.AtlasProjector
import com.example.sunproject.domain.render3d.PanoramaRenderer
import com.example.sunproject.domain.render3d.camera.GyroCameraController
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
    private var gyroController: GyroCameraController? = null

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

            // 1. Obtener la corrección de yaw de Fase 3 si está disponible.
//    Si no hay corrección (sesión sin GPS, o spread alto), devuelve 0f
//    y la cámara queda en gyro-N puro — consistente con el fallback del atlas.
            val yawCorrectionDeg = AtlasProjector.gyroToTrueNorthCorrectionDeg()
            Log.i(TAG, "Corrección yaw Fase 3: $yawCorrectionDeg°")

// 2. Crear el renderer y el controller.
            val sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
            val newRenderer = PanoramaRenderer(bitmap)
            val newController = GyroCameraController(sensorManager, yawCorrectionDeg)

// 3. Conectar: el renderer lee la viewMatrix del controller en cada frame.
            newRenderer.setExternalViewMatrix(newController.currentViewMatrix())

            renderer = newRenderer
            gyroController = newController

            glView.setRenderer(newRenderer)
            glView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY


            // 4. Arrancar el listener del sensor. Lo hacemos incondicional porque:
//    - Si onResume ya pasó (flujo típico), gyroController era null entonces y
//      el ?.start() de onResume fue no-op. Tenemos que arrancarlo ahora.
//    - Si onResume todavía no pasó, registerListener es idempotente y no hay problema.
            newController.start()

// Actualizar HUD con info extra.
            val correctionStr = if (yawCorrectionDeg != 0f) {
                "yaw fix: %.2f°".format(yawCorrectionDeg)
            } else {
                "gyro-N puro (sin fix)"
            }
            hudText.text = "Panorama 3D — ${bitmap.width}x${bitmap.height}\n" +
                    "modo: giroscopio  |  $correctionStr"
        }
    }

    override fun onResume() {
        super.onResume()
        if (renderer != null) glView.onResume()
        gyroController?.start()
    }

    override fun onPause() {
        super.onPause()
        gyroController?.stop()
        if (renderer != null) glView.onPause()
    }
}