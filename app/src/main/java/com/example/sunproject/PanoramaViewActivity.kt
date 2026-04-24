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

        // === Configuración del renderer — debe hacerse ANTES que GLSurfaceView se adjunte
        // a la ventana, o sea, sincronicamente en onCreate. Cualquier cosa async (decodificar
        // bitmap, leer session.json, etc.) se hace después y se inyecta al renderer vía setters.
        glView.setEGLContextClientVersion(2)

        val yawCorrectionDeg = AtlasProjector.gyroToTrueNorthCorrectionDeg()
        Log.i(TAG, "Corrección yaw Fase 3: $yawCorrectionDeg°")

        val sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        val newRenderer = PanoramaRenderer()
        val newController = GyroCameraController(sensorManager, yawCorrectionDeg)

        // Conectar renderer ↔ controller. Referencia del array se mantiene; el controller
        // muta el contenido cuando llegan eventos del sensor.
        newRenderer.setExternalViewMatrix(newController.currentViewMatrix())

        renderer = newRenderer
        gyroController = newController

        glView.setRenderer(newRenderer)
        glView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        // HUD inicial mientras carga el bitmap.
        hudText.text = "Panorama 3D — cargando atlas..."

        // Arrancar el sensor ya, aunque el bitmap todavía no esté listo — así la
        // cámara virtual ya está orientada cuando aparezcan los píxeles.
        newController.start()

        // === Carga del bitmap en background. Cuando termine, se inyecta al renderer y
        // en el siguiente frame se hace el GL upload.
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

            newRenderer.setAtlasBitmap(bitmap)

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
        glView.onResume()
        gyroController?.start()
    }

    override fun onPause() {
        super.onPause()
        gyroController?.stop()
        glView.onPause()
    }
}