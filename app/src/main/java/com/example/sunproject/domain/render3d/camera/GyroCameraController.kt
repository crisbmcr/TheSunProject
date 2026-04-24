package com.example.sunproject.domain.render3d.camera

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.opengl.Matrix
import android.util.Log

private const val TAG = "GyroCameraController"

/**
 * Controller de cámara para la vista 3D que usa el giroscopio del dispositivo
 * (TYPE_ROTATION_VECTOR) para alinear la cámara virtual con la orientación
 * real del teléfono.
 *
 * Pipeline de conversión:
 *  1. SensorManager.getRotationMatrixFromVector → R_world_from_device (4x4).
 *  2. Transponer → R_device_from_world = viewMatrix base.
 *  3. Pre-multiplicar por rotación alrededor de Z por yawCorrectionDeg
 *     (corrección Fase 3: gyro-N → true-N).
 *
 * El listener es stateful: mantiene el último viewMatrix computado. El renderer
 * lo consulta cada frame con currentViewMatrix().
 *
 * Uso típico:
 *   val controller = GyroCameraController(sensorManager, yawCorrectionDeg)
 *   controller.start()
 *   // ...en onDrawFrame:
 *   renderer.updateViewMatrix(controller.currentViewMatrix())
 *   // ...en onPause:
 *   controller.stop()
 */
class GyroCameraController(
    private val sensorManager: SensorManager,
    private val yawCorrectionDeg: Float = 0f
) : SensorEventListener {

    private val rotationVectorSensor: Sensor? =
        sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    // Buffers reutilizables para evitar allocations en el hot path del sensor.
    private val rotationMatrixWorldFromDevice = FloatArray(16)
    private val rotationMatrixDeviceFromWorld = FloatArray(16)
    private val yawCorrectionMatrix = FloatArray(16)
    private val viewMatrix = FloatArray(16)

    @Volatile
    private var hasValidReading = false

    init {
        // Pre-computamos la matriz de corrección yaw una vez — no cambia durante la sesión.
        Matrix.setIdentityM(yawCorrectionMatrix, 0)
        // Rotación alrededor del eje Z (Up) del mundo, grados positivos = rotación CCW
        // vista desde +Z. Si la corrección es gyro→true (delta = absAzimuth - measuredAzimuth),
        // el signo directo es el correcto.
        Matrix.rotateM(yawCorrectionMatrix, 0, yawCorrectionDeg, 0f, 0f, 1f)

        Matrix.setIdentityM(viewMatrix, 0)

        Log.i(TAG, "init — rotationVectorSensor=${rotationVectorSensor != null} " +
                "yawCorrectionDeg=$yawCorrectionDeg")
    }

    fun start() {
        val sensor = rotationVectorSensor
        if (sensor == null) {
            Log.w(TAG, "TYPE_ROTATION_VECTOR no disponible en este dispositivo")
            return
        }
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME)
        Log.i(TAG, "start — listener registrado")
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        Log.i(TAG, "stop — listener desregistrado")
    }

    /**
     * Devuelve el viewMatrix actual. Si todavía no hubo lectura del sensor,
     * devuelve identidad (cámara mirando al Sur por la convención ENU).
     * El array es de uso interno — el caller NO debe mutarlo.
     */
    fun currentViewMatrix(): FloatArray = viewMatrix

    fun hasValidReading(): Boolean = hasValidReading

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return

        // 1. Sensor → R_world_from_device (4x4, lo que dice Android).
        SensorManager.getRotationMatrixFromVector(rotationMatrixWorldFromDevice, event.values)

        // 2. Transponer para pasar de "mapea device → world" a "mapea world → device".
        //    Como el frame cámara de OpenGL coincide con el frame device de Android,
        //    esa misma matriz es nuestro viewMatrix base.
        Matrix.transposeM(rotationMatrixDeviceFromWorld, 0, rotationMatrixWorldFromDevice, 0)

        // 3. Aplicar corrección de yaw (Fase 3): rotar el mundo alrededor de Z antes
        //    de pasarlo al frame cámara. En forma matricial:
        //      viewMatrix = R_device_from_world · R_yawCorrection
        Matrix.multiplyMM(
            viewMatrix, 0,
            rotationMatrixDeviceFromWorld, 0,
            yawCorrectionMatrix, 0
        )

        hasValidReading = true
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // No nos preocupamos por accuracy en v1. El fix de Fase 3 ya promedia y descarta
        // lecturas con mucho spread, así que el yawCorrection incoming ya pasó un filtro.
    }
}