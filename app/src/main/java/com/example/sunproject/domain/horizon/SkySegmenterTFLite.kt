package com.example.sunproject.domain.horizon

import android.content.Context
import android.graphics.Bitmap
import android.util.Log
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

/**
 * Wrapper sobre DeepLab v3 MobileNet V2 entrenado en ADE20K.
 *
 * Modelo: deeplabv3_ade20k.tflite (float16 weights, ~4.4 MB).
 * Input:  Bitmap 513×513 RGB. Bytes UINT8 directos — el modelo hace el
 *         preprocesamiento (normalización a [-1, 1]) internamente.
 * Output: máscara booleana 513×513 donde true = cielo.
 *
 * ADE20K tiene 150 clases. La clase índice 3 es "sky". Todo lo demás
 * (building, tree, mountain, tower, pole, fence, etc.) se considera obstáculo.
 *
 * Esta es la versión MÍNIMA para validar setup. El tiling sobre el atlas
 * 3600×900 viene en la Fase B.
 *
 * Usar con try/finally o use{} para liberar recursos nativos al terminar.
 */
class SkySegmenterTFLite(context: Context) : AutoCloseable {

    companion object {
        private const val TAG = "SkySegmenter"
        private const val MODEL_FILE = "deeplabv3_ade20k.tflite"

        const val INPUT_SIZE = 513
        const val NUM_CLASSES = 151
        const val SKY_CLASS_INDEX = 3
    }

    private val interpreter: Interpreter
    private val gpuDelegate: GpuDelegate?
    val usingGpu: Boolean

    init {
        val modelBuffer = loadModelFromAssets(context, MODEL_FILE)
        val options = Interpreter.Options()

        // Intentar GPU delegate. Si el dispositivo no lo soporta, CPU con 4 threads.
        val compatList = CompatibilityList()
        if (compatList.isDelegateSupportedOnThisDevice) {
            val delegateOptions = compatList.bestOptionsForThisDevice
            gpuDelegate = GpuDelegate(delegateOptions)
            options.addDelegate(gpuDelegate)
            usingGpu = true
            Log.i(TAG, "GPU delegate activado.")
        } else {
            gpuDelegate = null
            options.setNumThreads(4)
            usingGpu = false
            Log.i(TAG, "GPU no soportado. CPU con 4 threads.")
        }

        interpreter = Interpreter(modelBuffer, options)

        // Verificar shapes esperados.
        val inShape = interpreter.getInputTensor(0).shape()
        val outShape = interpreter.getOutputTensor(0).shape()
        Log.i(TAG, "Modelo cargado. Input shape: ${inShape.contentToString()}, Output shape: ${outShape.contentToString()}")
        require(inShape.contentEquals(intArrayOf(1, INPUT_SIZE, INPUT_SIZE, 3))) {
            "Input shape inesperado: ${inShape.contentToString()}. Esperaba [1, $INPUT_SIZE, $INPUT_SIZE, 3]"
        }
        require(outShape.contentEquals(intArrayOf(1, INPUT_SIZE, INPUT_SIZE, NUM_CLASSES))) {
            "Output shape inesperado: ${outShape.contentToString()}. Esperaba [1, $INPUT_SIZE, $INPUT_SIZE, $NUM_CLASSES]"
        }
    }

    /**
     * Segmenta el cielo en un bitmap de 513×513.
     * Devuelve máscara [INPUT_SIZE][INPUT_SIZE] con true=cielo, false=obstáculo.
     */
    fun segmentSky(input: Bitmap): Array<BooleanArray> {
        require(input.width == INPUT_SIZE && input.height == INPUT_SIZE) {
            "Bitmap debe ser ${INPUT_SIZE}×${INPUT_SIZE}, recibido ${input.width}×${input.height}"
        }

        // Preparar input: bytes UINT8 directos (R, G, B por pixel). El modelo
        // internamente convierte a float y normaliza, no hay que hacer nada
        // adicional en Kotlin.
        val inputBuffer = ByteBuffer
            .allocateDirect(INPUT_SIZE * INPUT_SIZE * 3)
            .order(ByteOrder.nativeOrder())
        val pixels = IntArray(INPUT_SIZE * INPUT_SIZE)
        input.getPixels(pixels, 0, INPUT_SIZE, 0, 0, INPUT_SIZE, INPUT_SIZE)
        for (pixel in pixels) {
            inputBuffer.put(((pixel shr 16) and 0xFF).toByte())  // R
            inputBuffer.put(((pixel shr 8) and 0xFF).toByte())   // G
            inputBuffer.put((pixel and 0xFF).toByte())            // B
        }
        inputBuffer.rewind()

        // Output: 513×513×151 logits float32. Es ~152 MB. Buffer grande pero
        // se libera al salir de la función. Si esto resulta ser problema en
        // dispositivos con poca RAM, en Fase B se puede re-exportar el modelo
        // con SemanticPredictions como output (ya con argmax) para reducir a
        // ~1 MB. Por ahora mantenemos logits para flexibilidad.
        val outputBuffer = ByteBuffer
            .allocateDirect(INPUT_SIZE * INPUT_SIZE * NUM_CLASSES * 4)
            .order(ByteOrder.nativeOrder())

        val startNs = System.nanoTime()
        interpreter.run(inputBuffer, outputBuffer)
        val elapsedMs = (System.nanoTime() - startNs) / 1_000_000L

        // Argmax por pixel. Si la clase ganadora es sky → true en la máscara.
        outputBuffer.rewind()
        val floats = outputBuffer.asFloatBuffer()
        val mask = Array(INPUT_SIZE) { BooleanArray(INPUT_SIZE) }
        var skyCount = 0

        for (y in 0 until INPUT_SIZE) {
            for (x in 0 until INPUT_SIZE) {
                val base = (y * INPUT_SIZE + x) * NUM_CLASSES
                var bestClass = 0
                var bestScore = floats.get(base)
                for (c in 1 until NUM_CLASSES) {
                    val score = floats.get(base + c)
                    if (score > bestScore) {
                        bestScore = score
                        bestClass = c
                    }
                }
                if (bestClass == SKY_CLASS_INDEX) {
                    mask[y][x] = true
                    skyCount++
                }
            }
        }

        val skyPct = 100f * skyCount / (INPUT_SIZE * INPUT_SIZE)
        Log.i(
            TAG,
            "Inferencia OK: ${elapsedMs}ms (GPU=$usingGpu). " +
                    "Cielo: $skyCount / ${INPUT_SIZE * INPUT_SIZE} (${"%.1f".format(skyPct)}%)"
        )

        return mask
    }

    override fun close() {
        interpreter.close()
        gpuDelegate?.close()
    }

    private fun loadModelFromAssets(context: Context, fileName: String): MappedByteBuffer {
        val fd = context.assets.openFd(fileName)
        FileInputStream(fd.fileDescriptor).use { fis ->
            val channel = fis.channel
            return channel.map(FileChannel.MapMode.READ_ONLY, fd.startOffset, fd.declaredLength)
        }
    }
}