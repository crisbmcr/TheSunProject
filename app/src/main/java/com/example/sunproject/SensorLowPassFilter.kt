package com.example.sunproject

/**
 * Implementa un filtro de paso bajo para suavizar los datos de los sensores.
 * Esto es crucial para eliminar el "jitter" o temblor causado por el ruido
 * de los sensores de alta frecuencia.
 */
class SensorLowPassFilter {

    // El factor de suavizado. Un valor más cercano a 0 significa más suavizado
    // pero más retraso. Un valor más cercano a 1 significa menos suavizado
    // pero una respuesta más rápida. 0.15 es un buen punto de partida.
    private val alpha: Float = 0.15f

    private var previousValues: FloatArray? = null

    /**
     * Aplica el filtro de paso bajo a un nuevo conjunto de valores del sensor.
     * @param newValues Los nuevos valores crudos del sensor.
     * @return Los valores suavizados.
     */
    fun filter(newValues: FloatArray): FloatArray {
        if (previousValues == null) {
            previousValues = newValues.clone()
            return previousValues!!
        }

        for (i in newValues.indices) {
            previousValues!![i] = previousValues!![i] + alpha * (newValues[i] - previousValues!![i])
        }

        return previousValues!!
    }
}
