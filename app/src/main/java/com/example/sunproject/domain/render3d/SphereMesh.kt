package com.example.sunproject.domain.render3d

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.ShortBuffer
import kotlin.math.cos
import kotlin.math.sin

/**
 * Genera una esfera UV centrada en el origen, radio 1, con el polo norte en +Z (Up ENU).
 *
 * Convenciones:
 *  - Stack 0 es el polo norte (+Z, cenit).
 *  - Stack N es el polo sur (-Z, nadir).
 *  - Slice 0 arranca apuntando a +Y (Norte ENU) y gira hacia +X (Este ENU).
 *  - UV: u ∈ [0,1] recorre la longitud (0 = Norte, 0.25 = Este, 0.5 = Sur, 0.75 = Oeste).
 *        v ∈ [0,1] recorre la latitud (0 = cenit, 1 = nadir).
 *
 * Esto coincide con la convención equirectangular del atlas generado por el proyecto:
 *  - Columna x=0 del atlas mapea a azimut 0° (Norte).
 *  - Fila y=0 del atlas mapea a altitud +90° (cenit).
 *
 * Seam longitudinal: se duplica el meridiano inicial con un set de vértices adicional
 * en u=1.0 para evitar interpolación inversa de UV en el wrap.
 */
class SphereMesh(
    private val slices: Int = 60,
    private val stacks: Int = 30
) {
    val vertexBuffer: FloatBuffer
    val texCoordBuffer: FloatBuffer
    val indexBuffer: ShortBuffer
    val indexCount: Int

    init {
        // (slices + 1) porque duplicamos la costura en u=1.0
        val vertexRingCount = slices + 1
        val totalVertices = vertexRingCount * (stacks + 1)

        val positions = FloatArray(totalVertices * 3)
        val texCoords = FloatArray(totalVertices * 2)

        var vIdx = 0
        var tIdx = 0

        for (stack in 0..stacks) {
            val v = stack.toFloat() / stacks                  // 0 en polo norte, 1 en polo sur
            val altitudeRad = (Math.PI / 2.0) - v * Math.PI   // +π/2 en cenit, -π/2 en nadir
            val cosAlt = cos(altitudeRad).toFloat()
            val sinAlt = sin(altitudeRad).toFloat()

            for (slice in 0..slices) {
                val u = slice.toFloat() / slices              // 0 en Norte, 0.25 en Este...
                val azimuthRad = u * 2.0 * Math.PI            // azimut CW desde Norte
                val cosAz = cos(azimuthRad).toFloat()
                val sinAz = sin(azimuthRad).toFloat()

                // ENU cartesianas: X=Este, Y=Norte, Z=Up
                val x = cosAlt * sinAz
                val y = cosAlt * cosAz
                val z = sinAlt

                positions[vIdx++] = x
                positions[vIdx++] = y
                positions[vIdx++] = z

                texCoords[tIdx++] = u
                texCoords[tIdx++] = v
            }
        }

        // Índices: dos triángulos por quad, CCW visto desde adentro (back-face culling OFF o
        // invertimos winding — acá usamos el orden que se ve correcto desde el centro).
        // Para skybox mirando desde el interior, el winding se invierte respecto a una esfera
        // normal: los triángulos van CW desde afuera, CCW desde adentro.
        val indices = ShortArray(slices * stacks * 6)
        var iIdx = 0
        for (stack in 0 until stacks) {
            for (slice in 0 until slices) {
                val topLeft = (stack * vertexRingCount + slice).toShort()
                val topRight = (stack * vertexRingCount + slice + 1).toShort()
                val botLeft = ((stack + 1) * vertexRingCount + slice).toShort()
                val botRight = ((stack + 1) * vertexRingCount + slice + 1).toShort()

                // Triángulo 1: topLeft, botLeft, topRight
                indices[iIdx++] = topLeft
                indices[iIdx++] = botLeft
                indices[iIdx++] = topRight
                // Triángulo 2: topRight, botLeft, botRight
                indices[iIdx++] = topRight
                indices[iIdx++] = botLeft
                indices[iIdx++] = botRight
            }
        }

        indexCount = indices.size

        vertexBuffer = ByteBuffer
            .allocateDirect(positions.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply { put(positions); position(0) }

        texCoordBuffer = ByteBuffer
            .allocateDirect(texCoords.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply { put(texCoords); position(0) }

        indexBuffer = ByteBuffer
            .allocateDirect(indices.size * 2)
            .order(ByteOrder.nativeOrder())
            .asShortBuffer()
            .apply { put(indices); position(0) }
    }
}