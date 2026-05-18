package com.example.sunproject.domain.render3d.abacus

import com.example.sunproject.domain.solar.SolarChart
import com.example.sunproject.domain.solar.SolarPath
import com.example.sunproject.domain.solar.SolarPosition
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Geometría GL precomputada del ábaco solar para la vista 3D.
 *
 * Convenciones:
 *  - ENU: X=Este, Y=Norte, Z=Up. Idéntico a SphereMesh y AtlasProjector.worldDirectionRad.
 *  - Azimut NOAA: en grados, [0, 360) CW desde el Norte. Se usa directo en la fórmula,
 *    no se normaliza a [-180, 180] como hace SolarPathOverlay 2D — en 3D no hace falta.
 *  - Below-horizon (v1): puntos con altitudeDeg < 0 o belowHorizon=true se cortan.
 *    El strip se cierra y arranca uno nuevo si la curva vuelve a subir.
 *  - Wrap-around: si dos puntos consecutivos tienen |Δaz| > 90°, se corta el strip
 *    actual y arranca uno nuevo. Esto evita líneas atravesando la esfera en saltos
 *    cerca del cenit (caso analemma de 12h en Cauchari, lat ~-23.8°, cerca del
 *    Trópico de Capricornio donde el sol queda casi sobre el observador en verano).
 *  - Radio de proyección: 1.0 (mismo que SphereMesh). El control de "está delante
 *    de la esfera" lo hace el depth state del renderer, no el radio.
 *
 * Costo: un path diario tiene ~145 puntos, hay 12 dailies + 13 hourlies + 1 capture day.
 * Total de vertices ~= 2000. Trivial para GL. El builder corre en CPU una sola vez
 * por sesión 3D, en background. Los FloatBuffers resultantes se suben a GL como VBOs
 * (esa parte va en el paso 2).
 */

/**
 * Una path del ábaco partida en sub-strips (uno por discontinuidad).
 * Cada FloatBuffer está listo para glDrawArrays(GL_LINE_STRIP, 0, vertexCount)
 * con stride=0 y 3 floats por vertex (x, y, z) en ENU.
 */
data class SolarPathMesh3D(
    val kind: SolarPath.Kind,
    val monthIndex: Int?,
    val solarHour: Int?,
    val strips: List<FloatBuffer>,
    val vertexCounts: List<Int>
)

/** Posición del sol en el momento de la captura, lista para dibujar como punto. */
data class SunMarkerMesh3D(
    val position: FloatArray,   // [x, y, z] en ENU
    val buffer: FloatBuffer     // 3 floats, listo para glDrawArrays(GL_POINTS, 0, 1)
)

/** Conjunto completo del ábaco 3D. */
data class SolarChartMesh3D(
    val dailyPaths: List<SolarPathMesh3D>,
    val hourlyPaths: List<SolarPathMesh3D>,
    val captureDayPath: SolarPathMesh3D?,
    val sunMarker: SunMarkerMesh3D?
) {
    companion object {
        /** Radio sobre el cual se proyectan los puntos. Igual al de la esfera del skybox. */
        private const val RADIUS = 1.0f

        /** Umbral para detectar wrap-around longitudinal entre puntos consecutivos. */
        private const val WRAP_THRESHOLD_DEG = 90.0

        /**
         * Convierte un SolarChart a geometría 3D sobre la esfera de radio 1.
         * Ejecutar en background — hace ~2000 trig ops pero no toca GL.
         */
        fun build(chart: SolarChart): SolarChartMesh3D {
            val daily = chart.dailyPaths.map { buildPathMesh(it) }
            val hourly = chart.hourlyPaths.map { buildPathMesh(it) }
            val capture = chart.captureDayPath?.let { buildPathMesh(it) }
            val sun = chart.sunAtCapture
                ?.takeIf { !it.belowHorizon && it.altitudeDeg >= 0.0 }
                ?.let { buildSunMarker(it) }
            return SolarChartMesh3D(
                dailyPaths = daily,
                hourlyPaths = hourly,
                captureDayPath = capture,
                sunMarker = sun
            )
        }

        private fun buildPathMesh(path: SolarPath): SolarPathMesh3D {
            val strips = mutableListOf<MutableList<Float>>()
            var current: MutableList<Float>? = null
            var prevAz: Double? = null

            for (pos in path.points) {
                // Corte (v1): sub-horizonte se descarta. Strip queda cerrado.
                if (pos.belowHorizon || pos.altitudeDeg < 0.0) {
                    current = null
                    prevAz = null
                    continue
                }

                // Corte por discontinuidad angular (eg. analemma cruzando cenit).
                val wrap = prevAz != null &&
                        abs(shortestAngleDelta(prevAz!!, pos.azimuthDeg)) > WRAP_THRESHOLD_DEG

                if (current == null || wrap) {
                    current = mutableListOf()
                    strips.add(current)
                }

                // Proyección ENU sobre esfera radio R.
                // Misma fórmula que AtlasProjector.worldDirectionRad y SphereMesh:
                //   x = R·cos(alt)·sin(az)   (Este)
                //   y = R·cos(alt)·cos(az)   (Norte)
                //   z = R·sin(alt)           (Up)
                val azRad = Math.toRadians(pos.azimuthDeg)
                val altRad = Math.toRadians(pos.altitudeDeg)
                val cosAlt = cos(altRad).toFloat()
                val sinAlt = sin(altRad).toFloat()
                val sinAz = sin(azRad).toFloat()
                val cosAz = cos(azRad).toFloat()

                current!!.add(RADIUS * cosAlt * sinAz)
                current!!.add(RADIUS * cosAlt * cosAz)
                current!!.add(RADIUS * sinAlt)

                prevAz = pos.azimuthDeg
            }

            // GL_LINE_STRIP requiere al menos 2 vertices = 6 floats.
            // Strips de 1 vertex (eg. un solo punto válido entre dos cortes) se descartan.
            val validStrips = strips.filter { it.size >= 6 }

            val buffers = validStrips.map { floatsToBuffer(it.toFloatArray()) }
            val counts = validStrips.map { it.size / 3 }

            return SolarPathMesh3D(
                kind = path.kind,
                monthIndex = path.monthIndex,
                solarHour = path.solarHour,
                strips = buffers,
                vertexCounts = counts
            )
        }

        private fun buildSunMarker(pos: SolarPosition): SunMarkerMesh3D {
            val azRad = Math.toRadians(pos.azimuthDeg)
            val altRad = Math.toRadians(pos.altitudeDeg)
            val cosAlt = cos(altRad).toFloat()
            val xyz = floatArrayOf(
                RADIUS * cosAlt * sin(azRad).toFloat(),
                RADIUS * cosAlt * cos(azRad).toFloat(),
                RADIUS * sin(altRad).toFloat()
            )
            return SunMarkerMesh3D(position = xyz, buffer = floatsToBuffer(xyz))
        }

        private fun floatsToBuffer(data: FloatArray): FloatBuffer =
            ByteBuffer.allocateDirect(data.size * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()
                .apply { put(data); position(0) }

        private fun shortestAngleDelta(a: Double, b: Double): Double {
            var d = (b - a) % 360.0
            if (d > 180.0) d -= 360.0
            if (d < -180.0) d += 360.0
            return d
        }
    }
}