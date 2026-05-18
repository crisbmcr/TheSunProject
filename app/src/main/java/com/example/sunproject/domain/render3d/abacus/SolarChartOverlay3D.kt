package com.example.sunproject.domain.render3d.abacus

import android.graphics.Color
import android.opengl.GLES20
import android.util.Log
import com.example.sunproject.domain.solar.SolarChartPalette
import java.nio.FloatBuffer

private const val TAG = "SolarChartOverlay3D"

/**
 * Renderer GL del ábaco solar superpuesto al skybox panorámico.
 *
 * Ciclo de vida (alineado con PanoramaRenderer):
 *  1. Constructor: ningún recurso GL — seguro de instanciar antes del contexto GL.
 *  2. onSurfaceCreated(): se llama desde PanoramaRenderer.onSurfaceCreated.
 *     Resetea flags porque la Surface se puede recrear (volver desde background)
 *     y todos los handles GL viejos quedan invalidados.
 *  3. setMesh(): inyecta datos CPU desde cualquier thread. El upload GL es lazy
 *     y ocurre en el primer draw() después del set.
 *  4. draw(mvp): se llama desde PanoramaRenderer.onDrawFrame, DESPUÉS del skybox.
 *     Hace su propio glUseProgram, configura depth/blend, dibuja y restaura.
 *
 * Decisiones de render:
 *  - Depth test OFF durante el ábaco. El control de "delante de la esfera" es
 *    el orden de las drawcalls (esfera primero, ábaco después), no el radio.
 *  - Blend ON con SRC_ALPHA, ONE_MINUS_SRC_ALPHA — necesario para HOUR_LINE_COLOR
 *    que viene con alpha 140/255.
 *  - Un único shader program sirve líneas y puntos vía uniform uIsPoint.
 *  - Líneas: glDrawArrays(GL_LINE_STRIP) por sub-strip. Hay ~25 strips totales,
 *    una drawcall cada uno. Negligible overhead.
 *  - Sol: GL_POINTS de 1 vertex con gl_PointSize y discard circular en el frag
 *    para hacer círculo con anillo. Más simple que billboard quad.
 *
 * Caveat conocido: glLineWidth está cappeado a 1.0f por la spec GLES 2.0; muchos
 * drivers Android lo respetan literalmente. Si las líneas se ven flacas en
 * pantalla grande, Refinement 1 = reemplazar por triangle strips perpendiculares
 * al rayo cámara (tubos).
 */
class SolarChartOverlay3D {

    // ---- Constantes de estilo (mi criterio para v1) ----------------------
    // Si glLineWidth se cappea a 1, estos valores no se notan. Quedan declarados
    // igual para que el upgrade a tubos en Refinement 1 herede el mismo orden.
    companion object {
        private const val LINE_WIDTH_DAILY = 3.0f      // 2.4f en 2D, un toque más en 3D
        private const val LINE_WIDTH_HOURLY = 2.0f     // 1.4f en 2D
        private const val LINE_WIDTH_CAPTURE = 5.0f    // 4.5f en 2D
        private const val SUN_POINT_SIZE_PX = 28.0f
    }

    // ---- Estado CPU ------------------------------------------------------
    @Volatile
    private var pendingMesh: SolarChartMesh3D? = null

    private var glResourcesUploaded = false

    // ---- Estado GL -------------------------------------------------------
    private var program = 0
    private var aPositionLoc = -1
    private var uMvpMatrixLoc = -1
    private var uColorLoc = -1
    private var uIsPointLoc = -1
    private var uPointSizeLoc = -1

    private val lineDrawables = mutableListOf<LineDrawable>()
    private var sunDrawable: PointDrawable? = null

    private data class LineDrawable(
        val vboId: Int,
        val vertexCount: Int,
        val color: FloatArray,
        val lineWidth: Float
    )

    private data class PointDrawable(
        val vboId: Int,
        val color: FloatArray,
        val pointSize: Float
    )

    // ---- Shaders ---------------------------------------------------------

    private val vertexShaderSource = """
        attribute vec3 aPosition;
        uniform mat4 uMvpMatrix;
        uniform float uPointSize;
        void main() {
            gl_Position = uMvpMatrix * vec4(aPosition, 1.0);
            gl_PointSize = uPointSize;
        }
    """.trimIndent()

    // El fragment shader bifurca según uIsPoint:
    //  - Líneas: color uniforme directo.
    //  - Puntos: discard si está fuera del círculo de radio 0.5 (transparente).
    //    Anillo entre 0.4 y 0.5 con SUN_RING_COLOR hardcodeado (es la única vez
    //    que aparece, no vale la pena uniformizarlo).
    //    Centro con uColor (= SUN_FILL_COLOR).
    private val fragmentShaderSource = """
        precision mediump float;
        uniform vec4 uColor;
        uniform int uIsPoint;
        void main() {
            if (uIsPoint == 1) {
                vec2 c = gl_PointCoord - vec2(0.5);
                float r = length(c);
                if (r > 0.5) discard;
                if (r > 0.4) {
                    // SolarChartPalette.SUN_RING_COLOR = rgb(133, 79, 11)
                    gl_FragColor = vec4(0.5216, 0.3098, 0.0431, 1.0);
                } else {
                    gl_FragColor = uColor;
                }
            } else {
                gl_FragColor = uColor;
            }
        }
    """.trimIndent()

    // ---- API pública -----------------------------------------------------

    /**
     * Inyecta un mesh nuevo. Puede llamarse desde cualquier thread. El upload
     * GL ocurre en el render thread durante el siguiente draw().
     */
    fun setMesh(mesh: SolarChartMesh3D) {
        pendingMesh = mesh
        glResourcesUploaded = false
        Log.i(
            TAG,
            "setMesh — daily=${mesh.dailyPaths.size} hourly=${mesh.hourlyPaths.size} " +
                    "capture=${mesh.captureDayPath != null} sun=${mesh.sunMarker != null}"
        )
    }

    /**
     * Avisar al overlay que la Surface se (re)creó. PanoramaRenderer lo llama
     * desde su onSurfaceCreated. Resetea todo el estado GL — los handles viejos
     * son inválidos en el nuevo contexto.
     */
    fun onSurfaceCreated() {
        program = 0
        aPositionLoc = -1
        uMvpMatrixLoc = -1
        uColorLoc = -1
        uIsPointLoc = -1
        uPointSizeLoc = -1
        lineDrawables.clear()
        sunDrawable = null
        glResourcesUploaded = false
        Log.i(TAG, "onSurfaceCreated — estado GL reseteado")
    }

    /**
     * Dibuja el ábaco. Asume que el caller ya seteó viewport y projection.
     * Modifica depth/blend internamente y los restaura al estado que usa
     * PanoramaRenderer entre frames (depth ON, blend OFF).
     */
    fun draw(mvpMatrix: FloatArray) {
        // Compilar program si no hay (primer draw, o post-surfaceCreated).
        if (program == 0) {
            compileProgram()
        }

        // Upload de VBOs si hay mesh pendiente.
        val mesh = pendingMesh
        if (!glResourcesUploaded && mesh != null) {
            releaseDrawables()
            uploadMesh(mesh)
            glResourcesUploaded = true
        }

        if (lineDrawables.isEmpty() && sunDrawable == null) return

        renderDrawables(mvpMatrix)
    }

    /**
     * Libera todos los recursos GL. Llamar si se quiere destruir el overlay
     * explícitamente — opcional, ya que destruir el contexto GL libera todo.
     */
    fun release() {
        releaseDrawables()
        if (program != 0) {
            GLES20.glDeleteProgram(program)
            program = 0
        }
    }

    // ---- Compilación y upload -------------------------------------------

    private fun compileProgram() {
        val vs = compileShader(GLES20.GL_VERTEX_SHADER, vertexShaderSource)
        val fs = compileShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderSource)
        val prog = GLES20.glCreateProgram()
        GLES20.glAttachShader(prog, vs)
        GLES20.glAttachShader(prog, fs)
        GLES20.glLinkProgram(prog)

        val status = IntArray(1)
        GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetProgramInfoLog(prog)
            GLES20.glDeleteProgram(prog)
            throw RuntimeException("SolarChartOverlay3D link failed: $log")
        }

        program = prog
        aPositionLoc = GLES20.glGetAttribLocation(prog, "aPosition")
        uMvpMatrixLoc = GLES20.glGetUniformLocation(prog, "uMvpMatrix")
        uColorLoc = GLES20.glGetUniformLocation(prog, "uColor")
        uIsPointLoc = GLES20.glGetUniformLocation(prog, "uIsPoint")
        uPointSizeLoc = GLES20.glGetUniformLocation(prog, "uPointSize")

        Log.i(TAG, "program compilado — id=$program")
    }

    private fun compileShader(type: Int, source: String): Int {
        val shader = GLES20.glCreateShader(type)
        GLES20.glShaderSource(shader, source)
        GLES20.glCompileShader(shader)
        val status = IntArray(1)
        GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetShaderInfoLog(shader)
            GLES20.glDeleteShader(shader)
            throw RuntimeException("SolarChartOverlay3D compile failed: $log")
        }
        return shader
    }

    private fun uploadMesh(mesh: SolarChartMesh3D) {
        // Diarias mensuales — color por mes.
        for (path in mesh.dailyPaths) {
            val color = path.monthIndex
                ?.let { colorToFloatArray(SolarChartPalette.colorForMonth(it)) }
                ?: floatArrayOf(0.5f, 0.5f, 0.5f, 1f)
            path.strips.forEachIndexed { i, buf ->
                val vbo = createVbo(buf)
                lineDrawables.add(
                    LineDrawable(vbo, path.vertexCounts[i], color, LINE_WIDTH_DAILY)
                )
            }
        }

        // Horarias (analemmas) — color uniforme.
        val hourColor = colorToFloatArray(SolarChartPalette.HOUR_LINE_COLOR)
        for (path in mesh.hourlyPaths) {
            path.strips.forEachIndexed { i, buf ->
                val vbo = createVbo(buf)
                lineDrawables.add(
                    LineDrawable(vbo, path.vertexCounts[i], hourColor, LINE_WIDTH_HOURLY)
                )
            }
        }

        // Día de la captura — blanco grueso.
        val captureColor = colorToFloatArray(SolarChartPalette.CAPTURE_DAY_COLOR)
        mesh.captureDayPath?.strips?.forEachIndexed { i, buf ->
            val vbo = createVbo(buf)
            lineDrawables.add(
                LineDrawable(vbo, mesh.captureDayPath.vertexCounts[i], captureColor, LINE_WIDTH_CAPTURE)
            )
        }

        // Sol.
        mesh.sunMarker?.let { marker ->
            val vbo = createVbo(marker.buffer)
            val fillColor = colorToFloatArray(SolarChartPalette.SUN_FILL_COLOR)
            sunDrawable = PointDrawable(vbo, fillColor, SUN_POINT_SIZE_PX)
        }

        Log.i(TAG, "uploadMesh — ${lineDrawables.size} VBOs de línea + ${if (sunDrawable != null) 1 else 0} marker")
    }

    private fun createVbo(data: FloatBuffer): Int {
        data.position(0)
        val ids = IntArray(1)
        GLES20.glGenBuffers(1, ids, 0)
        val vbo = ids[0]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        GLES20.glBufferData(
            GLES20.GL_ARRAY_BUFFER,
            data.capacity() * 4,   // bytes
            data,
            GLES20.GL_STATIC_DRAW
        )
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        return vbo
    }

    private fun releaseDrawables() {
        val allVbos = mutableListOf<Int>()
        lineDrawables.forEach { allVbos.add(it.vboId) }
        sunDrawable?.let { allVbos.add(it.vboId) }
        if (allVbos.isNotEmpty()) {
            GLES20.glDeleteBuffers(allVbos.size, allVbos.toIntArray(), 0)
        }
        lineDrawables.clear()
        sunDrawable = null
    }

    // ---- Render ----------------------------------------------------------

    private fun renderDrawables(mvpMatrix: FloatArray) {
        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(uMvpMatrixLoc, 1, false, mvpMatrix, 0)

        // Depth OFF: el ábaco se ve siempre por delante de la esfera.
        // Blend ON: respetamos alpha de HOUR_LINE_COLOR.
        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        GLES20.glEnableVertexAttribArray(aPositionLoc)

        // ---- Líneas ----
        GLES20.glUniform1i(uIsPointLoc, 0)
        GLES20.glUniform1f(uPointSizeLoc, 1.0f) // ignorado para líneas pero seteo por higiene

        for (drawable in lineDrawables) {
            GLES20.glLineWidth(drawable.lineWidth)
            GLES20.glUniform4fv(uColorLoc, 1, drawable.color, 0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, drawable.vboId)
            GLES20.glVertexAttribPointer(aPositionLoc, 3, GLES20.GL_FLOAT, false, 0, 0)
            GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, 0, drawable.vertexCount)
        }

        // ---- Marker del sol ----
        sunDrawable?.let { sun ->
            GLES20.glUniform1i(uIsPointLoc, 1)
            GLES20.glUniform1f(uPointSizeLoc, sun.pointSize)
            GLES20.glUniform4fv(uColorLoc, 1, sun.color, 0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, sun.vboId)
            GLES20.glVertexAttribPointer(aPositionLoc, 3, GLES20.GL_FLOAT, false, 0, 0)
            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, 1)
        }

        // ---- Restaurar estado al que asume PanoramaRenderer ----
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        GLES20.glDisableVertexAttribArray(aPositionLoc)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glDisable(GLES20.GL_BLEND)
    }

    // ---- Helpers ---------------------------------------------------------

    private fun colorToFloatArray(color: Int): FloatArray = floatArrayOf(
        Color.red(color) / 255f,
        Color.green(color) / 255f,
        Color.blue(color) / 255f,
        Color.alpha(color) / 255f
    )
}