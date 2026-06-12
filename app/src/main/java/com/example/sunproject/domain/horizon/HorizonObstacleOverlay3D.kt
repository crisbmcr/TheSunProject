package com.example.sunproject.domain.render3d.horizon

import android.opengl.GLES20
import android.util.Log
import com.example.sunproject.domain.horizon.HorizonProfile
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.cos
import kotlin.math.sin

private const val TAG = "HorizonObstacleOverlay3D"

/**
 * Overlay GL del perfil de obstrucción del horizonte superpuesto al skybox.
 *
 * Dibuja dos capas a partir de un HorizonProfile (función az -> alt_min):
 *   1. BANDA: GL_TRIANGLE_STRIP entre alt=0 (horizonte) y alt=alt_min por
 *      cada azimut. Rellena la zona obstruida con color semitransparente.
 *      Es el análogo 3D del sombreado del atlas 2D.
 *   2. CRESTA: GL_LINE_STRIP cerrado recorriendo (az, alt_min). Marca el
 *      perfil exacto con color nítido encima de la banda.
 *
 * Convención de coordenadas: idéntica a SphereMesh y SolarChartMesh3D.
 *   x = cosAlt * sin(azRad)   (Este)
 *   y = cosAlt * cos(azRad)   (Norte)
 *   z = sinAlt                (Up)
 * con azRad el azimut REAL del bucket (0=N, CW), radio = 1.
 *
 * Lifecycle alineado con WorldGridOverlay3D:
 *  - Constructor: ningún recurso GL.
 *  - onSurfaceCreated(): resetea handles (Surface recreada -> handles inválidos).
 *  - setProfile(): inyecta datos CPU desde cualquier thread; upload GL lazy en draw().
 *  - draw(mvp): glUseProgram propio, depth OFF + blend ON, dibuja, restaura.
 *
 * Si nunca se setea un profile, draw() es no-op.
 */
class HorizonObstacleOverlay3D {

    companion object {
        // Resolución angular del muestreo. 1° = un sample por grado.
        private const val AZIMUTH_STEP_DEG = 1.0f

        // Radio levemente < 1 para que la banda quede justo "sobre" la textura
        // de la esfera (radio 1) sin z-fighting. Como depth test va OFF, el
        // orden de draw decide; este offset es defensivo y cosmético.
        private const val RADIUS = 0.999f

        // Banda obstruida: rojo cálido, alpha medio para ver la textura debajo.
        private val BAND_COLOR_RGBA = floatArrayOf(0.86f, 0.20f, 0.18f, 0.35f)

        // Cresta del perfil: rojo nítido, opaco.
        private val CREST_COLOR_RGBA = floatArrayOf(0.96f, 0.32f, 0.28f, 0.95f)

        private const val CREST_LINE_WIDTH = 3.0f
    }

    // ---- Estado CPU ------------------------------------------------------
    @Volatile
    private var pendingProfile: HorizonProfile? = null

    private var bandBuffer: FloatBuffer? = null
    private var bandVertexCount = 0
    private var crestBuffer: FloatBuffer? = null
    private var crestVertexCount = 0

    // ---- Estado GL -------------------------------------------------------
    private var program = 0
    private var aPositionLoc = -1
    private var uMvpMatrixLoc = -1
    private var uColorLoc = -1
    private var glResourcesUploaded = false

    // ---- Shaders (color uniforme, igual molde que WorldGridOverlay3D) -----

    private val vertexShaderSource = """
        attribute vec3 aPosition;
        uniform mat4 uMvpMatrix;
        void main() {
            gl_Position = uMvpMatrix * vec4(aPosition, 1.0);
        }
    """.trimIndent()

    private val fragmentShaderSource = """
        precision mediump float;
        uniform vec4 uColor;
        void main() {
            gl_FragColor = uColor;
        }
    """.trimIndent()

    // ---- API pública ------------------------------------------------------

    /**
     * Inyecta el perfil de horizonte. Puede llamarse desde cualquier thread.
     * El cálculo de vértices y upload GL ocurren en el render thread durante
     * el siguiente draw(). Pasar null elimina el overlay.
     */
    fun setProfile(profile: HorizonProfile?) {
        pendingProfile = profile
        glResourcesUploaded = false
        Log.i(TAG, "setProfile — buckets=${profile?.azimuthBuckets ?: 0}")
    }

    /**
     * PanoramaRenderer lo llama desde su onSurfaceCreated. Resetea estado GL:
     * los handles viejos son inválidos en el nuevo contexto.
     */
    fun onSurfaceCreated() {
        program = 0
        aPositionLoc = -1
        uMvpMatrixLoc = -1
        uColorLoc = -1
        glResourcesUploaded = false
        Log.i(TAG, "onSurfaceCreated — estado GL reseteado")
    }

    /**
     * Dibuja banda + cresta. Asume que el caller ya seteó viewport y
     * projection. Modifica depth/blend y restaura al estado que usa
     * PanoramaRenderer entre frames (depth ON, blend OFF).
     */
    fun draw(mvpMatrix: FloatArray) {
        if (program == 0) program = buildProgram(vertexShaderSource, fragmentShaderSource)

        if (!glResourcesUploaded) {
            val profile = pendingProfile
            if (profile != null) {
                buildMeshes(profile)
                cacheLocations()
                glResourcesUploaded = true
            }
        }

        val band = bandBuffer
        val crest = crestBuffer
        if (band == null || crest == null) return

        // State común del overlay.
        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(uMvpMatrixLoc, 1, false, mvpMatrix, 0)
        GLES20.glEnableVertexAttribArray(aPositionLoc)

        // 1) Banda obstruida (triángulos, alpha medio).
        band.position(0)
        GLES20.glVertexAttribPointer(aPositionLoc, 3, GLES20.GL_FLOAT, false, 0, band)
        GLES20.glUniform4fv(uColorLoc, 1, BAND_COLOR_RGBA, 0)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, bandVertexCount)

        // 2) Cresta del perfil (línea nítida encima).
        GLES20.glLineWidth(CREST_LINE_WIDTH)  // capped a 1.0 en muchos drivers GLES2
        crest.position(0)
        GLES20.glVertexAttribPointer(aPositionLoc, 3, GLES20.GL_FLOAT, false, 0, crest)
        GLES20.glUniform4fv(uColorLoc, 1, CREST_COLOR_RGBA, 0)
        GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, 0, crestVertexCount)

        GLES20.glDisableVertexAttribArray(aPositionLoc)

        // Restaurar al state que asume PanoramaRenderer entre frames.
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glDisable(GLES20.GL_BLEND)
    }

    fun release() {
        if (program != 0) GLES20.glDeleteProgram(program)
        onSurfaceCreated()
    }

    // ---- Construcción de mallas ------------------------------------------

    /**
     * Genera los buffers CPU de banda y cresta a partir del perfil.
     * Corre en el render thread (lo llama draw()).
     */
    private fun buildMeshes(profile: HorizonProfile) {
        val steps = (360f / AZIMUTH_STEP_DEG).toInt()  // 360 samples
        val ringCount = steps + 1                       // +1 para cerrar el loop

        // --- Banda: TRIANGLE_STRIP, 2 vértices por azimut (base + cima) ---
        val bandFloats = FloatArray(ringCount * 2 * 3)
        var bi = 0
        // --- Cresta: LINE_STRIP, 1 vértice por azimut ---
        val crestFloats = FloatArray(ringCount * 3)
        var ci = 0

        for (i in 0..steps) {
            val azDeg = (i * AZIMUTH_STEP_DEG) % 360f
            val altDeg = profile.altAtAzimuthDeg(azDeg).coerceIn(0f, 90f)

            val azRad = Math.toRadians(azDeg.toDouble())
            val sinAz = sin(azRad).toFloat()
            val cosAz = cos(azRad).toFloat()

            // Base en el horizonte (alt=0): cosAlt=1, sinAlt=0.
            val bx = RADIUS * sinAz
            val by = RADIUS * cosAz
            val bz = 0f

            // Cima en alt_min.
            val altRad = Math.toRadians(altDeg.toDouble())
            val cosAlt = cos(altRad).toFloat()
            val sinAlt = sin(altRad).toFloat()
            val tx = RADIUS * cosAlt * sinAz
            val ty = RADIUS * cosAlt * cosAz
            val tz = RADIUS * sinAlt

            // Strip: base, cima, base, cima, ...
            bandFloats[bi++] = bx; bandFloats[bi++] = by; bandFloats[bi++] = bz
            bandFloats[bi++] = tx; bandFloats[bi++] = ty; bandFloats[bi++] = tz

            // Cresta: solo la cima.
            crestFloats[ci++] = tx; crestFloats[ci++] = ty; crestFloats[ci++] = tz
        }

        bandBuffer = toBuffer(bandFloats)
        bandVertexCount = ringCount * 2
        crestBuffer = toBuffer(crestFloats)
        crestVertexCount = ringCount

        Log.i(TAG, "Mallas construidas — banda=$bandVertexCount verts, cresta=$crestVertexCount verts")
    }

    private fun toBuffer(data: FloatArray): FloatBuffer =
        ByteBuffer.allocateDirect(data.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply { put(data); position(0) }

    private fun cacheLocations() {
        aPositionLoc = GLES20.glGetAttribLocation(program, "aPosition")
        uMvpMatrixLoc = GLES20.glGetUniformLocation(program, "uMvpMatrix")
        uColorLoc = GLES20.glGetUniformLocation(program, "uColor")
    }

    private fun buildProgram(vsSrc: String, fsSrc: String): Int {
        val vs = compileShader(GLES20.GL_VERTEX_SHADER, vsSrc)
        val fs = compileShader(GLES20.GL_FRAGMENT_SHADER, fsSrc)
        val prog = GLES20.glCreateProgram()
        GLES20.glAttachShader(prog, vs)
        GLES20.glAttachShader(prog, fs)
        GLES20.glLinkProgram(prog)
        val status = IntArray(1)
        GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetProgramInfoLog(prog)
            GLES20.glDeleteProgram(prog)
            throw RuntimeException("HorizonObstacleOverlay3D link failed: $log")
        }
        return prog
    }

    private fun compileShader(type: Int, src: String): Int {
        val sh = GLES20.glCreateShader(type)
        GLES20.glShaderSource(sh, src)
        GLES20.glCompileShader(sh)
        val status = IntArray(1)
        GLES20.glGetShaderiv(sh, GLES20.GL_COMPILE_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetShaderInfoLog(sh)
            GLES20.glDeleteShader(sh)
            throw RuntimeException("HorizonObstacleOverlay3D compile failed: $log")
        }
        return sh
    }
}