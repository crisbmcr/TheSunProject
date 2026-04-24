package com.example.sunproject.domain.render3d

import android.graphics.Bitmap
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.GLUtils
import android.opengl.Matrix
import android.util.Log
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

private const val TAG = "PanoramaRenderer"

/**
 * Renderer del skybox esférico con atlas equirectangular.
 *
 * Pipeline v1 (esta tanda):
 *  - Skybox solo. Cámara estática mirando al Norte (+Y ENU) con pitch 0, FOV 75°.
 *  - Sin giroscopio, sin gestos, sin ábaco. Esos vienen en tandas siguientes.
 *
 * El Bitmap del atlas se sube como textura GL_TEXTURE_2D en onSurfaceCreated.
 * Importante: el Bitmap debe estar en config ARGB_8888 y no reciclado al momento del upload.
 */
class PanoramaRenderer(
    private val atlasBitmap: Bitmap
) : GLSurfaceView.Renderer {

    private val sphere = SphereMesh(
        slices = 60,
        stacks = 30,
        atlasMinAltitudeDeg = 0f,      // horizonte (fila inferior del atlas)
        atlasMaxAltitudeDeg = 90f      // cenit (fila superior del atlas)
    )

    private var program = 0
    private var aPositionLoc = -1
    private var aTexCoordLoc = -1
    private var uMvpMatrixLoc = -1
    private var uTextureLoc = -1

    private var textureId = 0

    private val projectionMatrix = FloatArray(16)
    private val viewMatrix = FloatArray(16)
    private val modelMatrix = FloatArray(16)
    private val mvpMatrix = FloatArray(16)
    private val tempMatrix = FloatArray(16)

    // Matrix externa (inyectada desde un CameraController). Si se setea, sobreescribe
// el setLookAtM estático del onDrawFrame. Acceso @Volatile porque se escribe desde
// un thread del sensor y se lee desde el render thread.
    @Volatile
    private var externalViewMatrix: FloatArray? = null

    private var viewportWidth = 1
    private var viewportHeight = 1
    private var fovDegrees = 75f

    // Vertex shader: transforma vértice por MVP y pasa UV al fragment.
    private val vertexShaderSource = """
        uniform mat4 uMvpMatrix;
        attribute vec4 aPosition;
        attribute vec2 aTexCoord;
        varying vec2 vTexCoord;
        void main() {
            gl_Position = uMvpMatrix * aPosition;
            vTexCoord = aTexCoord;
        }
    """.trimIndent()

    // Fragment shader: samplea el atlas directamente.
    private val fragmentShaderSource = """
        precision mediump float;
        varying vec2 vTexCoord;
        uniform sampler2D uTexture;
        void main() {
            gl_FragColor = texture2D(uTexture, vTexCoord);
        }
    """.trimIndent()

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)

        // Para skybox visto desde adentro: el winding del SphereMesh está armado
        // para que los triángulos sean CCW desde el centro. Culleamos los que son CW
        // (las "caras externas" de la esfera).
        GLES20.glEnable(GLES20.GL_CULL_FACE)
        GLES20.glCullFace(GLES20.GL_BACK)
        GLES20.glFrontFace(GLES20.GL_CCW)

        program = buildProgram(vertexShaderSource, fragmentShaderSource)
        aPositionLoc = GLES20.glGetAttribLocation(program, "aPosition")
        aTexCoordLoc = GLES20.glGetAttribLocation(program, "aTexCoord")
        uMvpMatrixLoc = GLES20.glGetUniformLocation(program, "uMvpMatrix")
        uTextureLoc = GLES20.glGetUniformLocation(program, "uTexture")

        textureId = uploadTexture(atlasBitmap)

        Matrix.setIdentityM(modelMatrix, 0)

        Log.i(TAG, "onSurfaceCreated OK — program=$program textureId=$textureId " +
                "atlas=${atlasBitmap.width}x${atlasBitmap.height}")
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewportWidth = width
        viewportHeight = height
        GLES20.glViewport(0, 0, width, height)
        updateProjection()
        Log.i(TAG, "onSurfaceChanged ${width}x${height} fov=$fovDegrees")
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        // Si hay un controller que inyectó una viewMatrix, la usamos. Si no, fallback
        // a cámara estática mirando al Norte (como v1).
        val currentView = externalViewMatrix
        if (currentView != null) {
            // Copiamos a viewMatrix local porque el controller puede seguir escribiendo
            // en paralelo. Una copia de 16 floats es trivial.
            System.arraycopy(currentView, 0, viewMatrix, 0, 16)
        } else {
            Matrix.setLookAtM(viewMatrix, 0,
                0f, 0f, 0f,     // eye
                0f, 1f, 0f,     // center (Norte)
                0f, 0f, 1f      // up (cenit)
            )
        }

        // MVP = projection * view * model
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(uMvpMatrixLoc, 1, false, mvpMatrix, 0)

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId)
        GLES20.glUniform1i(uTextureLoc, 0)

        GLES20.glEnableVertexAttribArray(aPositionLoc)
        GLES20.glVertexAttribPointer(aPositionLoc, 3, GLES20.GL_FLOAT, false, 0, sphere.vertexBuffer)

        GLES20.glEnableVertexAttribArray(aTexCoordLoc)
        GLES20.glVertexAttribPointer(aTexCoordLoc, 2, GLES20.GL_FLOAT, false, 0, sphere.texCoordBuffer)

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, sphere.indexCount,
            GLES20.GL_UNSIGNED_SHORT, sphere.indexBuffer)

        GLES20.glDisableVertexAttribArray(aPositionLoc)
        GLES20.glDisableVertexAttribArray(aTexCoordLoc)

        /**
         * Inyecta una viewMatrix externa (típicamente desde un CameraController).
         * Pasar null para volver al fallback estático.
         */
        fun setExternalViewMatrix(matrix: FloatArray?) {
            externalViewMatrix = matrix
        }
    }

    private fun updateProjection() {
        val aspect = viewportWidth.toFloat() / viewportHeight.toFloat()
        val fovRad = Math.toRadians(fovDegrees.toDouble())
        val top = (Math.tan(fovRad / 2.0) * 0.1).toFloat()
        val right = top * aspect
        Matrix.frustumM(projectionMatrix, 0, -right, right, -top, top, 0.1f, 10f)
    }

    private fun uploadTexture(bitmap: Bitmap): Int {
        val ids = IntArray(1)
        GLES20.glGenTextures(1, ids, 0)
        val id = ids[0]
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, id)

        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D,
            GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D,
            GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D,
            GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D,
            GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)

        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0)
        GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D)

        return id
    }

    private fun buildProgram(vsSource: String, fsSource: String): Int {
        val vs = compileShader(GLES20.GL_VERTEX_SHADER, vsSource)
        val fs = compileShader(GLES20.GL_FRAGMENT_SHADER, fsSource)
        val prog = GLES20.glCreateProgram()
        GLES20.glAttachShader(prog, vs)
        GLES20.glAttachShader(prog, fs)
        GLES20.glLinkProgram(prog)

        val status = IntArray(1)
        GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetProgramInfoLog(prog)
            GLES20.glDeleteProgram(prog)
            throw RuntimeException("Link program failed: $log")
        }
        return prog
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
            throw RuntimeException("Compile shader failed: $log")
        }
        return shader
    }
}