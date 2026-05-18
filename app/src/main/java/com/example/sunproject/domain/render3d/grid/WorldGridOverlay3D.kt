package com.example.sunproject.domain.render3d.grid

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.opengl.GLES20
import android.opengl.GLUtils
import android.util.Log
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.ShortBuffer
import kotlin.math.cos
import kotlin.math.sin

private const val TAG = "WorldGridOverlay3D"

/**
 * Overlay de referencias del mundo superpuestas al skybox: línea de horizonte
 * (altitud 0°) y etiquetas de azimut cada 10°.
 *
 * Arquitectura:
 *  - DOS programs GL — uno simple para la línea de horizonte (similar al de
 *    SolarChartOverlay3D pero sin lógica de puntos), y otro con sampler2D para
 *    los quads con textura de etiquetas.
 *  - UN bitmap atlas precomputado con las 36 etiquetas dispuestas en una grilla
 *    6x6 de slots 192x64. Se genera en CPU una sola vez al primer draw.
 *  - 36 quads billboard precomputados en CPU con `up=(0,0,1)`, `right` tangencial
 *    al círculo horizontal. Como el observador está fijo en el origen y el quad
 *    rota con el mundo (no con la cámara), no hace falta actualizar por frame.
 *
 * Lifecycle igual que SolarChartOverlay3D — onSurfaceCreated resetea state GL,
 * draw() es lazy con upload en primer call.
 */
class WorldGridOverlay3D {

    companion object {
        // ---- Línea de horizonte ----
        private const val HORIZON_SEGMENTS = 72  // resolución del círculo
        private const val HORIZON_LINE_WIDTH = 2.5f
        // Blanco con alpha ~70% para que sea referencia pero no tape las curvas.
        private val HORIZON_COLOR_RGBA = floatArrayOf(1f, 1f, 1f, 0.63f)

        // ---- Etiquetas de azimut ----
        private const val LABEL_INTERVAL_DEG = 10
        private const val LABEL_COUNT = 36  // 0°, 10°, ..., 350°
        private const val LABEL_ALTITUDE_DEG = 2.0  // apenas arriba del horizonte
        private const val LABEL_QUAD_HEIGHT = 0.05f  // altura del quad en mundo (radio=1)
        private const val LABEL_QUAD_WIDTH = 0.15f   // ancho (proporción 3:1 con slot)

        // Atlas de texturas: 6 columnas × 6 filas, cada slot 192x64.
        private const val ATLAS_COLS = 6
        private const val ATLAS_ROWS = 6
        private const val SLOT_WIDTH_PX = 192
        private const val SLOT_HEIGHT_PX = 64
        private const val ATLAS_WIDTH_PX = SLOT_WIDTH_PX * ATLAS_COLS   // 1152
        private const val ATLAS_HEIGHT_PX = SLOT_HEIGHT_PX * ATLAS_ROWS // 384

        private const val FLOATS_PER_VERTEX = 5  // x, y, z, u, v
        private const val BYTES_PER_VERTEX = FLOATS_PER_VERTEX * 4
    }

    // ---- Estado GL: program para líneas ----
    private var lineProgram = 0
    private var lineAPositionLoc = -1
    private var lineUMvpMatrixLoc = -1
    private var lineUColorLoc = -1

    // ---- Estado GL: program para quads texturados ----
    private var labelProgram = 0
    private var labelAPositionLoc = -1
    private var labelATexCoordLoc = -1
    private var labelUMvpMatrixLoc = -1
    private var labelUTextureLoc = -1

    // ---- Recursos GL ----
    private var horizonVboId = 0
    private var horizonVertexCount = 0

    private var labelTextureId = 0
    private var labelVertexVboId = 0
    private var labelIndexVboId = 0
    private var labelIndexCount = 0

    private var glResourcesUploaded = false

    // ---- Shaders ----------------------------------------------------------

    // Shader simple para la línea de horizonte: color uniforme.
    private val lineVertexShaderSource = """
        attribute vec3 aPosition;
        uniform mat4 uMvpMatrix;
        void main() {
            gl_Position = uMvpMatrix * vec4(aPosition, 1.0);
        }
    """.trimIndent()

    private val lineFragmentShaderSource = """
        precision mediump float;
        uniform vec4 uColor;
        void main() {
            gl_FragColor = uColor;
        }
    """.trimIndent()

    // Shader para quads con textura: sample del atlas y discard si transparente.
    private val labelVertexShaderSource = """
        attribute vec3 aPosition;
        attribute vec2 aTexCoord;
        varying vec2 vTexCoord;
        uniform mat4 uMvpMatrix;
        void main() {
            gl_Position = uMvpMatrix * vec4(aPosition, 1.0);
            vTexCoord = aTexCoord;
        }
    """.trimIndent()

    // Alpha test con discard: descarta los píxeles vacíos del slot. Los píxeles
    // con texto (alpha > 0.1) se renderean con blend normal.
    private val labelFragmentShaderSource = """
        precision mediump float;
        varying vec2 vTexCoord;
        uniform sampler2D uTexture;
        void main() {
            vec4 c = texture2D(uTexture, vTexCoord);
            if (c.a < 0.1) discard;
            gl_FragColor = c;
        }
    """.trimIndent()

    // ---- API pública ------------------------------------------------------

    fun onSurfaceCreated() {
        lineProgram = 0
        lineAPositionLoc = -1
        lineUMvpMatrixLoc = -1
        lineUColorLoc = -1
        labelProgram = 0
        labelAPositionLoc = -1
        labelATexCoordLoc = -1
        labelUMvpMatrixLoc = -1
        labelUTextureLoc = -1
        horizonVboId = 0
        horizonVertexCount = 0
        labelTextureId = 0
        labelVertexVboId = 0
        labelIndexVboId = 0
        labelIndexCount = 0
        glResourcesUploaded = false
        Log.i(TAG, "onSurfaceCreated — estado GL reseteado")
    }

    fun draw(mvpMatrix: FloatArray) {
        if (lineProgram == 0) lineProgram = buildProgram(lineVertexShaderSource, lineFragmentShaderSource)
        if (labelProgram == 0) labelProgram = buildProgram(labelVertexShaderSource, labelFragmentShaderSource)

        if (!glResourcesUploaded) {
            cacheProgramLocations()
            uploadHorizon()
            uploadLabels()
            glResourcesUploaded = true
        }

        // Set state común para el overlay completo.
        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        drawHorizon(mvpMatrix)
        drawLabels(mvpMatrix)

        // Restaurar al state que asume PanoramaRenderer entre frames.
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glDisable(GLES20.GL_BLEND)
    }

    fun release() {
        val vbos = intArrayOf(horizonVboId, labelVertexVboId, labelIndexVboId).filter { it != 0 }.toIntArray()
        if (vbos.isNotEmpty()) GLES20.glDeleteBuffers(vbos.size, vbos, 0)
        if (labelTextureId != 0) GLES20.glDeleteTextures(1, intArrayOf(labelTextureId), 0)
        if (lineProgram != 0) GLES20.glDeleteProgram(lineProgram)
        if (labelProgram != 0) GLES20.glDeleteProgram(labelProgram)
        onSurfaceCreated()  // reset all to zero
    }

    // ---- Compilación de shaders ------------------------------------------

    private fun cacheProgramLocations() {
        lineAPositionLoc = GLES20.glGetAttribLocation(lineProgram, "aPosition")
        lineUMvpMatrixLoc = GLES20.glGetUniformLocation(lineProgram, "uMvpMatrix")
        lineUColorLoc = GLES20.glGetUniformLocation(lineProgram, "uColor")

        labelAPositionLoc = GLES20.glGetAttribLocation(labelProgram, "aPosition")
        labelATexCoordLoc = GLES20.glGetAttribLocation(labelProgram, "aTexCoord")
        labelUMvpMatrixLoc = GLES20.glGetUniformLocation(labelProgram, "uMvpMatrix")
        labelUTextureLoc = GLES20.glGetUniformLocation(labelProgram, "uTexture")
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
            throw RuntimeException("WorldGridOverlay3D link failed: $log")
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
            throw RuntimeException("WorldGridOverlay3D compile failed: $log")
        }
        return sh
    }

    // ---- Línea de horizonte ----------------------------------------------

    private fun uploadHorizon() {
        // Cerramos el LINE_STRIP repitiendo el primer vertex al final.
        val totalVerts = HORIZON_SEGMENTS + 1
        val data = FloatArray(totalVerts * 3)
        for (i in 0..HORIZON_SEGMENTS) {
            val azDeg = (i.toDouble() / HORIZON_SEGMENTS) * 360.0
            val azRad = Math.toRadians(azDeg)
            // Altitud = 0 → cosAlt=1, sinAlt=0
            data[i * 3 + 0] = sin(azRad).toFloat()   // X = East
            data[i * 3 + 1] = cos(azRad).toFloat()   // Y = North
            data[i * 3 + 2] = 0f                      // Z = Up
        }
        horizonVertexCount = totalVerts
        horizonVboId = createBufferVbo(data)
    }

    private fun drawHorizon(mvpMatrix: FloatArray) {
        GLES20.glUseProgram(lineProgram)
        GLES20.glUniformMatrix4fv(lineUMvpMatrixLoc, 1, false, mvpMatrix, 0)
        GLES20.glUniform4fv(lineUColorLoc, 1, HORIZON_COLOR_RGBA, 0)
        GLES20.glLineWidth(HORIZON_LINE_WIDTH)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, horizonVboId)
        GLES20.glEnableVertexAttribArray(lineAPositionLoc)
        GLES20.glVertexAttribPointer(lineAPositionLoc, 3, GLES20.GL_FLOAT, false, 0, 0)
        GLES20.glDrawArrays(GLES20.GL_LINE_STRIP, 0, horizonVertexCount)

        GLES20.glDisableVertexAttribArray(lineAPositionLoc)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    // ---- Etiquetas de azimut ---------------------------------------------

    private fun uploadLabels() {
        // 1. Generar el atlas bitmap.
        val bitmap = generateLabelAtlasBitmap()
        labelTextureId = uploadTexture(bitmap)
        bitmap.recycle()

        // 2. Generar geometría (vertices + indices) de los 36 quads.
        val vertexData = FloatArray(LABEL_COUNT * 4 * FLOATS_PER_VERTEX)
        val indexData = ShortArray(LABEL_COUNT * 6)

        val altRad = Math.toRadians(LABEL_ALTITUDE_DEG)
        val cosAlt = cos(altRad).toFloat()
        val sinAlt = sin(altRad).toFloat()
        val halfW = LABEL_QUAD_WIDTH / 2f
        val halfH = LABEL_QUAD_HEIGHT / 2f

        var vIdx = 0
        var iIdx = 0

        for (i in 0 until LABEL_COUNT) {
            val azRad = Math.toRadians((i * LABEL_INTERVAL_DEG).toDouble())
            val sinAz = sin(azRad).toFloat()
            val cosAz = cos(azRad).toFloat()

            // Centro del quad en ENU (esfera radio 1).
            val cx = cosAlt * sinAz
            val cy = cosAlt * cosAz
            val cz = sinAlt

            // Right del quad = tangente al círculo horizontal (perpendicular al radio
            // y al up mundo). Para az=0° apunta a +X (Este); para az=90° a -Y (Sur).
            // up del quad = +Z mundo, válido mientras la altitud sea baja.
            val rx = cosAz
            val ry = -sinAz
            val ux = 0f
            val uy = 0f
            val uz = 1f

            // UV del slot dentro del atlas.
            val col = i % ATLAS_COLS
            val row = i / ATLAS_COLS
            val uLeft = col.toFloat() / ATLAS_COLS
            val uRight = (col + 1).toFloat() / ATLAS_COLS
            val vTop = row.toFloat() / ATLAS_ROWS
            val vBottom = (row + 1).toFloat() / ATLAS_ROWS

            // 4 vertices: BL, BR, TR, TL (CCW visto desde el observador).
            // BL = centro - right*halfW - up*halfH
            vertexData[vIdx++] = cx - rx * halfW - ux * halfH
            vertexData[vIdx++] = cy - ry * halfW - uy * halfH
            vertexData[vIdx++] = cz - 0f      - uz * halfH
            vertexData[vIdx++] = uLeft
            vertexData[vIdx++] = vBottom
            // BR
            vertexData[vIdx++] = cx + rx * halfW - ux * halfH
            vertexData[vIdx++] = cy + ry * halfW - uy * halfH
            vertexData[vIdx++] = cz + 0f       - uz * halfH
            vertexData[vIdx++] = uRight
            vertexData[vIdx++] = vBottom
            // TR
            vertexData[vIdx++] = cx + rx * halfW + ux * halfH
            vertexData[vIdx++] = cy + ry * halfW + uy * halfH
            vertexData[vIdx++] = cz + 0f       + uz * halfH
            vertexData[vIdx++] = uRight
            vertexData[vIdx++] = vTop
            // TL
            vertexData[vIdx++] = cx - rx * halfW + ux * halfH
            vertexData[vIdx++] = cy - ry * halfW + uy * halfH
            vertexData[vIdx++] = cz - 0f       + uz * halfH
            vertexData[vIdx++] = uLeft
            vertexData[vIdx++] = vTop

            // Triángulos: BL, BR, TR  +  BL, TR, TL
            val baseV = (i * 4).toShort()
            indexData[iIdx++] = baseV
            indexData[iIdx++] = (baseV + 1).toShort()
            indexData[iIdx++] = (baseV + 2).toShort()
            indexData[iIdx++] = baseV
            indexData[iIdx++] = (baseV + 2).toShort()
            indexData[iIdx++] = (baseV + 3).toShort()
        }

        labelIndexCount = indexData.size
        labelVertexVboId = createBufferVbo(vertexData)
        labelIndexVboId = createIndexVbo(indexData)
    }

    private fun generateLabelAtlasBitmap(): Bitmap {
        val bmp = Bitmap.createBitmap(ATLAS_WIDTH_PX, ATLAS_HEIGHT_PX, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bmp)

        val paintNumeric = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(235, 255, 255, 255)
            textAlign = Paint.Align.CENTER
            textSize = 36f
            setShadowLayer(4f, 0f, 0f, Color.BLACK)
        }
        val paintCardinal = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textAlign = Paint.Align.CENTER
            textSize = 52f
            isFakeBoldText = true
            setShadowLayer(5f, 0f, 0f, Color.BLACK)
        }

        for (i in 0 until LABEL_COUNT) {
            val azDeg = i * LABEL_INTERVAL_DEG
            val col = i % ATLAS_COLS
            val row = i / ATLAS_COLS
            val cx = col * SLOT_WIDTH_PX + SLOT_WIDTH_PX / 2f
            val cyTop = row * SLOT_HEIGHT_PX

            val paint = if (azDeg % 90 == 0) paintCardinal else paintNumeric
            val label = labelFor(azDeg)

            // Centrado vertical en el slot — usar fontMetrics para baseline correcto.
            val fm = paint.fontMetrics
            val baselineY = cyTop + SLOT_HEIGHT_PX / 2f - (fm.ascent + fm.descent) / 2f
            canvas.drawText(label, cx, baselineY, paint)
        }

        return bmp
    }

    private fun labelFor(azDeg: Int): String = when (azDeg) {
        0 -> "N"
        90 -> "E"
        180 -> "S"
        270 -> "O"
        else -> "${azDeg}°"
    }

    private fun uploadTexture(bitmap: Bitmap): Int {
        val ids = IntArray(1)
        GLES20.glGenTextures(1, ids, 0)
        val id = ids[0]
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, id)
        // Sin mipmaps — el atlas no es power-of-two y el filtering LINEAR alcanza.
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0)
        Log.i(TAG, "atlas de etiquetas subido — textureId=$id (${ATLAS_WIDTH_PX}x${ATLAS_HEIGHT_PX})")
        return id
    }

    private fun drawLabels(mvpMatrix: FloatArray) {
        GLES20.glUseProgram(labelProgram)
        GLES20.glUniformMatrix4fv(labelUMvpMatrixLoc, 1, false, mvpMatrix, 0)

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, labelTextureId)
        GLES20.glUniform1i(labelUTextureLoc, 0)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, labelVertexVboId)
        GLES20.glEnableVertexAttribArray(labelAPositionLoc)
        GLES20.glVertexAttribPointer(
            labelAPositionLoc, 3, GLES20.GL_FLOAT, false, BYTES_PER_VERTEX, 0
        )
        GLES20.glEnableVertexAttribArray(labelATexCoordLoc)
        GLES20.glVertexAttribPointer(
            labelATexCoordLoc, 2, GLES20.GL_FLOAT, false, BYTES_PER_VERTEX, 12
        )

        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, labelIndexVboId)
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, labelIndexCount, GLES20.GL_UNSIGNED_SHORT, 0)

        GLES20.glDisableVertexAttribArray(labelAPositionLoc)
        GLES20.glDisableVertexAttribArray(labelATexCoordLoc)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, 0)
    }

    // ---- Helpers de VBO ---------------------------------------------------

    private fun createBufferVbo(data: FloatArray): Int {
        val buf: FloatBuffer = ByteBuffer
            .allocateDirect(data.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply { put(data); position(0) }

        val ids = IntArray(1)
        GLES20.glGenBuffers(1, ids, 0)
        val vbo = ids[0]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, data.size * 4, buf, GLES20.GL_STATIC_DRAW)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        return vbo
    }

    private fun createIndexVbo(data: ShortArray): Int {
        val buf: ShortBuffer = ByteBuffer
            .allocateDirect(data.size * 2)
            .order(ByteOrder.nativeOrder())
            .asShortBuffer()
            .apply { put(data); position(0) }

        val ids = IntArray(1)
        GLES20.glGenBuffers(1, ids, 0)
        val vbo = ids[0]
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, vbo)
        GLES20.glBufferData(GLES20.GL_ELEMENT_ARRAY_BUFFER, data.size * 2, buf, GLES20.GL_STATIC_DRAW)
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, 0)
        return vbo
    }
}