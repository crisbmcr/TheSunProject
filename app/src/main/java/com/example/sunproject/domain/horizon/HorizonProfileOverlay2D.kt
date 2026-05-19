package com.example.sunproject.domain.horizon

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.Log
import com.example.sunproject.domain.atlas.AtlasConfig
import com.example.sunproject.domain.atlas.AtlasMath
import java.io.File
import java.io.FileOutputStream

/**
 * Superpone un HorizonProfile sobre el bitmap de un atlas equirectangular.
 *
 * Convenciones:
 *  - El overlay escribe directamente sobre el bitmap recibido (mutable).
 *  - Usa AtlasMath para mapear (az, alt) → (x, y), igual que SolarPathOverlay,
 *    así el perfil cae sobre los mismos píxeles que cualquier curva del ábaco.
 *  - Es seguro componerlo CON el SolarPathOverlay: dibujar primero el chart y
 *    después el perfil; el orden afecta solo qué capa queda visible donde se
 *    superponen.
 *
 * Patrón visual:
 *  - Línea verde lima sobre la frontera detectada.
 *  - Relleno verde-lima semi-transparente hacia abajo (zona obstruida).
 *
 * El estilo es configurable vía Style. Default = recomendado para Cauchari
 * (verde lima sobre cielos azules del ábaco contrasta bien).
 */
object HorizonProfileOverlay2D {

    private const val TAG = "HorizonOverlay2D"

    /** Verde lima saturado, contrasta con cielo azul y terreno gris/marrón. */
    private val DEFAULT_LINE_COLOR = Color.argb(255, 166, 255, 0)

    /** Mismo verde con alpha ~32%: zona obstruida visible sin tapar lo de abajo. */
    private val DEFAULT_FILL_COLOR = Color.argb(80, 166, 255, 0)

    data class Style(
        val lineColor: Int = DEFAULT_LINE_COLOR,
        val lineWidthPx: Float = 3.0f,
        val fillColor: Int = DEFAULT_FILL_COLOR,
        val drawFill: Boolean = true,
        val drawLine: Boolean = true
    ) {
        companion object {
            val DEFAULT = Style()
        }
    }

    /**
     * Dibuja el perfil sobre el bitmap. El bitmap debe ser ARGB_8888 mutable.
     *
     * Se itera por TODAS las columnas del bitmap (no por los buckets del perfil)
     * para que la curva tenga la resolución del atlas y no escalones visibles
     * entre buckets vecinos. profile.altAtAzimuthDeg() resuelve cada columna a
     * su altitud agregada.
     */
    fun drawProfile(
        atlasBitmap: Bitmap,
        atlasConfig: AtlasConfig,
        profile: HorizonProfile,
        style: Style = Style.DEFAULT
    ) {
        require(atlasBitmap.width == atlasConfig.widthPx && atlasBitmap.height == atlasConfig.heightPx) {
            "Bitmap (${atlasBitmap.width}×${atlasBitmap.height}) no coincide con AtlasConfig " +
                    "(${atlasConfig.widthPx}×${atlasConfig.heightPx})"
        }
        if (!style.drawFill && !style.drawLine) return

        val canvas = Canvas(atlasBitmap)
        val w = atlasConfig.widthPx
        val h = atlasConfig.heightPx

        // Construir la polilínea del perfil columna por columna.
        // Cada columna del atlas se mapea a un azimut atlas [-180, 180], que
        // normalizamos a NOAA [0, 360) antes de pedirle al perfil su altitud.
        val xs = FloatArray(w)
        val ys = FloatArray(w)
        for (x in 0 until w) {
            val azAtlas = AtlasMath.xToAzimuth(x, atlasConfig)
            var azNoaa = azAtlas
            if (azNoaa < 0f) azNoaa += 360f
            val altDeg = profile.altAtAzimuthDeg(azNoaa)
            xs[x] = x.toFloat()
            ys[x] = AtlasMath.altitudeToY(altDeg, atlasConfig).toFloat()
        }

        // Fill primero (debajo de la línea visualmente).
        if (style.drawFill) {
            val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = style.fillColor
                this.style = Paint.Style.FILL
            }
            val fillPath = Path().apply {
                moveTo(xs[0], ys[0])
                for (i in 1 until w) lineTo(xs[i], ys[i])
                // Cerrar al borde inferior del atlas (alt=0°).
                lineTo((w - 1).toFloat(), (h - 1).toFloat())
                lineTo(0f, (h - 1).toFloat())
                close()
            }
            canvas.drawPath(fillPath, fillPaint)
        }

        // Línea sobre el fill.
        if (style.drawLine) {
            val linePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = style.lineColor
                this.style = Paint.Style.STROKE
                strokeWidth = style.lineWidthPx
                strokeCap = Paint.Cap.ROUND
                strokeJoin = Paint.Join.ROUND
            }
            val linePath = Path().apply {
                moveTo(xs[0], ys[0])
                for (i in 1 until w) lineTo(xs[i], ys[i])
            }
            canvas.drawPath(linePath, linePaint)
        }
    }

    /**
     * Conveniencia: lee el atlas desde disco, aplica el overlay, y escribe un
     * PNG nuevo. Si outFile es null, lo deriva del atlas: <nombre>_horizon.png.
     * Útil para tener una versión del atlas con el perfil pegado sin tener el
     * bitmap en memoria.
     *
     * Devuelve el archivo escrito o null si algo falló.
     */
    fun exportWithProfile(
        atlasFile: File,
        atlasConfig: AtlasConfig,
        profile: HorizonProfile,
        style: Style = Style.DEFAULT,
        outFile: File? = null
    ): File? {
        if (!atlasFile.exists()) {
            Log.w(TAG, "exportWithProfile: atlas no existe ${atlasFile.absolutePath}")
            return null
        }

        val target = outFile ?: run {
            val parent = atlasFile.parentFile ?: return null
            File(parent, atlasFile.nameWithoutExtension + "_horizon.png")
        }

        return try {
            val base = BitmapFactory.decodeFile(atlasFile.absolutePath)
                ?: return null.also {
                    Log.w(TAG, "exportWithProfile: no se pudo decodificar ${atlasFile.absolutePath}")
                }
            val mutable = base.copy(Bitmap.Config.ARGB_8888, true)
            base.recycle()

            drawProfile(mutable, atlasConfig, profile, style)

            target.parentFile?.mkdirs()
            FileOutputStream(target).use { out ->
                mutable.compress(Bitmap.CompressFormat.PNG, 100, out)
            }
            mutable.recycle()

            Log.i(TAG, "Exported atlas+horizon to ${target.absolutePath}")
            target
        } catch (t: Throwable) {
            Log.e(TAG, "exportWithProfile failed", t)
            null
        }
    }
}