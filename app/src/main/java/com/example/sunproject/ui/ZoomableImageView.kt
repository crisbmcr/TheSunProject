package com.example.sunproject.ui

import android.content.Context
import android.graphics.Matrix
import android.graphics.RectF
import android.util.AttributeSet
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import androidx.appcompat.widget.AppCompatImageView
import kotlin.math.max
import kotlin.math.min

/**
 * ImageView con pinch-zoom, double-tap-zoom y pan por arrastre.
 *
 * Drop-in replacement de ImageView: setImageBitmap()/setImageURI() siguen
 * funcionando igual. El zoom es puramente visual — no altera el bitmap ni
 * ningún pipeline. Pensado para inspeccionar el atlas equirectangular 3600×900,
 * que con fitCenter queda muy chico en altura.
 *
 * Implementación:
 *  - scaleType forzado a MATRIX; toda la transformación va en imageMatrix.
 *  - El "fit" base (imagen entera centrada) se recalcula en cada cambio de
 *    tamaño de la vista o de imagen. El zoom del usuario se compone sobre él.
 *  - Pan clampeado a los bordes de la imagen: nunca se separa del viewport
 *    mostrando vacío, salvo centrado cuando la imagen es más chica que la vista
 *    en algún eje.
 *
 * Sin dependencias externas.
 */
class ZoomableImageView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0
) : AppCompatImageView(context, attrs, defStyle) {

    companion object {
        private const val MIN_SCALE = 1.0f       // 1.0 = imagen completa (fit)
        private const val MAX_SCALE = 8.0f       // tope de ampliación
        private const val DOUBLE_TAP_SCALE = 3.0f
    }

    // Matriz que se aplica a la imagen. Se reconstruye como: fit * zoomUser.
    private val matrixApplied = Matrix()

    // Escala y traslación actuales RELATIVAS al fit base (1.0 = fit, sin pan).
    private var currentScale = 1.0f
    private var transX = 0f
    private var transY = 0f

    // Parámetros del fit base (calculados desde el tamaño de vista e imagen).
    private var fitScale = 1.0f
    private var fitDx = 0f
    private var fitDy = 0f

    private val scaleDetector = ScaleGestureDetector(context, ScaleListener())
    private val gestureDetector = GestureDetector(context, GestureListener())

    init {
        scaleType = ScaleType.MATRIX
        // Necesario para recibir todos los eventos de touch.
        isClickable = true
    }

    // ---- Recalcular fit en cambios de tamaño o de imagen ------------------

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        recomputeFitAndReset()
    }

    override fun setImageBitmap(bm: android.graphics.Bitmap?) {
        super.setImageBitmap(bm)
        recomputeFitAndReset()
    }

    override fun setImageDrawable(drawable: android.graphics.drawable.Drawable?) {
        super.setImageDrawable(drawable)
        recomputeFitAndReset()
    }

    /**
     * Calcula el fit base (imagen entera centrada en la vista) y resetea el
     * zoom del usuario. Llamado cuando cambia el tamaño de la vista o la imagen.
     */
    private fun recomputeFitAndReset() {
        val d = drawable ?: return
        val vw = width.toFloat()
        val vh = height.toFloat()
        if (vw <= 0f || vh <= 0f) return

        val iw = d.intrinsicWidth.toFloat()
        val ih = d.intrinsicHeight.toFloat()
        if (iw <= 0f || ih <= 0f) return

        // fitScale: la imagen entera entra en la vista (el lado más restrictivo).
        fitScale = min(vw / iw, vh / ih)
        // Centrar la imagen escalada en la vista.
        fitDx = (vw - iw * fitScale) / 2f
        fitDy = (vh - ih * fitScale) / 2f

        // Reset del zoom del usuario.
        currentScale = 1.0f
        transX = 0f
        transY = 0f

        applyMatrix()
    }

    // ---- Composición y aplicación de la matriz ----------------------------

    /**
     * Construye imageMatrix = [fit] luego [zoom del usuario alrededor del
     * origen de la vista] + [pan], clampeando el pan a los bordes.
     */
    private fun applyMatrix() {
        clampTranslation()

        matrixApplied.reset()
        // 1) Fit base: escala + centrado.
        matrixApplied.postScale(fitScale, fitScale)
        matrixApplied.postTranslate(fitDx, fitDy)
        // 2) Zoom del usuario, anclado al origen de la vista (0,0). El pan
        //    (transX, transY) reubica el contenido; el clamp lo mantiene válido.
        matrixApplied.postScale(currentScale, currentScale)
        matrixApplied.postTranslate(transX, transY)

        imageMatrix = matrixApplied
    }

    /**
     * Limita transX/transY para que la imagen escalada no se despegue del
     * viewport dejando huecos. Si la imagen es más chica que la vista en un
     * eje (caso típico: altura del atlas con poco zoom), la centra en ese eje.
     */
    private fun clampTranslation() {
        val d = drawable ?: return
        val vw = width.toFloat()
        val vh = height.toFloat()

        val contentW = d.intrinsicWidth * fitScale * currentScale
        val contentH = d.intrinsicHeight * fitScale * currentScale

        // Posición del borde sup-izq del contenido en coords de vista, antes de pan.
        val baseX = fitDx * currentScale
        val baseY = fitDy * currentScale

        transX = clampAxis(transX, baseX, contentW, vw)
        transY = clampAxis(transY, baseY, contentH, vh)
    }

    /**
     * Clampa el pan de un eje. base = offset del contenido sin pan;
     * content = tamaño del contenido; view = tamaño del viewport.
     */
    private fun clampAxis(t: Float, base: Float, content: Float, view: Float): Float {
        return if (content <= view) {
            // Contenido más chico que la vista: centrar (anular el pan, recentrar).
            (view - content) / 2f - base
        } else {
            // Contenido más grande: permitir pan pero sin mostrar vacío.
            val minT = view - content - base  // borde derecho/inferior pegado
            val maxT = -base                  // borde izq/superior pegado
            t.coerceIn(minT, maxT)
        }
    }

    // ---- Touch ------------------------------------------------------------

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleDetector.onTouchEvent(event)
        gestureDetector.onTouchEvent(event)
        return true
    }

    // Permitir que el scroll del zoom no sea robado por contenedores padres.
    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    private inner class ScaleListener : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean {
            val factor = detector.scaleFactor
            val newScale = (currentScale * factor).coerceIn(MIN_SCALE, MAX_SCALE)
            val applied = newScale / currentScale  // factor efectivo tras el clamp

            // Anclar el zoom al punto entre los dedos (focus): el contenido bajo
            // el foco se queda quieto mientras escala.
            val fx = detector.focusX
            val fy = detector.focusY
            transX = fx - applied * (fx - transX)
            transY = fy - applied * (fy - transY)

            currentScale = newScale
            applyMatrix()
            return true
        }
    }

    private inner class GestureListener : GestureDetector.SimpleOnGestureListener() {
        override fun onScroll(
            e1: MotionEvent?, e2: MotionEvent,
            distanceX: Float, distanceY: Float
        ): Boolean {
            // Pan: arrastrar mueve el contenido (solo tiene efecto si hay zoom
            // o si el contenido excede la vista en algún eje).
            transX -= distanceX
            transY -= distanceY
            applyMatrix()
            return true
        }

        override fun onDoubleTap(e: MotionEvent): Boolean {
            // Toggle: si ya hay zoom, vuelve a fit; si no, amplía anclado al tap.
            if (currentScale > MIN_SCALE + 0.01f) {
                currentScale = MIN_SCALE
                transX = 0f
                transY = 0f
            } else {
                val target = DOUBLE_TAP_SCALE.coerceAtMost(MAX_SCALE)
                val applied = target / currentScale
                val fx = e.x
                val fy = e.y
                transX = fx - applied * (fx - transX)
                transY = fy - applied * (fy - transY)
                currentScale = target
            }
            applyMatrix()
            return true
        }
    }
}