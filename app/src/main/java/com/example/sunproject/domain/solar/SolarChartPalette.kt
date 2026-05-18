package com.example.sunproject.domain.solar

import android.graphics.Color

/**
 * Paleta compartida del ábaco solar — fuente única de verdad para colores entre
 * el overlay 2D (SolarPathOverlay sobre Canvas) y el overlay 3D (SolarChartOverlay3D
 * en GL). Si querés cambiar un color del ábaco, este es el lugar.
 *
 * Hemisferio Sur: diciembre = verano (rojo/cálido), junio = invierno (azul frío).
 */
object SolarChartPalette {

    /** Color de la trayectoria diaria del mes 1..12. */
    fun colorForMonth(month: Int): Int = when (month) {
        12 -> Color.rgb(226, 75, 74)
        1, 11 -> Color.rgb(232, 93, 36)
        2, 10 -> Color.rgb(239, 159, 39)
        3, 9 -> Color.rgb(186, 117, 23)
        4, 8 -> Color.rgb(151, 196, 89)
        5, 7 -> Color.rgb(29, 158, 117)
        6 -> Color.rgb(55, 138, 221)
        else -> Color.GRAY
    }

    /** Líneas horarias (analemas). Con alpha bajo para que no compitan con las diarias. */
    val HOUR_LINE_COLOR = Color.argb(140, 250, 199, 117)

    /** Etiqueta de hora ("12h solar"). Mismo tono que la línea, sin alpha. */
    val HOUR_LABEL_COLOR = Color.rgb(250, 199, 117)

    /** Trayectoria del día de la captura — blanco para destacarse sobre todo. */
    val CAPTURE_DAY_COLOR = Color.rgb(255, 255, 255)

    /** Marker del sol: relleno amarillo cálido. */
    val SUN_FILL_COLOR = Color.rgb(249, 203, 66)

    /** Marker del sol: anillo marrón oscuro. */
    val SUN_RING_COLOR = Color.rgb(133, 79, 11)
}