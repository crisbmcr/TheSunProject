package com.example.sunproject.domain.loss.irradiance

import android.util.Log
import org.json.JSONException
import org.json.JSONObject
import java.io.IOException

/**
 * Parser del formato JSON nativo de PVGIS para el endpoint TMY.
 *
 * ## Formato esperado (PVGIS v5.3)
 *
 * ```json
 * {
 *   "inputs": {
 *     "location": {
 *       "latitude": -24.093,
 *       "longitude": -66.717,
 *       "elevation": 4003.0,
 *       "irradiance_time_offset": 0.5
 *     },
 *     "meteo_data": { ... }
 *   },
 *   "outputs": {
 *     "months_selected": [
 *       {"month": 1, "year": 2020},
 *       {"month": 2, "year": 2017},
 *       ...
 *     ],
 *     "tmy_hourly": [
 *       {
 *         "time(UTC)": "20200101:0000",
 *         "T2m": 8.34, "RH": 71.03,
 *         "G(h)": 0.0, "Gb(n)": 0.0, "Gd(h)": 0.0,
 *         "IR(h)": 269.35, "WS10m": 1.69, "WD10m": 67.0, "SP": 62130.0
 *       },
 *       ... (8760 entradas)
 *     ]
 *   },
 *   "meta": { ... }
 * }
 * ```
 *
 * ## Mapeo a [IrradianceSample]
 *
 *   G(h)   → ghiWm2     (Global Horizontal Irradiance, W/m²)
 *   Gb(n)  → dniWm2     (Direct Normal Irradiance, W/m²)
 *   Gd(h)  → dhiWm2     (Diffuse Horizontal Irradiance, W/m²)
 *   T2m    → airTempC   (temperatura 2m, °C)
 *   WS10m  → windSpeedMs (viento 10m, m/s)
 *
 * ## Convención de unidades
 *
 * PVGIS reporta DNI/DHI/GHI en W/m² instantáneos al timestamp con offset
 * de +0.5h (el valor "20200101:0000" representa la media de [00:00, 01:00)
 * centrada en 00:30). [HourlyStepIrradianceSeries] mapea cualquier epoch
 * dentro de la hora a la misma muestra, así que el offset no afecta el
 * cálculo a nivel de paso de 15 min.
 *
 * ## Manejo de valores negativos
 *
 * En algunas filas PVGIS reporta `-0.0` (cero negativo) o pequeños valores
 * ligeramente negativos por ruido del modelo de ERA5. El parser aplica
 * `coerceAtLeast(0.0)` antes de construir [IrradianceSample] para no
 * fallar en su validación `require(>= 0)`.
 *
 * ## Por qué JSON y no CSV
 *
 *  - Campos auto-descriptivos (no hay que adivinar índices ni contar
 *    líneas de header).
 *  - Anidamiento explícito (`inputs.location.latitude`) auto-valida la
 *    estructura del documento — si PVGIS cambia el orden de columnas o
 *    introduce variantes regionales del CSV, el JSON sigue funcionando.
 *  - Inspección humana directa para debug.
 *  - Un solo formato consistente entre upstream (PVGIS) y cache local.
 */
internal object TmyJsonParser {

    private const val TAG = "PvgisTmy.JsonParser"

    /**
     * Parsea el JSON crudo de PVGIS a [TmyParsedData.ParsedTmy].
     *
     * @throws IOException si el JSON tiene estructura inesperada (campos
     *   faltantes, tipos incorrectos, conteos no esperados).
     */
    fun parse(jsonText: String): TmyParsedData.ParsedTmy {
        val root = try {
            JSONObject(jsonText)
        } catch (e: JSONException) {
            throw IOException("Respuesta de PVGIS no es JSON válido: ${e.message}", e)
        }

        // ============================================================
        // inputs.location: metadatos del sitio
        // ============================================================
        val inputs = root.optJSONObject("inputs")
            ?: throw IOException("JSON PVGIS sin nodo 'inputs'")
        val location = inputs.optJSONObject("location")
            ?: throw IOException("JSON PVGIS sin nodo 'inputs.location'")

        val lat = location.optDouble("latitude", Double.NaN).also {
            if (it.isNaN()) throw IOException("Falta 'inputs.location.latitude'")
        }
        val lon = location.optDouble("longitude", Double.NaN).also {
            if (it.isNaN()) throw IOException("Falta 'inputs.location.longitude'")
        }
        val elev = optDoubleOrNull(location, "elevation")
        val timeOffset = optDoubleOrNull(location, "irradiance_time_offset")

        // ============================================================
        // outputs: months_selected + tmy_hourly
        // ============================================================
        val outputs = root.optJSONObject("outputs")
            ?: throw IOException("JSON PVGIS sin nodo 'outputs'")

        val monthsArr = outputs.optJSONArray("months_selected")
            ?: throw IOException("JSON PVGIS sin 'outputs.months_selected'")
        if (monthsArr.length() != 12) {
            throw IOException(
                "Se esperaban 12 month sources, leídos: ${monthsArr.length()}"
            )
        }
        val monthSources = List(12) { i ->
            val o = monthsArr.getJSONObject(i)
            TmyParsedData.MonthSource(
                month = o.getInt("month"),
                year = o.getInt("year")
            )
        }

        val hourlyArr = outputs.optJSONArray("tmy_hourly")
            ?: throw IOException("JSON PVGIS sin 'outputs.tmy_hourly'")
        if (hourlyArr.length() != TmyParsedData.HOURS_PER_YEAR) {
            throw IOException(
                "Se esperaban ${TmyParsedData.HOURS_PER_YEAR} entradas horarias, " +
                        "leídas: ${hourlyArr.length()}"
            )
        }

        // ============================================================
        // Conversión de cada entrada horaria a IrradianceSample
        // ============================================================
        val hourly = Array(TmyParsedData.HOURS_PER_YEAR) { i ->
            val o = hourlyArr.getJSONObject(i)
            try {
                IrradianceSample(
                    dniWm2 = o.getDouble("Gb(n)").coerceAtLeast(0.0),
                    dhiWm2 = o.getDouble("Gd(h)").coerceAtLeast(0.0),
                    ghiWm2 = o.getDouble("G(h)").coerceAtLeast(0.0),
                    airTempC = optDoubleOrNull(o, "T2m"),
                    windSpeedMs = optDoubleOrNull(o, "WS10m")?.coerceAtLeast(0.0)
                )
            } catch (e: JSONException) {
                throw IOException(
                    "Entrada horaria #$i mal formada (time=${o.optString("time(UTC)")}): ${e.message}",
                    e
                )
            }
        }

        Log.i(
            TAG,
            "Parseo OK: lat=$lat lon=$lon elev=$elev offset=$timeOffset " +
                    "muestras=${hourly.size}"
        )

        return TmyParsedData.ParsedTmy(
            hourly = hourly,
            latitudeDeg = lat,
            longitudeDeg = lon,
            elevationM = elev,
            irradianceTimeOffsetH = timeOffset,
            monthSources = monthSources
        )
    }

    /**
     * Lee un Double de un campo opcional. Devuelve null si el campo no
     * está presente, es JSON null, o no es numérico.
     */
    private fun optDoubleOrNull(o: JSONObject, key: String): Double? {
        if (!o.has(key) || o.isNull(key)) return null
        return try {
            o.getDouble(key)
        } catch (_: JSONException) {
            null
        }
    }
}