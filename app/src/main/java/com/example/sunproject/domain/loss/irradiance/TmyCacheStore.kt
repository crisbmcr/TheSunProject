package com.example.sunproject.domain.loss.irradiance

import android.content.Context
import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.util.Locale

/**
 * Cache local persistente del TMY de PVGIS.
 *
 * Persiste el resultado del parser ([TmyJsonParser] u otro en el futuro)
 * como JSON en `context.filesDir/irradiance/pvgis_tmy_lat<lat>_lon<lon>.json`.
 *
 * **Importante**: este JSON local es independiente del JSON que devuelve
 * PVGIS. Acá guardamos un formato propio con metadata (schemaVersion,
 * downloadedAtUtcMs) que el JSON de PVGIS no tiene. Si en el futuro
 * cambiamos el parser de upstream, el cache local sigue funcionando.
 *
 * ## Decisiones de diseño
 *
 * - **filesDir, no cacheDir**: el TMY tarda en bajarse (varios segundos en
 *   conexiones lentas) y es válido por años. Vale la pena persistencia
 *   firme; cacheDir lo puede limpiar Android bajo presión.
 *
 * - **Cache key redondeado a 2 decimales**: PVGIS-ERA5 tiene celdas de
 *   ~28 km, 2 decimales ≈ 1 km de precisión. Suficiente y evita re-descargas
 *   por GPS levemente distinto entre sesiones del mismo sitio.
 *
 * - **Schema version**: incluido desde día uno. Si en C.3 cambiamos
 *   formato (ej. agregar IAM, espectro, albedo dinámico), bumpeamos
 *   [SCHEMA_VERSION] y forzamos re-parse/re-download del cache.
 *
 * - **Independencia del año del cálculo**: el JSON local guarda
 *   dni/dhi/ghi/t/ws planos. El año del cálculo (yearStartEpochUtc) NO
 *   se guarda — se asigna al construir el [HourlyStepIrradianceSeries]
 *   desde el caller, porque el TMY es agnóstico al año real de cada
 *   muestra (decisión arquitectural de C.2.1).
 */
internal class TmyCacheStore(private val context: Context) {

    /**
     * Lee el cache para las coordenadas dadas. Si no existe, está corrupto,
     * o el schema version no coincide, devuelve null.
     */
    fun load(lat: Double, lon: Double): TmyParsedData.ParsedTmy? {
        val file = fileFor(lat, lon)
        if (!file.exists()) {
            Log.d(TAG, "Cache miss para lat=$lat lon=$lon (file no existe)")
            return null
        }
        return try {
            val obj = JSONObject(file.readText(Charsets.UTF_8))
            val schema = obj.optInt("schemaVersion", -1)
            if (schema != SCHEMA_VERSION) {
                Log.w(
                    TAG,
                    "Cache schema mismatch: actual=$schema esperado=$SCHEMA_VERSION. " +
                            "Se ignora el cache y se re-descarga."
                )
                return null
            }

            val hourlyArr = obj.getJSONArray("hourly")
            if (hourlyArr.length() != TmyParsedData.HOURS_PER_YEAR) {
                Log.w(
                    TAG,
                    "Cache corrupto: ${hourlyArr.length()} muestras, " +
                            "se esperaban ${TmyParsedData.HOURS_PER_YEAR}"
                )
                return null
            }

            val hourly = Array(TmyParsedData.HOURS_PER_YEAR) { i ->
                val o = hourlyArr.getJSONObject(i)
                IrradianceSample(
                    dniWm2 = o.optDouble("dni", 0.0).coerceAtLeast(0.0),
                    dhiWm2 = o.optDouble("dhi", 0.0).coerceAtLeast(0.0),
                    ghiWm2 = o.optDouble("ghi", 0.0).coerceAtLeast(0.0),
                    airTempC = if (o.has("t") && !o.isNull("t")) o.getDouble("t") else null,
                    windSpeedMs = if (o.has("ws") && !o.isNull("ws"))
                        o.getDouble("ws").coerceAtLeast(0.0)
                    else null
                )
            }

            val monthSourcesArr = obj.optJSONArray("monthSources")
            val monthSources = if (monthSourcesArr != null) {
                List(monthSourcesArr.length()) { i ->
                    val o = monthSourcesArr.getJSONObject(i)
                    TmyParsedData.MonthSource(
                        month = o.getInt("month"),
                        year = o.getInt("year")
                    )
                }
            } else emptyList()

            val downloadedAt = obj.optLong("downloadedAtUtcMs", 0L)
            val ageDays = (System.currentTimeMillis() - downloadedAt) / (1000L * 60 * 60 * 24)
            Log.i(
                TAG,
                "Cache hit: ${file.name} (descargado hace ${ageDays}d, schema=$schema)"
            )

            TmyParsedData.ParsedTmy(
                hourly = hourly,
                latitudeDeg = obj.getDouble("lat"),
                longitudeDeg = obj.getDouble("lon"),
                elevationM = if (obj.has("elevationM") && !obj.isNull("elevationM"))
                    obj.getDouble("elevationM")
                else null,
                irradianceTimeOffsetH = if (obj.has("irradianceTimeOffsetH") && !obj.isNull("irradianceTimeOffsetH"))
                    obj.getDouble("irradianceTimeOffsetH")
                else null,
                monthSources = monthSources
            )
        } catch (t: Throwable) {
            Log.w(TAG, "Fallo leyendo cache ${file.name}: ${t.message}")
            null
        }
    }

    /**
     * Persiste el TMY parseado. Sobrescribe el cache previo si existía.
     * Errores de I/O se loguean pero no se propagan (el caller ya tiene
     * los datos en memoria, perder el cache no es crítico).
     */
    fun save(lat: Double, lon: Double, parsed: TmyParsedData.ParsedTmy) {
        val dir = cacheDir()
        if (!dir.exists() && !dir.mkdirs()) {
            Log.w(TAG, "No se pudo crear directorio ${dir.absolutePath}")
            return
        }
        val file = fileFor(lat, lon)

        try {
            val hourlyArr = JSONArray()
            parsed.hourly.forEach { s ->
                val o = JSONObject().apply {
                    put("dni", s.dniWm2)
                    put("dhi", s.dhiWm2)
                    put("ghi", s.ghiWm2)
                    s.airTempC?.let { put("t", it) }
                    s.windSpeedMs?.let { put("ws", it) }
                }
                hourlyArr.put(o)
            }

            val monthSourcesArr = JSONArray()
            parsed.monthSources.forEach { ms ->
                monthSourcesArr.put(
                    JSONObject().apply {
                        put("month", ms.month)
                        put("year", ms.year)
                    }
                )
            }

            val root = JSONObject().apply {
                put("schemaVersion", SCHEMA_VERSION)
                put("source", "PVGIS-ERA5-TMY")
                put("downloadedAtUtcMs", System.currentTimeMillis())
                put("lat", parsed.latitudeDeg)
                put("lon", parsed.longitudeDeg)
                parsed.elevationM?.let { put("elevationM", it) }
                parsed.irradianceTimeOffsetH?.let { put("irradianceTimeOffsetH", it) }
                put("monthSources", monthSourcesArr)
                put("hourly", hourlyArr)
            }

            // toString(2) para pretty-print humano-legible. Costo: ~30% más
            // de tamaño (el archivo queda en ~1.2 MB). Aceptable para
            // facilidad de inspección y debug.
            file.writeText(root.toString(2), Charsets.UTF_8)
            Log.i(TAG, "Cache guardado: ${file.name} (${file.length() / 1024} KB)")
        } catch (t: Throwable) {
            Log.w(TAG, "Fallo guardando cache ${file.name}: ${t.message}", t)
        }
    }

    /** Para diagnóstico/UI: devuelve true si hay cache válido para el sitio. */
    fun has(lat: Double, lon: Double): Boolean = fileFor(lat, lon).exists()

    /** Para tests/debugging: borra el cache de un sitio. */
    fun delete(lat: Double, lon: Double): Boolean {
        val f = fileFor(lat, lon)
        return if (f.exists()) f.delete() else false
    }

    private fun cacheDir(): File = File(context.filesDir, CACHE_SUBDIR)

    private fun fileFor(lat: Double, lon: Double): File {
        // Redondeo a 2 decimales. Formato con Locale.US para asegurar punto
        // decimal (filenames con coma decimal son fuente de bugs).
        val key = String.format(
            Locale.US,
            "pvgis_tmy_lat%.2f_lon%.2f.json",
            lat, lon
        )
        return File(cacheDir(), key)
    }

    companion object {
        private const val TAG = "PvgisTmy.Cache"
        private const val CACHE_SUBDIR = "irradiance"

        /**
         * Versión del schema del cache. Bumpear cuando cambie la estructura
         * del JSON persistido (ej. agregar nuevos campos no opcionales,
         * cambiar unidades, etc).
         */
        const val SCHEMA_VERSION = 1
    }
}