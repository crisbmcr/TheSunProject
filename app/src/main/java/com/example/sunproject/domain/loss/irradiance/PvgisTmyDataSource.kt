package com.example.sunproject.domain.loss.irradiance

import android.content.Context
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.time.LocalDateTime
import java.time.ZoneOffset

/**
 * Fuente de datos PVGIS-ERA5 TMY para SunProject.
 *
 * Orquesta tres componentes:
 *   - [TmyCacheStore]: persistencia local del TMY parseado.
 *   - [TmyHttpClient]: descarga del JSON desde PVGIS.
 *   - [TmyJsonParser]: parseo del JSON a [IrradianceSample]s.
 *
 * ## Flujo público
 *
 *   1. Cache hit → cargar desde disco, construir [HourlyStepIrradianceSeries],
 *      return [FetchResult.Success] con `fromCache=true`. Sin red.
 *   2. Cache miss → descargar JSON (3 reintentos con backoff), parsear,
 *      guardar en cache, return [FetchResult.Success] con `fromCache=false`.
 *   3. Cache miss + descarga falla → return [FetchResult.Failure].
 *
 * El caller (típicamente LossesActivity en C.2.4) decide qué hacer con
 * [FetchResult.Failure]: ofrecer reintento, degradar a modo geométrico,
 * o cancelar el cálculo.
 *
 * ## Threading
 *
 * El entrypoint [fetchOrLoad] es suspend. Internamente despacha I/O al
 * Dispatchers.IO. El caller debe invocarlo desde un coroutine scope
 * apropiado (lifecycleScope, viewModelScope, GlobalScope.launch, etc).
 *
 * ## Año del cálculo
 *
 * El TMY es agnóstico al año real de cada muestra (PVGIS construye el
 * TMY combinando meses de distintos años). El parámetro `year` del
 * [fetchOrLoad] define el `yearStartEpochUtc` del [HourlyStepIrradianceSeries]
 * resultante, que es la referencia temporal del mapeo timestamp → índice
 * del array horario. Debe coincidir con [ShadingLossConfig.year].
 */
class PvgisTmyDataSource(context: Context) {

    private val cache = TmyCacheStore(context)

    /**
     * Resultado posible de la operación de carga/descarga.
     */
    sealed class FetchResult {
        /**
         * TMY disponible. `series` es la fuente lista para inyectar al
         * motor. `fromCache=true` si se sirvió desde disco; `false` si
         * implicó descarga HTTP.
         */
        data class Success(
            val series: IrradianceTimeSeries,
            val fromCache: Boolean,
            val metadata: Metadata
        ) : FetchResult()

        /**
         * No se pudo obtener el TMY ni desde cache ni descargándolo.
         * El caller debe ofrecer un fallback (ej. degradar a modo
         * geométrico) o reintentar.
         */
        data class Failure(
            val reason: String,
            val cause: Throwable? = null
        ) : FetchResult()
    }

    /**
     * Metadatos descriptivos del TMY cargado. Útil para mostrar en UI
     * ("Datos PVGIS-ERA5, sitio X.XX°S Y.YY°W, elevación ZZZ m").
     */
    data class Metadata(
        val sourceLatitudeDeg: Double,
        val sourceLongitudeDeg: Double,
        val elevationM: Double?,
        val monthSources: List<TmyParsedData.MonthSource>
    )

    /**
     * Obtiene el TMY para `(lat, lon)`. Cache primero; descarga si no hay.
     *
     * @param lat latitud del sitio en grados decimales.
     * @param lon longitud del sitio en grados decimales.
     * @param year año del cálculo. Usado para construir el `yearStartEpochUtc`
     *   del [HourlyStepIrradianceSeries] resultante. Debe ser no bisiesto
     *   (mismo contrato que [ShadingLossConfig.year]).
     */
    suspend fun fetchOrLoad(
        lat: Double,
        lon: Double,
        year: Int
    ): FetchResult = withContext(Dispatchers.IO) {
        require(year in 1901..2099) { "year fuera de rango: $year" }

        // ---------- 1) Cache primero ----------
        val cached = cache.load(lat, lon)
        if (cached != null) {
            val series = buildSeries(cached, year)
            return@withContext FetchResult.Success(
                series = series,
                fromCache = true,
                metadata = cached.toMetadata()
            )
        }

        // ---------- 2) Descarga ----------
        Log.i(TAG, "Cache vacío para lat=$lat lon=$lon — descargando de PVGIS")
        val json = try {
            TmyHttpClient.downloadTmyJson(lat, lon)
        } catch (t: Throwable) {
            Log.e(TAG, "Descarga PVGIS falló para lat=$lat lon=$lon", t)
            return@withContext FetchResult.Failure(
                reason = "No se pudo descargar el TMY desde PVGIS. " +
                        "Verificá tu conexión a internet.",
                cause = t
            )
        }

        // ---------- 3) Parseo ----------
        val parsed = try {
            TmyJsonParser.parse(json)
        } catch (t: Throwable) {
            Log.e(TAG, "Parseo del JSON PVGIS falló", t)
            return@withContext FetchResult.Failure(
                reason = "El TMY descargado tiene un formato inesperado. " +
                        "Probá de nuevo más tarde.",
                cause = t
            )
        }

        // ---------- 4) Guardar cache (best effort) ----------
        cache.save(lat, lon, parsed)

        // ---------- 5) Construir serie ----------
        val series = buildSeries(parsed, year)
        FetchResult.Success(
            series = series,
            fromCache = false,
            metadata = parsed.toMetadata()
        )
    }

    /** Para diagnóstico/UI: ¿hay cache disponible para este sitio? */
    fun hasCache(lat: Double, lon: Double): Boolean = cache.has(lat, lon)

    /** Para tests/debugging: invalida el cache de un sitio. */
    fun deleteCache(lat: Double, lon: Double): Boolean = cache.delete(lat, lon)

    // ============================================================
    // Helpers privados
    // ============================================================

    private fun buildSeries(
        parsed: TmyParsedData.ParsedTmy,
        year: Int
    ): HourlyStepIrradianceSeries {
        val yearStartEpochUtc = LocalDateTime.of(year, 1, 1, 0, 0)
            .toInstant(ZoneOffset.UTC)
            .toEpochMilli()
        return HourlyStepIrradianceSeries(
            hourlyData = parsed.hourly,
            yearStartEpochUtc = yearStartEpochUtc
        )
    }

    private fun TmyParsedData.ParsedTmy.toMetadata(): Metadata = Metadata(
        sourceLatitudeDeg = latitudeDeg,
        sourceLongitudeDeg = longitudeDeg,
        elevationM = elevationM,
        monthSources = monthSources
    )

    companion object {
        private const val TAG = "PvgisTmy"
    }
}