package com.example.sunproject.domain.loss.irradiance

import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL
import java.util.Locale
import javax.net.ssl.HttpsURLConnection

/**
 * Cliente HTTP minimalista para descargar el TMY de PVGIS en formato JSON.
 *
 * Sin librerías externas (HttpsURLConnection de la stdlib Android).
 * Política: hasta 3 reintentos con backoff exponencial (1s, 2s, 4s),
 * timeouts de 15 segundos cada uno (conexión y lectura).
 *
 * Devuelve el JSON crudo como String (UTF-8) en caso de éxito, o lanza
 * [IOException] con mensaje descriptivo si todos los intentos fallan.
 *
 * Endpoint utilizado:
 *   https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=X&lon=Y&outputformat=json
 *
 * Documentación oficial:
 *   https://joint-research-centre.ec.europa.eu/photovoltaic-geographical-information-system-pvgis/
 *
 * El servicio PVGIS es público y gratuito, no requiere API key.
 */
internal object TmyHttpClient {

    private const val TAG = "PvgisTmy.Http"
    private const val ENDPOINT_TEMPLATE =
        "https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=%s&lon=%s&outputformat=json"
    private const val CONNECT_TIMEOUT_MS = 15_000
    private const val READ_TIMEOUT_MS = 15_000
    private const val MAX_ATTEMPTS = 3
    private const val USER_AGENT = "SunProject/1.0 (Android; academic; PV shading analysis)"

    /**
     * Descarga el TMY para las coordenadas dadas.
     *
     * @param lat latitud en grados decimales (negativa para hemisferio sur).
     * @param lon longitud en grados decimales (negativa para oeste de Greenwich).
     * @return JSON crudo de PVGIS-ERA5 TMY como String UTF-8.
     * @throws IOException si todos los reintentos fallan.
     */
    suspend fun downloadTmyJson(lat: Double, lon: Double): String =
        withContext(Dispatchers.IO) {
            val url = buildUrl(lat, lon)
            Log.i(TAG, "GET $url")

            var lastError: Throwable? = null
            for (attempt in 1..MAX_ATTEMPTS) {
                try {
                    val json = fetchOnce(url)
                    Log.i(TAG, "Descarga OK en intento $attempt (${json.length} bytes)")
                    return@withContext json
                } catch (t: Throwable) {
                    lastError = t
                    Log.w(TAG, "Intento $attempt/$MAX_ATTEMPTS falló: ${t.message}")
                    if (attempt < MAX_ATTEMPTS) {
                        // Backoff exponencial: 1s, 2s, 4s
                        val backoffMs = 1000L * (1L shl (attempt - 1))
                        delay(backoffMs)
                    }
                }
            }

            throw IOException(
                "Descarga PVGIS falló tras $MAX_ATTEMPTS intentos: ${lastError?.message}",
                lastError
            )
        }

    private fun buildUrl(lat: Double, lon: Double): String {
        // Locale.US para asegurar punto decimal independiente del locale del
        // dispositivo. PVGIS espera decimales con punto, una coma rompe la URL.
        val latStr = String.format(Locale.US, "%.5f", lat)
        val lonStr = String.format(Locale.US, "%.5f", lon)
        return String.format(Locale.US, ENDPOINT_TEMPLATE, latStr, lonStr)
    }

    private fun fetchOnce(urlString: String): String {
        val url = URL(urlString)
        val conn = (url.openConnection() as HttpsURLConnection).apply {
            requestMethod = "GET"
            connectTimeout = CONNECT_TIMEOUT_MS
            readTimeout = READ_TIMEOUT_MS
            setRequestProperty("User-Agent", USER_AGENT)
            setRequestProperty("Accept", "application/json")
            // PVGIS comprime con gzip si lo aceptamos. Pedimos identity para
            // no manejar el stream comprimido (negligible para este tamaño).
            setRequestProperty("Accept-Encoding", "identity")
        }

        try {
            val code = conn.responseCode
            if (code != HttpURLConnection.HTTP_OK) {
                // Leer body de error si está disponible para diagnóstico.
                val errBody = try {
                    conn.errorStream?.bufferedReader()?.use { it.readText() }?.take(500)
                } catch (_: Throwable) { null }
                throw IOException(
                    "PVGIS respondió HTTP $code: ${conn.responseMessage}" +
                            (errBody?.let { " | body=$it" } ?: "")
                )
            }
            val body = conn.inputStream.bufferedReader(Charsets.UTF_8).use { it.readText() }
            if (body.isBlank()) throw IOException("Body vacío del servidor PVGIS")
            return body
        } finally {
            conn.disconnect()
        }
    }
}