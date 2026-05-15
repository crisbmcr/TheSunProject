package com.example.sunproject.diagnostics

import android.util.Log
import java.io.BufferedReader
import java.io.File
import java.io.FileWriter
import java.io.InputStreamReader
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Captura logs de tags específicos a un archivo, para validar afuera sin
 * tener que mantener depuración wifi/USB conectada.
 *
 * Tags capturados (hardcoded para el debug de norte verdadero, sesión
 * 2026-05-14):
 *   - SunDeclination       (CaptureActivity, declinación al iniciar sesión)
 *   - AtlasBuildUseCase    (AtlasBuildUseCase, declinación inyectada al projector)
 *   - ZenithMatrix         (ZenithMatrixProjector, corrección aplicada al Z0)
 *
 * Mecanismo: spawnea un proceso `logcat` hijo con filtros de tag y bombea
 * su stdout línea por línea a un archivo. Una app puede leer su propio
 * logcat sin permisos especiales, por lo que el proceso hijo hereda el
 * UID y solo ve los logs de SunProject.
 *
 * Salida: /Android/data/com.example.sunproject/files/logs/sun_log_<ts>.txt
 * Accesible desde el explorador de archivos del celular sin root, o
 * conectando por USB y navegando a esa ruta.
 *
 * Cada arranque de la app crea un archivo nuevo (timestamped). Si querés
 * compartir el archivo después de la prueba, navegá hasta esa carpeta y
 * usá "Compartir" desde el explorador.
 */
object FileLogger {

    private const val TAG = "FileLogger"

    private val CAPTURED_TAGS = listOf(
        "SunDeclination",
        "AtlasBuildUseCase",
        "ZenithMatrix",
        "AtlasPose",            // yaws crudos de cada H0/H45 proyectado
        "AtlasDeclCorrection"   // setSessionDeclinationDeg del projector
    )

    @Volatile private var logcatProcess: Process? = null
    @Volatile private var pumpThread: Thread? = null
    @Volatile private var currentLogFile: File? = null
    @Volatile private var currentSessionDir: File? = null

    /**
     * Arranca la captura para una sesión concreta. El archivo de log
     * queda en <sessionDir>/logs/sun_log_<timestamp>.txt, autocontenido
     * con frames.json, session.json y atlas/.
     *
     * Idempotente por sessionDir: si ya está corriendo apuntando a la
     * misma sesión, no hace nada. Si está corriendo apuntando a OTRA
     * sesión (el usuario inició una nueva captura), para el logger
     * anterior y arranca uno nuevo en el directorio nuevo. Esto evita
     * que logs de sesiones distintas se mezclen.
     */
    fun start(sessionDir: File) {
        val existing = currentSessionDir
        if (logcatProcess != null && existing != null && existing.absolutePath == sessionDir.absolutePath) {
            Log.i(TAG, "ya activo en sessionDir=${sessionDir.name}")
            return
        }
        // Si había uno corriendo para otra sesión, lo paramos limpio antes.
        if (logcatProcess != null) {
            Log.i(TAG, "rotando: stop sesión vieja=${existing?.name}, start nueva=${sessionDir.name}")
            stop()
        }

        val logsDir = File(sessionDir, "logs").apply { mkdirs() }
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val logFile = File(logsDir, "sun_log_$stamp.txt")
        currentLogFile = logFile
        currentSessionDir = sessionDir

        // Comando logcat:
        //   -v threadtime  → formato con timestamp/PID/TID
        //   TAG:V          → nivel Verbose (todos) para cada tag de interés
        //   *:S            → silencia todo el resto (Silent)
        val cmd = mutableListOf("logcat", "-v", "threadtime")
        CAPTURED_TAGS.forEach { cmd += "$it:V" }
        cmd += "*:S"

        val proc = try {
            ProcessBuilder(cmd).redirectErrorStream(true).start()
        } catch (t: Throwable) {
            Log.e(TAG, "no pude arrancar logcat", t)
            return
        }
        logcatProcess = proc

        // Thread que bombea stdout del logcat hijo al archivo. Daemon para
        // que no bloquee el shutdown de la app. Flush por línea para que
        // los datos sobrevivan un crash o un kill del proceso.
        val thread = Thread({
            try {
                BufferedReader(InputStreamReader(proc.inputStream)).use { reader ->
                    FileWriter(logFile, true).use { writer ->
                        writer.write("=== SunProject FileLogger ===\n")
                        writer.write("started=$stamp\n")
                        writer.write("tags=${CAPTURED_TAGS.joinToString(",")}\n")
                        writer.write("============================\n")
                        writer.flush()

                        var line: String?
                        while (reader.readLine().also { line = it } != null) {
                            writer.write(line)
                            writer.write("\n")
                            writer.flush()
                        }
                    }
                }
            } catch (t: Throwable) {
                Log.w(TAG, "pump thread terminó", t)
            }
        }, "FileLogger-Pump").apply {
            isDaemon = true
            start()
        }
        pumpThread = thread

        Log.i(TAG, "arrancado → ${logFile.absolutePath}")
    }

    /** Para casos donde quieras cerrar el logger explícitamente. */
    fun stop() {
        logcatProcess?.destroy()
        logcatProcess = null
        pumpThread = null
        currentSessionDir = null
        Log.i(TAG, "detenido")
    }

    /** Ruta absoluta del archivo de log actual, o null si no está activo. */
    fun currentLogPath(): String? = currentLogFile?.absolutePath
}