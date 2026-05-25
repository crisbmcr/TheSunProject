package com.example.sunproject.domain.loss.irradiance

/**
 * Tipos compartidos del flujo TMY, agnósticos al formato origen.
 *
 * Separar estos tipos del parser específico ([TmyJsonParser]) permite
 * que el resto del pipeline ([TmyCacheStore], [PvgisTmyDataSource]) no
 * dependa del formato concreto de PVGIS. Si en el futuro se agregara
 * soporte para otra fuente (NASA POWER, Solargis, archivo local CSV),
 * solo cambiaría el parser; cache y data source siguen sin tocar.
 *
 * Estos tipos están en su propio archivo (no anidados en el parser)
 * exactamente por eso: para no acoplar el almacenamiento al formato
 * de descarga.
 */
internal object TmyParsedData {

    /** Cantidad de muestras horarias en un año no bisiesto. */
    const val HOURS_PER_YEAR = 8760

    /**
     * Resultado de parsear un dataset TMY desde su formato origen.
     *
     * @property hourly array de [IrradianceSample]s, exactamente
     *   [HOURS_PER_YEAR] elementos en orden cronológico desde el 1 de
     *   enero a las 00:00 UTC.
     * @property latitudeDeg latitud del sitio reportada por la fuente.
     * @property longitudeDeg longitud del sitio reportada por la fuente.
     * @property elevationM elevación del sitio en metros, si la fuente la
     *   reporta. PVGIS la incluye siempre.
     * @property irradianceTimeOffsetH offset temporal de las muestras
     *   respecto al timestamp nominal (PVGIS usa 0.5 = el valor de "HH:00"
     *   representa la media centrada en "HH:30"). Informativo;
     *   [HourlyStepIrradianceSeries] resuelve la indexación step-wise
     *   independientemente del offset.
     * @property monthSources qué año real provee cada mes calendario del
     *   TMY. PVGIS construye el TMY combinando meses de distintos años
     *   (ej. enero de 2020, febrero de 2017, ...).
     */
    data class ParsedTmy(
        val hourly: Array<IrradianceSample>,
        val latitudeDeg: Double,
        val longitudeDeg: Double,
        val elevationM: Double?,
        val irradianceTimeOffsetH: Double?,
        val monthSources: List<MonthSource>
    ) {
        // equals/hashCode manuales por presencia de Array. data class por
        // defecto usaría identidad para Arrays, lo que falsea igualdad.
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other !is ParsedTmy) return false
            if (!hourly.contentEquals(other.hourly)) return false
            if (latitudeDeg != other.latitudeDeg) return false
            if (longitudeDeg != other.longitudeDeg) return false
            if (elevationM != other.elevationM) return false
            if (irradianceTimeOffsetH != other.irradianceTimeOffsetH) return false
            if (monthSources != other.monthSources) return false
            return true
        }

        override fun hashCode(): Int {
            var result = hourly.contentHashCode()
            result = 31 * result + latitudeDeg.hashCode()
            result = 31 * result + longitudeDeg.hashCode()
            result = 31 * result + (elevationM?.hashCode() ?: 0)
            result = 31 * result + (irradianceTimeOffsetH?.hashCode() ?: 0)
            result = 31 * result + monthSources.hashCode()
            return result
        }
    }

    /**
     * Origen real (año) de un mes calendario del TMY. PVGIS reporta esto
     * en `outputs.months_selected[]`.
     */
    data class MonthSource(val month: Int, val year: Int)
}