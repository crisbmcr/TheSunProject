package com.example.sunproject.data.model

data class FrameRecord(

    val frameId: String,

    val sessionId: String,

    val ringId: String,

    val shotIndex: Int,

    val originalPath: String,
    val capturedAtUtcMs: Long,

    val targetAzimuthDeg: Float,

    val targetPitchDeg: Float,

    // Pose display / UI
    val measuredAzimuthDeg: Float,

    val measuredPitchDeg: Float,

    val measuredRollDeg: Float,

    // Pose absoluta derivada del mismo TYPE_ROTATION_VECTOR
    val absAzimuthDeg: Float? = null,

    val absPitchDeg: Float? = null,

    val absRollDeg: Float? = null,

    // ============================================================
    // Matriz device->world (R_world←device) — fuente: TYPE_ROTATION_VECTOR
    // ============================================================
    // Esta es la matriz "histórica" del proyecto, persistida desde el
    // primer día. Para H0/H45 viene de SensorManager.getRotationMatrixFromVector
    // sobre el evento de TYPE_ROTATION_VECTOR. Para Z0 puede venir de la
    // misma fuente o de gravity+mag según el flag USE_GRAV_MAG_Z0_MATRIX.
    //
    // El frame de referencia es ENU magnético calibrado por Android (incluye
    // la calibración hard-iron/soft-iron interna). Las H0/H45 viven SIEMPRE
    // en este frame.
    //
    // Si en el futuro se quiere distinguir explícitamente la fuente, considerar
    // renombrar a `rotationRotvecM00..M22`. Por ahora se mantiene el nombre
    // viejo por compatibilidad con archivos JSON ya escritos.
    val rotationM00: Float? = null,

    val rotationM01: Float? = null,

    val rotationM02: Float? = null,

    val rotationM10: Float? = null,

    val rotationM11: Float? = null,

    val rotationM12: Float? = null,

    val rotationM20: Float? = null,

    val rotationM21: Float? = null,

    val rotationM22: Float? = null,

    // ============================================================
    // Matriz device->world (R_world←device) — fuente: gravity + magnetic
    // ============================================================
    // Construida vía SensorManager.getRotationMatrix(R, null, gravity, mag)
    // sobre los buffers promediados de TYPE_GRAVITY y TYPE_MAGNETIC_FIELD
    // crudos, en el momento del disparo. Algoritmo TRIAD/Wahba 2-vectores,
    // sin filtros internos de Android.
    //
    // El frame de referencia es ENU magnético crudo (sin la calibración
    // hard-iron/soft-iron que TYPE_ROTATION_VECTOR aplica internamente).
    // Por eso difiere ligeramente del frame de las matrices rotationM**:
    // existe una rotación de bias, casi constante durante una sesión,
    // entre los dos sistemas.
    //
    // Estos campos se completan SI Y SOLO SI:
    //   - Hay buffers con suficientes samples (≥15) en el momento del disparo.
    //   - magneticAccuracy ≥ MEDIUM.
    //   - Norma de gravedad ≈ 9.81 (celular quieto).
    //   - Norma magnética en [20, 80] µT.
    // Si cualquiera de las validaciones falla, los 9 campos quedan null.
    //
    // USO: la fix de Fase 5 usa estos campos para construir la matriz del
    // Z0. La fix de Fase 6 (bias correction) los usa también para H0/H45,
    // calcula R_bias = R_rotvec · R_gravmag^T y promedia sobre los frames
    // no-zenitales.
    val rotationGravMagM00: Float? = null,

    val rotationGravMagM01: Float? = null,

    val rotationGravMagM02: Float? = null,

    val rotationGravMagM10: Float? = null,

    val rotationGravMagM11: Float? = null,

    val rotationGravMagM12: Float? = null,

    val rotationGravMagM20: Float? = null,

    val rotationGravMagM21: Float? = null,

    val rotationGravMagM22: Float? = null,

    val latitudeDeg: Double?,

    val longitudeDeg: Double?,

    val altitudeM: Double?,

    val imageWidthPx: Int?,

    val imageHeightPx: Int?,

    val hfovDeg: Float?,

    val vfovDeg: Float?
)