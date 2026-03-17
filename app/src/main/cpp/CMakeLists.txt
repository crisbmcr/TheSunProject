cmake_minimum_required(VERSION 3.18.1)
project("panorama-lib")

add_library(panorama-lib SHARED native-lib.cpp)

# OPENCV_SDK llega desde Gradle: -DOPENCV_SDK=...
set(OPENCV_INC    "${OPENCV_SDK}/native/jni/include")
set(OPENCV_STATIC "${OPENCV_SDK}/native/staticlibs/${ANDROID_ABI}")
set(OPENCV_3P     "${OPENCV_SDK}/native/3rdparty/libs/${ANDROID_ABI}")

target_include_directories(panorama-lib PRIVATE "${OPENCV_INC}")
target_compile_options(panorama-lib PRIVATE -fexceptions -frtti)
target_compile_features(panorama-lib PRIVATE cxx_std_17)

find_library(LOG_LIB log)
find_library(ANDROID_LIB android)
find_library(JNIGRAPHICS_LIB jnigraphics)
find_library(Z_LIB z)
find_library(M_LIB m)
find_library(CPP_SHARED_LIB c++_shared)

# Atomic: opcional (en arm64 muchas veces no existe)
find_library(ATOMIC_LIB atomic)

target_link_libraries(panorama-lib
        # --- OpenCV módulos (estáticos) ---
        "${OPENCV_STATIC}/libopencv_stitching.a"
        "${OPENCV_STATIC}/libopencv_photo.a"
        "${OPENCV_STATIC}/libopencv_calib3d.a"
        "${OPENCV_STATIC}/libopencv_features2d.a"
        "${OPENCV_STATIC}/libopencv_flann.a"
        "${OPENCV_STATIC}/libopencv_imgproc.a"
        "${OPENCV_STATIC}/libopencv_imgcodecs.a"
        "${OPENCV_STATIC}/libopencv_core.a"

        # --- 3rdparty ---
        "${OPENCV_3P}/liblibjpeg-turbo.a"
        "${OPENCV_3P}/liblibpng.a"
        "${OPENCV_3P}/liblibtiff.a"
        "${OPENCV_3P}/liblibwebp.a"
        "${OPENCV_3P}/liblibopenjp2.a"
        "${OPENCV_3P}/libIlmImf.a"
        "${OPENCV_3P}/libtbb.a"
        "${OPENCV_3P}/libittnotify.a"
        "${OPENCV_3P}/libcpufeatures.a"
        "${OPENCV_3P}/libtegra_hal.a"

        # --- sistema ---
        ${Z_LIB}
        ${M_LIB}
        ${LOG_LIB}
        ${ANDROID_LIB}
        ${JNIGRAPHICS_LIB}
        ${CPP_SHARED_LIB}
)

if(ATOMIC_LIB)
    target_link_libraries(panorama-lib ${ATOMIC_LIB})
endif()
