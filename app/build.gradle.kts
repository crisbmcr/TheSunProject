val opencvSdk = "${rootProject.projectDir}/third_party/opencv/sdk"

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
}

android {
    namespace = "com.example.sunproject"
    compileSdk = 35
    sourceSets["main"].jniLibs.srcDirs("$opencvSdk/native/libs")

    defaultConfig {
        applicationId = "com.example.sunproject"
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        ndk {
            abiFilters += listOf("arm64-v8a")
        }

        // 👇 AQUÍ van las flags y argumentos de CMake
        externalNativeBuild {
            cmake {
                cppFlags += listOf("-std=c++17", "-O3")
                arguments += listOf(
                    "-DANDROID_STL=c++_shared",
                    "-DOPENCV_SDK=$opencvSdk"     //
                )
            }
        }
    }

    // 👇 AQUÍ solo la ruta al CMakeLists.txt
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }

    buildFeatures {
        compose = true
        viewBinding = true
        prefab = false
    }
}
repositories {
    //maven { url = uri("https://jitpack.io") }
}

configurations.all {
    resolutionStrategy {
        force("com.google.ar:core:1.42.0")
    }
}

dependencies {
    // --- DEPENDENCIA AL MÓDULO DE OPENCV ---
    implementation("org.opencv:opencv:4.10.0")

    implementation("org.apache.commons:commons-math3:3.6.1")
    // --- DEPENDENCIAS EXISTENTES ---
    implementation(libs.androidx.core.ktx)
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.12.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("junit:junit:4.13.2")
    implementation("androidx.test.ext:junit:1.2.1")
    implementation("androidx.test.espresso:espresso-core:3.6.1")

    // --- DEPENDENCIAS DE ARCORE / SCENEFORM ---
    implementation("com.google.ar:core:1.42.0")
    implementation("com.gorisse.thomas.sceneform:sceneform:1.21.0")
    implementation("com.gorisse.thomas.sceneform:ux:1.21.0")

    // --- DEPENDENCIA DE UBICACIÓN (GPS) ---
    implementation("com.google.android.gms:play-services-location:21.3.0")

    // --- ¡CORREGIDAS! DEPENDENCIAS PARA CAMERAX CON SINTAXIS KOTLIN ---
    val camerax_version = "1.3.4" // Se usa 'val' en lugar de 'def'
    implementation("androidx.camera:camera-core:${camerax_version}")
    implementation("androidx.camera:camera-camera2:${camerax_version}")
    implementation("androidx.camera:camera-lifecycle:${camerax_version}")
    implementation("androidx.camera:camera-view:${camerax_version}")


    // --- DEPENDENCIAS DE COMPOSE ---
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.activity.compose)
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.ui)
    implementation(libs.androidx.ui.graphics)
    implementation(libs.androidx.ui.tooling.preview)
    implementation(libs.androidx.material3)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.ui.test.junit4)
    debugImplementation(libs.androidx.ui.tooling)
    debugImplementation(libs.androidx.ui.test.manifest)

}
