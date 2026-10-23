plugins {
    id("com.android.application")
    // The Flutter Gradle Plugin must be applied after the Android and Kotlin Gradle plugins.
    id("dev.flutter.flutter-gradle-plugin")
}

android {
    packaging {
        jniLibs {
            useLegacyPackaging = true
            excludes += listOf("lib/armeabi-v7a/**", "lib/x86/**", "lib/x86_64/**")
        }
    }

    namespace = "com.hsv2.hsv2_lite"

    compileSdk = flutter.compileSdkVersion

    // Use the already-installed NDK version.
    ndkVersion = "28.2.13676358"
    
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    defaultConfig {
        applicationId = "com.hsv2.hsv2_lite"

        // Sailens output (VibratorManager, accessibility vibration) needs API 31.
        minSdk = maxOf(flutter.minSdkVersion, 31)
        targetSdk = flutter.targetSdkVersion
        versionCode = flutter.versionCode
        versionName = flutter.versionName

        ndk {
            abiFilters.clear()
            abiFilters.add("arm64-v8a")
        }
    }

    buildTypes {
        release {
            signingConfig = signingConfigs.getByName("debug")
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
        }
    }
}

kotlin {
    compilerOptions {
        jvmTarget = org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17
    }
}

flutter {
    source = "../.."
}

dependencies {
    // Sailens SpeechManager / AccessibilityStatusProvider expose StateFlow.
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.9.0")
    // "What's ahead?": Gemma 4 E2B on the phone (LiteRT-LM, Apache-2.0).
    implementation("com.google.ai.edge.litertlm:litertlm-android:0.18.0")
    implementation("ai.moonshine:moonshine-voice:0.1.5")
    implementation("com.k2fsa.sherpa.onnx:sherpa-onnx-static:1.13.8")
    implementation("com.google.code.gson:gson:2.14.0")
}
