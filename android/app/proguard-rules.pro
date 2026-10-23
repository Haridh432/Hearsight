# Proguard rules for Hearsight

# Keep Flutter wrapper and engine
-keep class io.flutter.** { *; }

# Keep all project application and voice classes (JNI reflection, callbacks, receivers)
-keep class com.hsv2.hsv2_lite.** { *; }
-keep interface com.hsv2.hsv2_lite.** { *; }
-keep class com.sailens.** { *; }
-keep interface com.sailens.** { *; }

# Keep native methods and classes referenced by native code
-keepclasseswithmembernames class * {
    native <methods>;
}

# Keep Sherpa-ONNX JNI classes
-keep class com.k2fsa.sherpa.onnx.** { *; }
-keep interface com.k2fsa.sherpa.onnx.** { *; }

# Keep Moonshine ASR JNI classes
-keep class ai.moonshine.** { *; }
-keep interface ai.moonshine.** { *; }

# Keep LiteRT-LM (Gemma) JNI classes
-keep class com.google.ai.edge.litertlm.** { *; }
-keep interface com.google.ai.edge.litertlm.** { *; }

# Keep Gson models and serialization
-keepattributes Signature
-keepattributes *Annotation*
-keep class com.google.gson.** { *; }
