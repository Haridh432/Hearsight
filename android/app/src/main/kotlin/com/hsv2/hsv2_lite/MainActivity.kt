package com.hsv2.hsv2_lite

import java.io.File
import java.io.FileOutputStream

import android.Manifest
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.annotation.Keep
import androidx.annotation.NonNull
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel
import org.json.JSONObject
import com.hsv2.hsv2_lite.voice.VoiceAssistantManager
import com.hsv2.hsv2_lite.voice.VoiceCommandProcessor

class MainActivity : FlutterActivity() {

    // Phone IMU stands in for the D455 IMU (off by default, see native-lib).
    private val phoneImu by lazy { PhoneImu(this) }
    // Phase 6: voice + haptic guidance (Sailens output).
    private val voiceGuide by lazy { VoiceGuide(this).also { it.start(); it.onSay = { k, t -> validation.said(k, t) } } }
    // "What's ahead?": Gemma 4 E2B scene description on the phone (LiteRT-LM).
    private val sceneDescriber by lazy { SceneDescriber(this) }
    @Volatile private var lastTelemetry: JSONObject? = null
    // Phase 7: per-scenario CSV of telemetry + speech while a validation test runs.
    private val validation by lazy { ValidationLogger(this) }

    // Offline Voice Assistant (Argus integration)
    private val voiceCommandProcessor by lazy {
        VoiceCommandProcessor(
            onStartNavigation = {
                val err = try {
                    startPipeline()
                } catch (e: Exception) {
                    Log.e(LOG_TAG, "Voice startPipeline failed", e)
                    e.message ?: "Failed to start camera pipeline"
                }
                if (err.isEmpty()) {
                    voiceGuide.onNavigationStarted()
                    if (sceneDescriber.modelAvailable) sceneDescriber.warmUp()
                    runOnUiThread { window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) }
                    null
                } else {
                    err
                }
            },
            onStopNavigation = {
                try {
                    stopPipeline()
                } catch (e: Exception) {
                    Log.e(LOG_TAG, "Voice stopPipeline failed", e)
                }
                validation.stop()
                voiceGuide.onNavigationStopped()
                runOnUiThread { window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) }
            },
            onDescribeSurroundings = { q ->
                runOnUiThread {
                    describeScene { text ->
                        Log.i(LOG_TAG, "Voice scene describe result: $text")
                    }
                }
            },
            getTelemetry = { lastTelemetry }
        )
    }

    private val voiceAssistantManager by lazy {
        VoiceAssistantManager(
            context = this,
            commandProcessor = voiceCommandProcessor,
            onSayResponse = { text ->
                voiceGuide.sayAssistantResponse(text)
            }
        )
    }

    private var hasAutoStartedVoice = false

    private fun checkPermissionsAndStartVoice(greet: Boolean = true) {
        val missing = REQUIRED_PERMISSIONS.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            Log.i(LOG_TAG, "Requesting runtime permissions: $missing")
            ActivityCompat.requestPermissions(this, missing.toTypedArray(), PERMISSION_REQUEST_CODE)
        } else {
            if (!hasAutoStartedVoice) {
                hasAutoStartedVoice = true
                Log.i(LOG_TAG, "Permissions already granted, starting voice interaction (greet=$greet)")
                voiceGuide.whenReady {
                    if (greet) {
                        voiceAssistantManager.startInteraction()
                    } else {
                        voiceAssistantManager.startListening()
                    }
                }
            }
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_CODE) {
            val audioIdx = permissions.indexOf(Manifest.permission.RECORD_AUDIO)
            if (audioIdx != -1 && grantResults.getOrNull(audioIdx) == PackageManager.PERMISSION_GRANTED) {
                Log.i(LOG_TAG, "RECORD_AUDIO granted, starting voice assistant interaction")
                hasAutoStartedVoice = true
                voiceGuide.whenReady {
                    voiceAssistantManager.startInteraction()
                }
            } else {
                Log.w(LOG_TAG, "RECORD_AUDIO permission was not granted by user")
            }
        }
    }

    override fun onResume() {
        super.onResume()
        phoneImu.start()
        checkPermissionsAndStartVoice(greet = true)
    }

    override fun onPause() {
        phoneImu.stop()
        super.onPause()
    }

    private val COMMAND_CHANNEL = "com.hsv2/command"
    private val TELEMETRY_CHANNEL = "com.hsv2/telemetry"
    private val USB_STATUS_CHANNEL = "com.hsv2/usb_status"

    private val ACTION_USB_PERMISSION =
        "com.hsv2.hsv2_lite.USB_PERMISSION"

    // ============================================================
    // Flutter SurfaceTextures
    // ============================================================

    private var rgbTextureEntry:
            io.flutter.view.TextureRegistry.SurfaceTextureEntry? = null

    private var rgbSurface:
            android.view.Surface? = null

    private var depthTextureEntry:
            io.flutter.view.TextureRegistry.SurfaceTextureEntry? = null

    private var depthSurface:
            android.view.Surface? = null

    // ============================================================
    // Event / polling state
    // ============================================================

    private var eventSink:
            EventChannel.EventSink? = null

    private var usbStatusSink:
            EventChannel.EventSink? = null

    private val handler =
            Handler(Looper.getMainLooper())

    private var isPolling = false

    private lateinit var usbManager: UsbManager

    // ============================================================
    // Native library
    // ============================================================

    companion object {
        private const val LOG_TAG = "HSV2Lite"
        private const val PERMISSION_REQUEST_CODE = 2001
        private val REQUIRED_PERMISSIONS = arrayOf(
            Manifest.permission.RECORD_AUDIO,
            Manifest.permission.CAMERA
        )

        @Volatile
        var nativeLoaded = false
            private set

        init {
            try {
                System.loadLibrary("realsense2")
                Log.i(LOG_TAG, "Native realsense2 library loaded successfully")
            } catch (t: Throwable) {
                Log.w(LOG_TAG, "realsense2 load: ${t.message}")
            }
            try {
                System.loadLibrary("hsv2_lite")
                nativeLoaded = true
                Log.i(
                    LOG_TAG,
                    "Native hsv2_lite library loaded successfully"
                )
            } catch (t: Throwable) {
                Log.e(
                    LOG_TAG,
                    "Failed to load hsv2_lite native library: ${t.message}",
                    t
                )
            }
        }
    }

    // ============================================================
    // Native functions
    // ============================================================

    @Keep
    /** Returns "" on success, otherwise the error message. */
    external fun startPipeline(): String

    @Keep
    external fun stopPipeline()

    @Keep
    external fun setStage(stage: Int)

    @Keep
    external fun pollTelemetry(): String

    // Latest colour frame [w, h (int32 LE), RGB...] for the scene describer, or null.
    external fun getLatestColor(): ByteArray?

    @Keep
    private external fun initRealSenseWithFD(
        fd: Int,
        usbfsPath: String
    ): Boolean

    @Keep
    external fun setNativeWindow(
        surface: android.view.Surface
    )

    @Keep
    external fun releaseNativeWindow()

    @Keep
    external fun setDepthNativeWindow(
        surface: android.view.Surface
    )

    @Keep
    external fun releaseDepthNativeWindow()

    // ============================================================
    // QNN initialization
    // ============================================================

    // Scene segmentation (DeepLabV3+ ADE20K); runs on its own native thread.
    @Keep
    private external fun initSemSeg(
        contextBinaryPath: String,
        nativeLibDir: String
    ): Boolean

    // 0 = camera only, 1 = camera + segmentation overlay.
    @Keep
    external fun setViewMode(mode: Int)

    // Walk recorder: files go to /sdcard/Android/data/<pkg>/files/rec/.
    @Keep
    external fun startRecording(dir: String)

    @Keep
    external fun stopRecording()

    @Keep
    private external fun initQnn(
        contextBinaryPath: String,
        nativeLibDir: String
    ): Boolean

    // ============================================================
    // USB Broadcast Receiver
    // ============================================================

    private val usbReceiver =
        object : BroadcastReceiver() {

            override fun onReceive(
                context: Context,
                intent: Intent
            ) {

                val action = intent.action

                if (ACTION_USB_PERMISSION == action) {

                    synchronized(this) {

                        val device: UsbDevice? =
                            intent.getParcelableExtra(
                                UsbManager.EXTRA_DEVICE
                            )

                        if (
                            intent.getBooleanExtra(
                                UsbManager.EXTRA_PERMISSION_GRANTED,
                                false
                            )
                        ) {

                            device?.apply {

                                Log.i(
                                    LOG_TAG,
                                    "USB Permission granted for device: " +
                                            device.deviceName
                                )

                                connectToDevice(this)
                            }

                        } else {

                            Log.e(
                                LOG_TAG,
                                "USB Permission denied for device $device"
                            )

                            notifyUsbStatus(
                                "PERMISSION DENIED"
                            )
                        }
                    }

                } else if (
                    UsbManager.ACTION_USB_DEVICE_ATTACHED ==
                    action
                ) {

                    Log.i(
                        LOG_TAG,
                        "USB Device Attached"
                    )

                    checkUsbDevices()

                } else if (
                    UsbManager.ACTION_USB_DEVICE_DETACHED ==
                    action
                ) {

                    Log.i(
                        LOG_TAG,
                        "USB Device Detached"
                    )

                    Log.i(LOG_TAG, "STOP requested: USB device detached")
                    // Sensor health: camera lost while running -> long STOP
                    // vibration (the stream stops, so no telemetry follows).
                    validation.event("camera_detached")
                    if (android.os.SystemClock.elapsedRealtime() - lastStreamMs < 2000) voiceGuide.onCameraLost()
                    else voiceGuide.notice("Camera disconnected", "கேமரா துண்டிக்கப்பட்டது")
                    runOnUiThread { window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) }
                    // stopPipeline joins the stream thread, which can sit in
                    // wait_for_frames for up to 5 s: keep it off the UI thread.
                    Thread {
                        try {
                            stopPipeline()
                        } catch (e: Exception) {
                            Log.e(LOG_TAG, "stopPipeline on detach failed", e)
                        }
                    }.start()

                    notifyUsbStatus(
                        "DISCONNECTED"
                    )
                }
            }
        }

    // ============================================================
    // Flutter Engine
    // ============================================================

    override fun configureFlutterEngine(
        @NonNull flutterEngine: FlutterEngine
    ) {

        super.configureFlutterEngine(
            flutterEngine
        )

        usbManager =
            getSystemService(
                Context.USB_SERVICE
            ) as UsbManager

        // ========================================================
        // USB receiver (Guarded for Android 14+ SecurityException)
        // ========================================================

        try {
            val filter = IntentFilter(ACTION_USB_PERMISSION).apply {
                addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
                addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
            }

            if (Build.VERSION.SDK_INT >= 33) {
                registerReceiver(
                    usbReceiver,
                    filter,
                    Context.RECEIVER_EXPORTED
                )
            } else {
                registerReceiver(
                    usbReceiver,
                    filter
                )
            }
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "Failed to register usbReceiver: ${t.message}", t)
        }

        // Scene-description benchmark, started from the PC:
        //   adb shell am broadcast -a com.hsv2.DESCRIBE_TEST -p com.hsv2.hsv2_lite
        // Runs Gemma on up to 10 snapshots of the newest walk recording (files/rec/*.frames)
        // and writes files/scene_test.csv (image, ms, answer).
        try {
            val testFilter = IntentFilter("com.hsv2.DESCRIBE_TEST")
            if (Build.VERSION.SDK_INT >= 33) registerReceiver(describeTestReceiver, testFilter, Context.RECEIVER_EXPORTED)
            else registerReceiver(describeTestReceiver, testFilter)
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "Failed to register describeTestReceiver: ${t.message}", t)
        }

        // ========================================================
        // Prepare QNN files & Initialize QNN HTP (Guarded)
        // ========================================================

        if (nativeLoaded) {
            try {
                prepareQnnFiles()
            } catch (t: Throwable) {
                Log.e(LOG_TAG, "prepareQnnFiles failed: ${t.message}", t)
            }

            try {
                initializeQnn()
            } catch (t: Throwable) {
                Log.e(LOG_TAG, "initializeQnn failed: ${t.message}", t)
            }
        } else {
            Log.w(LOG_TAG, "Native library not loaded; skipping QNN initialization")
        }

        // ========================================================
        // Flutter Command Channel
        // ========================================================

        voiceGuide   // start the TTS engine early (it takes a moment to be ready)
        voiceAssistantManager // warm up voice assistant models early in background
        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            COMMAND_CHANNEL
        ).setMethodCallHandler { call, result ->

            when (call.method) {

                "startPipeline" -> {

                    val error =
                        try {
                            startPipeline()
                        } catch (e: Exception) {
                            Log.e(
                                LOG_TAG,
                                "Native startPipeline failed",
                                e
                            )
                            e.message ?: "startPipeline failed"
                        }

                    if (error.isEmpty()) {
                        // Accessibility: say it started, and keep the screen on while guiding
                        // (with the screen off nothing is checked -> "Camera stalled").
                        voiceGuide.onNavigationStarted()
                        if (sceneDescriber.modelAvailable) sceneDescriber.warmUp()
                        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        result.success(null)
                    } else {
                        result.error(
                            "START_FAILED",
                            error,
                            null
                        )
                    }
                }

                "stopPipeline" -> {

                    Log.i(LOG_TAG, "STOP requested by the app UI (source: ${call.argument<String>("source") ?: "button"})")
                    try {

                        stopPipeline()

                    } catch (e: Exception) {

                        Log.e(
                            LOG_TAG,
                            "Native stopPipeline failed",
                            e
                        )
                    }
                    validation.stop()   // STOP also ends a running validation test (file saved)
                    voiceGuide.onNavigationStopped()
                    window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

                    result.success(null)
                }

                "startRecording" -> {
                    try {
                        val dir = File(getExternalFilesDir(null), "rec")
                        dir.mkdirs()
                        startRecording(dir.absolutePath)
                        result.success(dir.absolutePath)
                    } catch (e: Exception) {
                        Log.e(LOG_TAG, "startRecording failed", e)
                        result.success(null)
                    }
                }

                "stopRecording" -> {
                    try { stopRecording() } catch (e: Exception) { Log.e(LOG_TAG, "stopRecording failed", e) }
                    result.success(null)
                }

                "setVoice" -> {
                    voiceGuide.configure(
                        call.argument<Boolean>("enabled") ?: true,
                        call.argument<String>("lang") ?: "en",
                    )
                    result.success(voiceGuide.statusJson().toString())
                }

                "getVoice" -> result.success(voiceGuide.statusJson().toString())

                "startVoiceAssistant" -> {
                    val greet = call.argument<Boolean>("greet") ?: false
                    if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED) {
                        if (greet) {
                            voiceAssistantManager.startInteraction()
                        } else {
                            voiceAssistantManager.startListening()
                        }
                    } else {
                        checkPermissionsAndStartVoice(greet = greet)
                    }
                    result.success(voiceAssistantManager.state.value.name)
                }

                "stopVoiceAssistant" -> {
                    voiceAssistantManager.stopListening()
                    result.success(null)
                }

                "toggleVoiceAssistant" -> {
                    voiceAssistantManager.toggleListening()
                    result.success(voiceAssistantManager.state.value.name)
                }

                "getVoiceAssistantStatus" -> {
                    result.success(voiceAssistantManager.getStatusJson().toString())
                }

                "triggerVoiceCommand" -> {
                    val text = call.argument<String>("text") ?: ""
                    val reply = voiceCommandProcessor.process(text)
                    if (reply != null) {
                        voiceGuide.sayAssistantResponse(reply)
                    }
                    result.success(reply)
                }

                "sayNotice" -> {
                    val text = call.argument<String>("text") ?: ""
                    if (text.isNotBlank()) {
                        voiceGuide.sayAssistantResponse(text)
                    }
                    result.success(null)
                }

                "describeScene" -> describeScene { text -> runOnUiThread { result.success(text) } }

                "setLiveDescribe" -> {
                    liveDescribe = call.argument<Boolean>("on") ?: false
                    scenePrefs.edit().putBoolean("live", liveDescribe).apply()
                    if (liveDescribe && sceneDescriber.modelAvailable) sceneDescriber.warmUp()
                    result.success(liveDescribe)
                }

                "getLiveDescribe" -> {
                    liveDescribe = scenePrefs.getBoolean("live", false)
                    result.success(liveDescribe)
                }

                "startScenario" -> {
                    validation.start(call.argument<String>("id") ?: "S0", call.argument<String>("label") ?: "")
                    result.success(validation.file?.absolutePath)
                }

                "stopScenario" -> {
                    val path = validation.file?.absolutePath
                    validation.stop()
                    result.success(path)
                }

                "markScenario" -> {
                    validation.event("mark:" + (call.argument<String>("note") ?: ""))
                    result.success(null)
                }

                "setViewMode" -> {
                    try {
                        setViewMode(call.argument<Int>("mode") ?: 1)
                    } catch (e: Exception) {
                        Log.e(LOG_TAG, "Native setViewMode failed", e)
                    }
                    result.success(null)
                }

                "setStage" -> {

                    val stage =
                        call.argument<Int>("stage") ?: 1

                    try {

                        setStage(stage)

                    } catch (e: Exception) {

                        Log.e(
                            LOG_TAG,
                            "Native setStage failed",
                            e
                        )
                    }

                    result.success(null)
                }

                // =================================================
                // RGB texture
                // =================================================

                "createTexture" -> {

                    rgbTextureEntry =
                        flutterEngine.renderer
                            .createSurfaceTexture()

                    val surfaceTexture =
                        rgbTextureEntry!!.surfaceTexture()

                    surfaceTexture.setDefaultBufferSize(
                        640,
                        480
                    )

                    rgbSurface =
                        android.view.Surface(
                            surfaceTexture
                        )

                    if (nativeLoaded) {
                        try {
                            setNativeWindow(
                                rgbSurface!!
                            )
                        } catch (t: Throwable) {
                            Log.e(LOG_TAG, "setNativeWindow failed: ${t.message}", t)
                        }
                    }

                    result.success(
                        rgbTextureEntry!!.id()
                    )
                }

                // =================================================
                // Depth texture
                // =================================================

                "createDepthTexture" -> {

                    depthTextureEntry =
                        flutterEngine.renderer
                            .createSurfaceTexture()

                    val surfaceTexture =
                        depthTextureEntry!!.surfaceTexture()

                    surfaceTexture.setDefaultBufferSize(
                        640,
                        480
                    )

                    depthSurface =
                        android.view.Surface(
                            surfaceTexture
                        )

                    if (nativeLoaded) {
                        try {
                            setDepthNativeWindow(
                                depthSurface!!
                            )
                        } catch (t: Throwable) {
                            Log.e(LOG_TAG, "setDepthNativeWindow failed: ${t.message}", t)
                        }
                    }

                    result.success(
                        depthTextureEntry!!.id()
                    )
                }

                // =================================================
                // USB check
                // =================================================

                "zeroImu" -> {

                    phoneImu.zero()

                    result.success(null)
                }

                "checkUsb" -> {

                    checkUsbDevices()

                    result.success(null)
                }

                else -> {

                    result.notImplemented()
                }
            }
        }

        // ========================================================
        // Telemetry Event Channel
        // ========================================================

        EventChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            TELEMETRY_CHANNEL
        ).setStreamHandler(

            object : EventChannel.StreamHandler {

                override fun onListen(
                    arguments: Any?,
                    events: EventChannel.EventSink?
                ) {

                    eventSink = events

                    isPolling = true

                    startPolling()
                }

                override fun onCancel(
                    arguments: Any?
                ) {

                    eventSink = null

                    isPolling = false
                }
            }
        )

        // ========================================================
        // USB Status Event Channel
        // ========================================================

        EventChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            USB_STATUS_CHANNEL
        ).setStreamHandler(

            object : EventChannel.StreamHandler {

                override fun onListen(
                    arguments: Any?,
                    events: EventChannel.EventSink?
                ) {

                    usbStatusSink = events

                    checkUsbDevices()
                }

                override fun onCancel(
                    arguments: Any?
                ) {

                    usbStatusSink = null
                }
            }
        )

        Log.i(
            LOG_TAG,
            "MainActivity Flutter engine configured"
        )
    }

    // ============================================================
    // QNN file preparation
    // ============================================================

    private fun prepareQnnFiles() {

        try {

            val contextFile =
                prepareQnnContextBinary()

            Log.i(
                LOG_TAG,
                "QNN context prepared: ${contextFile.absolutePath}"
            )

            if (!contextFile.exists()) {

                Log.e(
                    LOG_TAG,
                    "QNN context binary does not exist"
                )
            }

        } catch (t: Throwable) {

            Log.e(
                LOG_TAG,
                "Failed to prepare QNN files: ${t.message}",
                t
            )
        }
    }

    // ============================================================
    // Copy QNN HTP context binary from assets
    // ============================================================

    // Reads an Android system property (e.g. debug.hsv2.*); "" if unset.
    private fun systemProperty(name: String): String =
        try {
            Class.forName("android.os.SystemProperties")
                .getMethod("get", String::class.java)
                .invoke(null, name) as String
        } catch (e: Exception) {
            ""
        }

    // HTP context binaries are compiled per Hexagon arch.
    // SoC -> arch from the QAIRT 2.47 SoC table.
    private fun qnnContextAssetName(): String {
        return "${qnnModelName()}_${hexagonArch()}.bin"
    }

    private fun hexagonArch(): String {

        val soc =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
                Build.SOC_MODEL
            else
                ""

        val arch =
            when (soc) {
                "SM6650", "SM6450", "SM7550", "SM7675", "SM7750",
                "SM8550", "SM8550P" -> "v73"
                else -> "v81"
            }

        Log.i(LOG_TAG, "SoC: $soc -> QNN HTP $arch")
        return arch
    }

    private fun qnnModelName(): String {
        // YOLOv8s-seg (boxes + person masks) by default;
        // adb shell setprop debug.hsv2.model det -> detection-only model.
        val model =
            if (systemProperty("debug.hsv2.model") == "det")
                "yolov8s_int8"
            else
                "yolov8s_seg_int8"

        return model
    }

    private fun prepareQnnContextBinary(
        assetName: String = qnnContextAssetName()
    ): File {

        val qnnDir =
            File(
                filesDir,
                "qnn"
            )

        if (!qnnDir.exists()) {

            qnnDir.mkdirs()
        }

        val outputFile =
            File(
                qnnDir,
                assetName
            )

        // Re-copy after every app install/update so a new model in
        // assets replaces the cached copy.
        val appUpdatedAt =
            packageManager
                .getPackageInfo(packageName, 0)
                .lastUpdateTime

        if (
            !outputFile.exists() ||
            outputFile.length() == 0L ||
            outputFile.lastModified() < appUpdatedAt
        ) {

            Log.i(
                LOG_TAG,
                "Copying QNN context binary from assets..."
            )

            assets.open(
                "qnn/$assetName"
            ).use { input ->

                FileOutputStream(
                    outputFile
                ).use { output ->

                    val buffer =
                        ByteArray(8192)

                    var read: Int

                    while (
                        input.read(buffer)
                            .also { read = it } != -1
                    ) {

                        output.write(
                            buffer,
                            0,
                            read
                        )
                    }

                    output.flush()
                }
            }

            Log.i(
                LOG_TAG,
                "QNN context binary copied successfully"
            )

        } else {

            Log.i(
                LOG_TAG,
                "QNN context binary already exists"
            )
        }

        return outputFile
    }

    // ============================================================
    // Initialize QNN HTP
    // ============================================================

    private fun initializeQnn() {

        try {

            val contextBinary =
                prepareQnnContextBinary()

            Log.i(
                LOG_TAG,
                "----------------------------------------"
            )

            Log.i(
                LOG_TAG,
                "Starting QNN HTP initialization"
            )

            Log.i(
                LOG_TAG,
                "Context: ${contextBinary.absolutePath}"
            )

            Log.i(
                LOG_TAG,
                "Context size: ${contextBinary.length()} bytes"
            )

            if (!contextBinary.exists()) {

                Log.e(
                    LOG_TAG,
                    "QNN context binary NOT FOUND"
                )

                return
            }

            val success =
                initQnn(
                    contextBinary.absolutePath,
                    applicationInfo.nativeLibraryDir
                )

            if (success) {

                Log.i(
                    LOG_TAG,
                    "========================================"
                )

                Log.i(
                    LOG_TAG,
                    "QNN HTP INITIALIZATION SUCCESS"
                )

                Log.i(
                    LOG_TAG,
                    "========================================"
                )

                // Scene segmentation; adb shell setprop debug.hsv2.semseg 0 disables it.
                if (systemProperty("debug.hsv2.semseg") != "0") {
                    val segFile =
                        prepareQnnContextBinary("deeplab_ade_int8_${hexagonArch()}.bin")
                    val segOk =
                        initSemSeg(segFile.absolutePath, applicationInfo.nativeLibraryDir)
                    Log.i(LOG_TAG, "Scene segmentation init: ${if (segOk) "OK" else "FAILED"}")
                }

            } else {

                Log.e(
                    LOG_TAG,
                    "========================================"
                )

                Log.e(
                    LOG_TAG,
                    "QNN HTP INITIALIZATION FAILED"
                )

                Log.e(
                    LOG_TAG,
                    "========================================"
                )
            }

        } catch (t: Throwable) {

            Log.e(
                LOG_TAG,
                "QNN initialization exception: ${t.message}",
                t
            )
        }
    }

    // ============================================================
    // Activity destroy
    // ============================================================

    override fun onDestroy() {

        Log.i(LOG_TAG, "STOP requested: activity destroyed")
        try { voiceAssistantManager.release() } catch (e: Exception) { Log.w(LOG_TAG, "voice assistant release failed", e) }
        try { validation.stop() } catch (e: Exception) { Log.w(LOG_TAG, "validation stop failed", e) }
        try { voiceGuide.release() } catch (e: Exception) { Log.w(LOG_TAG, "voice release failed", e) }
        try { sceneDescriber.release() } catch (e: Exception) { Log.w(LOG_TAG, "describer release failed", e) }
        try {

            stopPipeline()

        } catch (e: Exception) {

            Log.e(
                LOG_TAG,
                "Error stopping pipeline",
                e
            )
        }

        try {

            unregisterReceiver(
                usbReceiver
            )

        } catch (e: Exception) {

            Log.w(
                LOG_TAG,
                "USB receiver already unregistered"
            )
        }

        super.onDestroy()
    }

    // ============================================================
    // USB status
    // ============================================================

    private fun notifyUsbStatus(
        status: String
    ) {

        handler.post {

            usbStatusSink?.success(
                status
            )
        }
    }

    // ============================================================
    // Check USB devices
    // ============================================================

    private fun checkUsbDevices() {

        val deviceList =
            usbManager.deviceList

        var foundCamera = false

        for (device in deviceList.values) {

            // Intel RealSense Vendor ID
            // 0x8086 = 32902

            if (
                device.vendorId == 0x8086 ||
                device.vendorId == 32902
            ) {

                foundCamera = true

                Log.i(
                    LOG_TAG,
                    "Found Intel RealSense device"
                )

                if (
                    usbManager.hasPermission(
                        device
                    )
                ) {

                    connectToDevice(
                        device
                    )

                } else {

                    Log.i(
                        LOG_TAG,
                        "Requesting USB permission"
                    )

                    notifyUsbStatus(
                        "REQUESTING PERMISSION"
                    )

                    val flags =
                        if (
                            Build.VERSION.SDK_INT >=
                            Build.VERSION_CODES.S
                        ) {

                            PendingIntent.FLAG_MUTABLE

                        } else {

                            0
                        }

                    val intent =
                        Intent(
                            ACTION_USB_PERMISSION
                        )

                    intent.setPackage(
                        packageName
                    )

                    val permissionIntent =
                        PendingIntent.getBroadcast(
                            this,
                            0,
                            intent,
                            flags
                        )

                    usbManager.requestPermission(
                        device,
                        permissionIntent
                    )
                }

                break
            }
        }

        if (!foundCamera) {

            if (deviceList.isEmpty()) {

                notifyUsbStatus(
                    "DISCONNECTED (0 devices)"
                )

            } else {

                val vids =
                    deviceList.values
                        .joinToString {
                            it.vendorId.toString()
                        }

                notifyUsbStatus(
                    "DISCONNECTED (VIDs: $vids)"
                )
            }
        }
    }

    // ============================================================
    // Connect to D455
    // ============================================================

    private fun connectToDevice(
        device: UsbDevice
    ) {

        val connection =
            usbManager.openDevice(
                device
            )

        if (connection != null) {

            val fd =
                connection.fileDescriptor

            val usbfsPath =
                device.deviceName

            Log.i(
                LOG_TAG,
                "USB Device opened successfully. " +
                        "FD: $fd, Path: $usbfsPath"
            )
            voiceGuide.notice("Camera connected", "கேமரா இணைக்கப்பட்டது", longArrayOf(0, 40, 70, 40))

            try {

                val success =
                    initRealSenseWithFD(
                        fd,
                        usbfsPath
                    )

                if (success) {

                    notifyUsbStatus(
                        "CONNECTED"
                    )

                } else {

                    Log.e(
                        LOG_TAG,
                        "C++ initRealSenseWithFD failed"
                    )

                    notifyUsbStatus(
                        "OPEN FAILED"
                    )
                }

            } catch (e: Exception) {

                Log.e(
                    LOG_TAG,
                    "Native initRealSenseWithFD call failed",
                    e
                )

                notifyUsbStatus(
                    "OPEN FAILED"
                )
            }

        } else {

            Log.e(
                LOG_TAG,
                "Failed to open USB device"
            )

            notifyUsbStatus(
                "OPEN FAILED"
            )
        }
    }

    // ============================================================
    // Telemetry polling
    // ============================================================

    // Last time native telemetry arrived (the camera was streaming).
    private var lastStreamMs = -100000L

    // ---------------------------------------------------------------- "What's ahead?"

    /** HearSight's facts for the prompt, from the latest telemetry. */
    private fun sceneFacts(): String {
        val o = lastTelemetry ?: return ""
        val v = o.optJSONObject("validator")
        val g = o.optJSONObject("guide")
        val parts = mutableListOf<String>()
        when (v?.optInt("verdict", 0)) {
            2 -> parts.add("STOP: ${o.optString("alertReason")} ahead" +
                (o.optDouble("alertDist", -1.0).takeIf { it > 0 }?.let { " at %.1f m".format(it) } ?: ""))
            1 -> parts.add("caution: ${o.optString("alertReason")}")
        }
        val ahead = v?.optDouble("ahead", -1.0) ?: -1.0
        if (ahead > 0) parts.add("nearest thing straight ahead %.1f m (%s)".format(ahead, v?.optString("aheadSrc") ?: ""))
        g?.optString("text")?.takeIf { it.isNotBlank() }?.let { parts.add("guidance: $it") }
        if (v?.optBoolean("mayMove") == true) parts.add("the planned way ahead is clear")
        return parts.joinToString("; ")
    }

    /**
     * Safety filter: the validator decides, the description only describes.
     * STOP now -> nothing (the STOP is already being spoken); a "clear / go" claim while
     * walking is not allowed -> replaced.
     */
    private fun filterDescription(text: String?, allowDuringStop: Boolean = false): String? {
        if (text.isNullOrBlank()) return null
        val o = lastTelemetry
        if (o?.optString("alert") == "stop" && (!allowDuringStop || o.optString("alertReason") == "sensor")) return null
        if (o?.optBoolean("stale") == true) return null   // the picture may be old
        val mayMove = o?.optJSONObject("validator")?.optBoolean("mayMove") ?: false
        // Any "go / move on" advice too (phone: "you can pass it by moving forward" while the
        // way was blocked).
        val goWords = "(clear|free|open|safe|go ahead|(walk|walking|move|moving|go|going|step|stepping|continue|keep going|proceed)( \\w+)? (straight|forward|ahead|on))"
        val claimsClear = Regex("\\b$goWords\\b", RegexOption.IGNORE_CASE).containsMatchIn(text)
        if (claimsClear && !mayMove) return text.replace(Regex("[^.,;]*\\b$goWords\\b[^.,;]*[.,;]?", RegexOption.IGNORE_CASE), "")
            .trim().ifBlank { null }?.let { "$it Wait, checking the way." } ?: "Wait, checking the way."
        return text
    }

    /**
     * Fix 1: a Gemma answer is lower priority than instructions, so it was refused while
     * one was being spoken. Retry every 250 ms until it gets the voice or [waitMs] passes;
     * the safety filter runs again before every attempt (a new STOP drops it).
     */
    private fun speakDescriptionWhenFree(text: String, waitMs: Long, allowDuringStop: Boolean = false,
                                         onSpoken: () -> Unit = {}) {
        val until = android.os.SystemClock.elapsedRealtime() + waitMs
        val handler = android.os.Handler(android.os.Looper.getMainLooper())
        fun attempt() {
            val safe = filterDescription(text, allowDuringStop) ?: return
            if (voiceGuide.sayDescription(safe)) { onSpoken(); return }
            if (android.os.SystemClock.elapsedRealtime() < until) handler.postDelayed({ attempt() }, 250)
            else Log.i(LOG_TAG, "SCENE,dropped (voice busy),$safe")
        }
        handler.post { attempt() }
    }

    private fun describeScene(done: (String) -> Unit) {
        if (!sceneDescriber.modelAvailable) { done("Description model not installed"); return }
        if (sceneDescriber.busy) { done("Still describing"); return }
        val frame = getLatestColor()
        if (frame == null || frame.size < 8) { done("Start the camera first"); return }
        val bb = java.nio.ByteBuffer.wrap(frame, 0, 8).order(java.nio.ByteOrder.LITTLE_ENDIAN)
        val w = bb.int; val h = bb.int
        val rgb = frame.copyOfRange(8, frame.size)
        val facts = sceneFacts()
        voiceGuide.notice("Looking", "பார்க்கிறேன்")
        sceneDescriber.describe(rgb, w, h, facts) { text, ms ->
            val safe = filterDescription(text)
            Log.i(LOG_TAG, "SCENE,live,$ms ms,facts=[$facts],raw=[${text ?: "-"}],said=[${safe ?: "-"}]")
            safe?.let { speakDescriptionWhenFree(it, waitMs = 8000) }
            done(safe ?: (text ?: "No description"))
        }
    }

    // ------------------------------------------------ live narration (Gemma during navigation)
    //
    // Events from the telemetry (10 Hz) trigger a focused question; the deterministic
    // instruction / STOP is always spoken first by VoiceGuide, Gemma only adds context.
    //   new thing ahead (< 3 m)  · door / stairs / turn ahead  · stopped > 2 s (blocked)
    //   · open walking, at most every 15 s
    // At least 6 s between comments, one at a time, answers older than 6 s are dropped.
    private val scenePrefs by lazy { getSharedPreferences("hearsight_scene", Context.MODE_PRIVATE) }
    @Volatile private var liveDescribe = false
    private var liveLastSpokenMs = 0L
    private var liveLastAheadSrc = ""
    private var liveLastAheadMs = 0L
    private var liveLastLandmark = ""
    private var liveHaltSinceMs = 0L
    private var liveHaltExplained = false

    private fun liveOnTelemetry(o: JSONObject) {
        if (!liveDescribe || !sceneDescriber.modelAvailable || sceneDescriber.busy) return
        if (o.optString("status") != "streaming") return
        val now = android.os.SystemClock.elapsedRealtime()
        val v = o.optJSONObject("validator") ?: return
        val g = o.optJSONObject("guide")
        val verdict = v.optInt("verdict", 0)
        val code = g?.optString("code") ?: ""
        // Track a stop episode (also "No clear way").
        val halted = verdict == 2 || code == "blocked"
        if (halted) { if (liveHaltSinceMs == 0L) { liveHaltSinceMs = now; liveHaltExplained = false } }
        else liveHaltSinceMs = 0L
        if (now - liveLastSpokenMs < 6000) return

        var question: String? = null
        var key = ""
        var duringStop = false
        val ahead = v.optDouble("ahead", -1.0)
        val src = v.optString("aheadSrc")
        when {
            halted && !liveHaltExplained && now - liveHaltSinceMs > 2000 -> {
                question = "why is my way blocked, and which side, left or right, looks free to walk?"
                key = "blocked"; duringStop = true; liveHaltExplained = true
            }
            code in setOf("door", "stairs", "turn_left_soon", "turn_right_soon") && code != liveLastLandmark -> {
                question = when (code) {
                    "door" -> "describe the door ahead of me and whether it is open."
                    "stairs" -> "describe the stairs or step ahead of me."
                    else -> "describe where the corridor or path turns ahead of me."
                }
                key = code; liveLastLandmark = code
            }
            !halted && ahead in 0.5..3.0 && src.isNotBlank() && src != "none" &&
                (src != liveLastAheadSrc || now - liveLastAheadMs > 20000) -> {
                question = "what is the nearest obstacle in front of me, and on which side can I pass it?"
                key = "ahead:$src"; liveLastAheadSrc = src; liveLastAheadMs = now
            }
            !halted && v.optBoolean("mayMove") && now - liveLastSpokenMs > 15000 -> {
                question = "briefly describe what is around me."
                key = "around"
            }
        }
        if (question == null) return
        val frame = getLatestColor() ?: return
        if (frame.size < 8) return
        val bb = java.nio.ByteBuffer.wrap(frame, 0, 8).order(java.nio.ByteOrder.LITTLE_ENDIAN)
        val w = bb.int; val h = bb.int
        val rgb = frame.copyOfRange(8, frame.size)
        val facts = sceneFacts()
        liveLastSpokenMs = now   // cooldown starts at the request
        sceneDescriber.describe(rgb, w, h, facts, question) { text, ms ->
            val safe = if (ms > 6000) null else filterDescription(text, duringStop)
            Log.i(LOG_TAG, "SCENE,live-auto,$key,$ms ms,raw=[${text ?: "-"}],said=[${safe ?: "-"}]")
            safe?.let {
                // Live comments wait less: they must still match what is ahead.
                speakDescriptionWhenFree(it, waitMs = 3000, allowDuringStop = duringStop) {
                    liveLastSpokenMs = android.os.SystemClock.elapsedRealtime()
                }
            }
        }
    }

    private val describeTestReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            val rec = File(getExternalFilesDir(null), "rec")
            val frames = rec.listFiles { f -> f.isDirectory && f.name.endsWith(".frames") }
                ?.maxByOrNull { it.lastModified() }
                ?.listFiles { f -> f.name.endsWith(".ppm") }?.sortedBy { it.name } ?: emptyList()
            val pick = frames.filterIndexed { i, _ -> i % 4 == 0 }.take(10)
            val out = File(getExternalFilesDir(null), "scene_test.csv")
            out.writeText("image,ms,answer\n")
            Log.i(LOG_TAG, "SCENE,test,start,${pick.size} images,init ${sceneDescriber.lastInitMs} ms")
            fun next(i: Int) {
                if (i >= pick.size) { Log.i(LOG_TAG, "SCENE,test,done -> ${out.absolutePath}"); return }
                val (w, h, rgb) = readPpm(pick[i]) ?: run { next(i + 1); return }
                sceneDescriber.describe(rgb, w, h, "") { text, ms ->
                    out.appendText("${pick[i].name},$ms,\"${(text ?: "").replace("\"", "'")}\"\n")
                    next(i + 1)
                }
            }
            next(0)
        }
    }

    private fun readPpm(f: File): Triple<Int, Int, ByteArray>? = try {
        val b = f.readBytes()
        var pos = 0
        val tok = mutableListOf<String>()
        while (tok.size < 4) {
            while (b[pos].toInt().toChar().isWhitespace()) pos++
            val st = pos
            while (!b[pos].toInt().toChar().isWhitespace()) pos++
            tok.add(String(b, st, pos - st))
        }
        pos++
        Triple(tok[1].toInt(), tok[2].toInt(), b.copyOfRange(pos, b.size))
    } catch (e: Exception) { null }

    // Use the phone IMU unless the native D455 IMU is running.
    private fun withPhoneImu(telemetryJson: String): String =
        try {
            val obj = JSONObject(telemetryJson)
            // Sensor health: native writes telemetry every processed frame
            // (wallMs). Older than 0.7 s means the processing loop is not
            // running (hang / NPU stall): the last instruction must not stay
            // on screen -> STOP.
            val wallMs = obj.optDouble("wallMs", -1.0)
            if (wallMs > 0 && System.currentTimeMillis() - wallMs > 700 &&
                obj.optString("status") != "no_frames") {
                obj.put("alert", "stop")
                obj.put("alertReason", "sensor")
                obj.put("stale", true)
                Log.w(LOG_TAG, "STALE,age ${(System.currentTimeMillis() - wallMs).toLong()} ms,gemma busy ${sceneDescriber.busy}")
                obj.put("guide", JSONObject().put("code", "sensor_fault")
                    .put("text", "STOP! Camera stalled.").put("seq", -2))
            }
            voiceGuide.vibrateOnAlert(obj.optString("alert", "none"))
            voiceGuide.onTelemetry(obj)
            obj.optJSONObject("guide")?.let { g ->
                voiceGuide.displaySentence(obj).takeIf { it.isNotEmpty() }?.let { g.put("sentence", it) }
            }
            obj.put("voice", voiceGuide.statusJson())
            obj.put("voiceAssistant", voiceAssistantManager.getStatusJson())
            validation.telemetry(obj)
            validation.scenario?.let { obj.put("scenario", it) }
            lastTelemetry = obj
            liveOnTelemetry(obj)
            obj.put("scene", JSONObject().put("live", liveDescribe).put("busy", sceneDescriber.busy)
                .put("model", sceneDescriber.modelAvailable))
            val nativeImu = obj.optJSONObject("imu")
            if (nativeImu == null || !nativeImu.optBoolean("running")) {
                obj.put("imu", phoneImu.toJson().put("d455Failed", nativeImu?.optBoolean("d455Failed") ?: false))
            } else {
                nativeImu.put("source", "d455")
            }
            obj.toString()
        } catch (e: Exception) {
            telemetryJson
        }

    private fun startPolling() {

        handler.post(
            object : Runnable {

                override fun run() {

                    if (isPolling) {

                        try {

                            val telemetryJson =
                                pollTelemetry()

                            if (
                                telemetryJson != "{}"
                            ) {
                                lastStreamMs = android.os.SystemClock.elapsedRealtime()

                                eventSink?.success(
                                    withPhoneImu(telemetryJson)
                                )
                            }

                        } catch (e: Exception) {

                            Log.e(
                                LOG_TAG,
                                "Native pollTelemetry failed",
                                e
                            )
                        }

                        // Poll at approximately 10 Hz

                        handler.postDelayed(
                            this,
                            100
                        )
                    }
                }
            }
        )
    }
}
