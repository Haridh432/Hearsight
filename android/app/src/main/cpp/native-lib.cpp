#include <jni.h>
#include <string>
#include <android/log.h>
#include <sys/system_properties.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <thread>
#include <atomic>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <vector>
#include "qnn_runtime.h"
#include "yolo_detector.h"
#include "imu.h"
#include "tracker.h"
#include "safety.h"
#include "semseg.h"
#include "vo.h"
#include "recorder.h"
#ifdef HSV2_VIO
#include "vio_thread.h"
#include <sys/stat.h>
#endif
#include "imu_sync.h"
#include "gravity_calib.h"
#include "local_map.h"
#include "nav_plan.h"
#include "view3d.h"
#include "guidance.h"
#include "sensor_health.h"
#include "safety_validator.h"

QnnRuntime g_qnnRuntime;
YoloDetector g_detector;
// Guards g_qnnRuntime/g_detector between initQnn() and the stream thread.
std::mutex g_qnnMutex;
std::atomic<bool> g_qnnInitialized(false);
ImuStreamer g_imu;
// Track IDs, velocity and time-to-collision for detections (stream thread only).
ObjectTracker g_tracker;
// Layer 1 reactive safety (floor, corridor, head height, drop-off) from live depth.
SafetyLayer g_safety;
// Whole-scene semantic segmentation on its own worker thread (~10-15 Hz).
SemanticSegmenter g_semseg;
// Visual odometry (Phase 3): the user's own motion; stream thread only.
VoThread g_vo;
// Phone IMU on CLOCK_BOOTTIME, matched to each frame's capture time; its
// gravity (rotated into the camera by an online calibration) pins the floor fit.
PhoneImuSync g_imuSync;
GravityCalib g_gravityCalib;
// D455 IMU -> depth fine alignment from the floor (the factory extrinsics leave a
// small mounting error; this D455 reports no IMU calibration).
GravityCalib g_d455Calib;
double g_d455MatchMs = -1;
// Alignment diagnostics per 10 s log window: frames with the D455 up, with the floor found,
// and steady enough (|gyro| < 0.5 rad/s) to be used.
int g_d455Frames = 0, g_d455FloorFrames = 0, g_d455SteadyFrames = 0;   // time distance frame <-> nearest IMU sample (-1 = latest used)
// Phase 4 dynamic local map (stream thread only). debug.hsv2.map 0 disables.
LocalMap g_map;
// Phase 4b: cost map + human-centred planner (5 Hz) and the 3D view.
CostMap g_costMap;
Planner g_planner;
Guidance g_guidance;
SensorHealth g_health;
SafetyValidator g_validator;
// Scene description ("What's ahead?"): latest half-size colour frame (RGB).
std::mutex g_sceneMutex;
std::vector<uint8_t> g_sceneRgb;
int g_sceneW = 0, g_sceneH = 0;
double g_sceneT = -1;
PlanResult g_plan;
bool g_mapEnabled = true;
int64_t g_imuOffsetNs = 0;   // residual camera->IMU offset: setprop debug.hsv2.imu_offset_ms
// Walk recorder (REC button): started by the stream thread on the next frame
// once the camera calibration is known (g_recDir set by JNI).
Recorder g_recorder;
#ifdef HSV2_VIO
// OpenVINS VIO in shadow mode (pose reported, map/planner still on VO).
// Off by default: adb shell setprop debug.hsv2.vio 1 (needs the D455 IMU).
VioThread g_vio;
bool g_vioEnabled = false;
// The app's private files dir (always writable, no permission).
constexpr const char* kVioWorkDir = "/data/data/com.hsv2.hsv2_lite/files/vio";
#endif
std::mutex g_recMutex;
std::string g_recDir;          // non-empty = start requested
std::atomic<bool> g_recStopReq(false);
bool g_voEnabled = true;   // adb shell setprop debug.hsv2.vo 0 disables
// 0 = camera only, 1 = camera + segmentation overlay, 2 = segmentation only,
// 3 = 3D map view (chase camera: map, point cloud, trajectory, planned path).
std::atomic<int> g_viewMode(1);
rs2::device g_rsDevice;  // the pipeline's active device (shared with the IMU)

// Per-second timing and per-frame E2E logs; off by default because they
// exhaust the per-app log quota. Debug builds (or release built with
// -DHSV2_TIMING_LOGS=ON): adb shell setprop debug.hsv2.timing 1
std::atomic<bool> g_timingLogs(false);
// Consecutive wait_for_frames timeouts before telemetry reports no_frames.
// Timeouts are 1 s, so a stall is reported (and the alert cleared) within
// about a second; after kStallRestartTimeouts the pipeline is restarted.
constexpr int kNoFrameTimeouts = 1;
constexpr int kFrameTimeoutMs = 1000;
constexpr int kStallRestartTimeouts = 3;

// Depth auto-recovery: a UVC watchdog reset of the depth endpoint at stream
// start can leave depth frames empty (0.2-0.5% valid pixels vs 54-81% in
// normal scenes) while colour keeps flowing. Restart the pipeline when depth
// stays below kDepthDeadFraction for kDepthDeadSeconds, at most once per
// kDepthRestartCooldownSeconds so a scene with genuinely no depth can't loop.
constexpr float kDepthDeadFraction = 0.02f;
constexpr double kDepthDeadSeconds = 2.0;
constexpr double kDepthRestartCooldownSeconds = 10.0;
int g_streamFps = 30;
bool g_imuEnabled = false;
// D455 IMU watchdog (stream thread): no accel sample for kImuNoDataSec -> restart the
// pipeline once (the IMU starts with it); still nothing -> run without the D455 IMU
// (gravity from the phone through the calibration). Phone: the IMU 'started' but
// delivered nothing on some starts (badge NO DATA).
constexpr double kImuNoDataSec = 3.0;
int g_imuRestarts = 0;
bool g_imuFailed = false;
std::chrono::steady_clock::time_point g_imuStartTime;


// adb shell setprop <name> <value>; empty -> default.
static bool debugPropEnabled(const char* name, bool defaultValue) {
    char v[PROP_VALUE_MAX] = {0};
    __system_property_get(name, v);
    if (v[0] == '\0') return defaultValue;
    return v[0] != '0';
}

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "HSV2Native", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "HSV2Native", __VA_ARGS__)

int currentStage = 1;
bool isCameraInitialized = false;

// RealSense Global State
std::unique_ptr<rs2::context> rs_ctx;
std::unique_ptr<rs2::pipeline> rs_pipe;

// Fraction of non-zero depth pixels on a 40x30 grid.
static float depthValidFraction(const rs2::depth_frame& depth) {
    const uint16_t* z = static_cast<const uint16_t*>(depth.get_data());
    const int w = depth.get_width(), h = depth.get_height();
    int valid = 0, total = 0;
    for (int gy = 0; gy < 30; ++gy) {
        const uint16_t* row = z + static_cast<size_t>((gy * h) / 30) * w;
        for (int gx = 0; gx < 40; ++gx, ++total) {
            if (row[(gx * w) / 40] != 0) ++valid;
        }
    }
    return total ? static_cast<float>(valid) / total : 0.0f;
}

static void preconfigureSensors(bool keepGlobalTime) {
    try {
        if (!rs_ctx) return;
        auto devices = rs_ctx->query_devices();
        if (devices.size() > 0) {
            for (auto&& sensor : devices.front().query_sensors()) {
                if (sensor.is<rs2::motion_sensor>()) continue;
                if (!keepGlobalTime && sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                    try { sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f); } catch (...) {}
                }
                if (sensor.supports(RS2_OPTION_THERMAL_COMPENSATION)) {
                    try { sensor.set_option(RS2_OPTION_THERMAL_COMPENSATION, 0.0f); } catch (...) {}
                }
            }
        }
    } catch (...) {}
}

// (Re)creates rs_pipe streaming colour+depth at [rate], updates g_rsDevice,
// and starts D455 IMU synchronously before the frame loop begins.
static void startPipelineAt(int rate) {
    if (g_imuEnabled) {
        preconfigureSensors(debugPropEnabled("debug.hsv2.globaltime", false));
    }
    rs_pipe = std::make_unique<rs2::pipeline>(*rs_ctx);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, rate);
    rs_pipe->start(cfg);
    g_rsDevice = rs_pipe->get_active_profile().get_device();
    g_imuStartTime = std::chrono::steady_clock::now();

    if (g_imuEnabled) {
        std::string imuError;
        int gyroHz = -1, accelHz = -1;
        {
            char v[PROP_VALUE_MAX] = {0};
            __system_property_get("debug.hsv2.gyro_hz", v);
            if (atoi(v) > 0) gyroHz = atoi(v);
            v[0] = 0;
            __system_property_get("debug.hsv2.accel_hz", v);
            if (atoi(v) > 0) accelHz = atoi(v);
        }
        if (g_imu.start(g_rsDevice, imuError, gyroHz, accelHz)) {
            try {
                g_imu.setDepthProfile(rs_pipe->get_active_profile().get_stream(RS2_STREAM_DEPTH));
            } catch (const std::exception&) {}
            LOGI("D455 IMU started synchronously with pipeline (gyro %d Hz, accel %d Hz)",
                 g_imu.snapshot().gyroProfileHz, g_imu.snapshot().accelProfileHz);
        } else {
            LOGE("D455 IMU start failed: %s", imuError.c_str());
        }
    }
}
ANativeWindow* nativeWindow = nullptr;
ANativeWindow* depthNativeWindow = nullptr;
std::atomic<bool> isStreaming(false);
std::thread streamThread;
std::atomic<double> current_fps(0.0);

// Telemetry state
std::mutex telemetry_mutex;
char telemetry_json[12288] = "{}";

// Forward declare the internal librealsense USB device injection method
extern "C" void Java_com_intel_realsense_librealsense_DeviceWatcher_nAddUsbDevice(JNIEnv *env, jclass type, jstring deviceName, jint fileDescriptor);

extern "C" JNIEXPORT jboolean JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_initQnn(
    JNIEnv* env,
    jobject thiz,
    jstring contextBinaryPath,
    jstring nativeLibDir)
{
    (void)thiz;

    if (contextBinaryPath == nullptr || nativeLibDir == nullptr) {
        LOGE("QNN init: null path argument");
        return JNI_FALSE;
    }

    const char* contextPath = env->GetStringUTFChars(contextBinaryPath, nullptr);
    const char* libDir = env->GetStringUTFChars(nativeLibDir, nullptr);

    if (contextPath == nullptr || libDir == nullptr) {
        LOGE("QNN init: failed to read Java string arguments");
        if (contextPath) env->ReleaseStringUTFChars(contextBinaryPath, contextPath);
        if (libDir) env->ReleaseStringUTFChars(nativeLibDir, libDir);
        return JNI_FALSE;
    }

    LOGI("========================================");
    LOGI("JNI initQnn() called");
    LOGI("QNN context: %s", contextPath);
    LOGI("Native lib dir: %s", libDir);

    std::lock_guard<std::mutex> qnnLock(g_qnnMutex);

    g_qnnInitialized = false;

    bool success = g_qnnRuntime.initialize(contextPath, libDir);

    if (success) {
        std::string detectorError;
        success = g_detector.configure(g_qnnRuntime, detectorError);

        if (!success) {
            LOGE("YOLO detector setup failed: %s", detectorError.c_str());
        }
    }

    g_qnnInitialized = success;

    if (success) {
        LOGI("QNN initialization SUCCESS");
    } else {
        LOGE("QNN initialization FAILED: %s",
             g_qnnRuntime.getLastError().c_str());
    }

    env->ReleaseStringUTFChars(contextBinaryPath, contextPath);
    env->ReleaseStringUTFChars(nativeLibDir, libDir);

    LOGI("========================================");

    return success ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_initRealSenseWithFD(
        JNIEnv* env,
        jobject /* this */,
        jint fd,
        jstring usbfsPath) {
    const char *path = env->GetStringUTFChars(usbfsPath, nullptr);
    LOGI("Init RealSense with FD: %d, Path: %s", fd, path);
    
    try {
        if (!rs_ctx) {
            rs_ctx = std::make_unique<rs2::context>();
            LOGI("librealsense2 context instantiated successfully!");
        }
        
        // Inject the Android USB File Descriptor directly into the librealsense backend!
        Java_com_intel_realsense_librealsense_DeviceWatcher_nAddUsbDevice(env, nullptr, usbfsPath, fd);
        LOGI("Successfully injected USB FD into rsusb backend!");
        
    } catch (const std::exception& e) {
        LOGE("Error: %s", e.what());
        env->ReleaseStringUTFChars(usbfsPath, path);
        return JNI_FALSE;
    }

    env->ReleaseStringUTFChars(usbfsPath, path);
    isCameraInitialized = true;
    return JNI_TRUE;
}



// Returns "" on success, otherwise an error message for the UI.
extern "C" JNIEXPORT jstring JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_startPipeline(
        JNIEnv* env,
        jobject /* this */) {
    if (isStreaming) return env->NewStringUTF("");
    if (!rs_ctx) return env->NewStringUTF("RealSense camera not initialised");

    try {
#if defined(NDEBUG) && !defined(HSV2_TIMING_LOGS)
        // Release builds: timing logs (~90 lines/s) are compiled out, so a leftover
        // debug.hsv2.timing=1 cannot flood the log. Build with -DHSV2_TIMING_LOGS=ON for them.
        g_timingLogs = false;
#else
        g_timingLogs = debugPropEnabled("debug.hsv2.timing", false);
#endif

        // D455 IMU enabled by default; override with: adb shell setprop debug.hsv2.imu 0
        const bool imuEnabled = debugPropEnabled("debug.hsv2.imu", true);
        g_imuEnabled = imuEnabled;
        g_imuRestarts = 0;
        g_imuFailed = false;
#ifdef HSV2_VIO
        g_vioEnabled = imuEnabled && debugPropEnabled("debug.hsv2.vio", false);
        LOGI("VIO (OpenVINS, shadow mode): %s", g_vioEnabled ? "on" : "off (setprop debug.hsv2.vio 1)");
#endif
        g_d455Calib.reset();
        // The D455 IMU is built into the camera (always rigid): accept up to 4 deg RMS (recorded
        // upright walk: 3.09 deg after the outlier pass); the phone keeps its 3 deg rigidity check.
        g_d455Calib.maxResidualDeg = 4.0f;

        // Stream rate: 30 fps by default (app processing p99 ~24 ms fits the
        // 33 ms budget). Override: adb shell setprop debug.hsv2.fps 15|30|60.
        // Falls back to 15 fps if the camera/USB link refuses the higher rate.
        int fps = 30;
        {
            char v[PROP_VALUE_MAX] = {0};
            __system_property_get("debug.hsv2.fps", v);
            int requested = atoi(v);
            if (requested == 15 || requested == 30 || requested == 60) fps = requested;
        }

        if (imuEnabled) {
            preconfigureSensors(debugPropEnabled("debug.hsv2.globaltime", false));
        }

        try {
            startPipelineAt(fps);
        } catch (const std::exception& e) {
            if (fps == 15) throw;
            LOGE("Start at %d fps failed (%s); falling back to 15 fps", fps, e.what());
            fps = 15;
            startPipelineAt(fps);
        }
        g_streamFps = fps;
        g_tracker.reset();
        g_vo.stop();   // restarted with the camera calibration on the first frame
        g_map.reset();
        g_plan = PlanResult{};
        g_planner.reset();
        g_guidance.reset();
        g_health.reset();
        g_validator.reset();
        g_mapEnabled = debugPropEnabled("debug.hsv2.map", true);
        g_voEnabled = debugPropEnabled("debug.hsv2.vo", true);
        g_safety.reset();
        {
            char v[PROP_VALUE_MAX] = {0};
            __system_property_get("debug.hsv2.imu_offset_ms", v);
            g_imuOffsetNs = static_cast<int64_t>(atof(v) * 1e6);
        }
        g_imuSync.start();
        LOGI("Streaming colour+depth 640x480 @ %d fps (D455 IMU %s)",
             fps, imuEnabled ? "enabled" : "disabled");

        {
            // Drop the previous session's telemetry: it would look stale (sensor health in
            // MainActivity) and report "Camera stalled" before the first new frame.
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            snprintf(telemetry_json, sizeof(telemetry_json), "{}");
        }
        isStreaming = true;
        current_fps = 0.0;
        
        streamThread = std::thread([]() {
            LOGI("Native stream thread started!");
            rs2::colorizer color_map;   // one for the whole stream (depth view)
            
            auto last_time = std::chrono::steady_clock::now();
            int frame_count = 0;
            int consecutive_timeouts = 0;
            SegMap seg_map;
            VoResult vo_res;
            float map_ahead = -1, map_hazard = -1;
            bool map_valid = false;
            int plan_counter = 0, map_updates = 0;
            bool walk_valid = false;
            float walk_dx = 0, walk_dy = 1;
            double plan_ms_last = 0;
            int map_frames = 0;
            double map_ms_sum = 0;
            float vo_R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, vo_T[3] = {0, 0, 0};
            double vo_last_t = -1;
            int vo_frames = 0, vo_ok = 0;
            double vo_ms_sum = 0, vo_speed_sum = 0;
            std::vector<uint8_t> seg_roles;
            uint64_t seg_roles_seq = 0;

            // Depth auto-recovery state (see kDepthDeadFraction).
            auto depth_dead_since = std::chrono::steady_clock::time_point{};
            bool depth_dead = false;
            auto last_depth_restart = std::chrono::steady_clock::now() -
                std::chrono::seconds(static_cast<int>(kDepthRestartCooldownSeconds));
            
            // Depth is NOT aligned to colour: full-frame rs2::align cost
            // 35-86 ms/frame on the CPU and capped the stream at ~10 fps.
            // Instead, only the colour pixels we need depth for are
            // projected into the depth image (see depthAtColor below).
            bool calib_ready = false;
            rs2_intrinsics depth_intrin{}, color_intrin{};
            rs2_extrinsics color_to_depth{}, depth_to_color{};
            unsigned long long last_frame_num = 0;

            // Inference timing, logged once per second.
            auto stats_start = std::chrono::steady_clock::now();
            int stats_frames = 0;
            size_t stats_dets = 0;
            double stats_total_sum = 0.0, stats_total_max = 0.0;
            double stats_npu_sum = 0.0, stats_npu_max = 0.0;
            // Per-stage frame-loop timing (diagnostics).
            double stats_wait_sum = 0.0, stats_render_sum = 0.0;
            int total_dropped_frames = 0;
            
            while (isStreaming) {
                if (currentStage >= 1) {
                    try {
                        auto t_wait = std::chrono::steady_clock::now();
                        rs2::frameset frames = rs_pipe->wait_for_frames(kFrameTimeoutMs);
                        consecutive_timeouts = 0;
                        stats_wait_sum += std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_wait).count();
                        // Per-frame end-to-end timing (system clock, ms).
                        auto e2e_now_ms = [] {
                            return std::chrono::duration<double, std::milli>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
                        };
                        double t_arrive_ms = e2e_now_ms();
                        
                        rs2::video_frame color_frame = frames.get_color_frame();
                        rs2::depth_frame depth_frame = frames.get_depth_frame();

                        // --- DEPTH AUTO-RECOVERY ---
                        if (g_imuEnabled) {
                            const ImuSnapshot ims = g_imu.snapshot();
                            const double up_s = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - g_imuStartTime).count();
                            const bool no_data = ims.accelAgeSec < 0 ? up_s > kImuNoDataSec
                                                                     : ims.accelAgeSec > kImuNoDataSec;
                            if (no_data) {
                                g_imu.stop();
                                if (g_imuRestarts == 0) {
                                    ++g_imuRestarts;
                                    LOGE("D455 IMU: no data for %.0f s; restarting pipeline with IMU", kImuNoDataSec);
                                } else {
                                    g_imuEnabled = false;
                                    g_imuFailed = true;
                                    LOGE("D455 IMU: still no data after a restart; continuing without it (phone sensors)");
                                }
                                try {
                                    rs_pipe->stop();
                                    startPipelineAt(g_streamFps);
                                    calib_ready = false;
                                    LOGI("Pipeline restarted (IMU recovery)");
                                } catch (const std::exception& re) {
                                    LOGE("IMU recovery restart failed: %s", re.what());
                                }
                                continue;
                            }
                        }
                        float depth_valid_frac = depth_frame ? 1.0f : 0.0f;   // sensor health input
                        if (depth_frame && color_frame) {
                            auto now = std::chrono::steady_clock::now();
                            depth_valid_frac = depthValidFraction(depth_frame);
                            if (depth_valid_frac < kDepthDeadFraction) {
                                if (!depth_dead) {
                                    depth_dead = true;
                                    depth_dead_since = now;
                                }
                                double dead_s = std::chrono::duration<double>(now - depth_dead_since).count();
                                double since_restart_s =
                                    std::chrono::duration<double>(now - last_depth_restart).count();
                                if (dead_s >= kDepthDeadSeconds &&
                                    since_restart_s >= kDepthRestartCooldownSeconds) {
                                    LOGE("Depth empty for %.1f s; restarting pipeline @ %d fps",
                                         dead_s, g_streamFps);
                                    last_depth_restart = now;
                                    depth_dead = false;
                                    g_imu.stop();
                                    try {
                                        rs_pipe->stop();
                                        startPipelineAt(g_streamFps);
                                        calib_ready = false;
                                        LOGI("Pipeline restarted (depth recovery)");
                                    } catch (const std::exception& re) {
                                        // Next attempt after the cooldown; the
                                        // frame loop backs off on errors meanwhile.
                                        LOGE("Depth recovery restart failed: %s", re.what());
                                    }
                                    continue;
                                }
                            } else {
                                depth_dead = false;
                            }
                        }

                        if (!calib_ready && color_frame && depth_frame) {
                            auto cp = color_frame.get_profile().as<rs2::video_stream_profile>();
                            auto dp = depth_frame.get_profile().as<rs2::video_stream_profile>();
                            color_intrin = cp.get_intrinsics();
                            depth_intrin = dp.get_intrinsics();
                            color_to_depth = cp.get_extrinsics_to(dp);
                            depth_to_color = dp.get_extrinsics_to(cp);
                            calib_ready = true;
                        }

                        // Colour pixel -> depth pixel. Each call is a line search
                        // in the depth image (~60 us), so keep the count low.
                        auto colorToDepthPixel = [&](float x, float y, int& dx, int& dy) -> bool {
                            if (!calib_ready || !depth_frame) return false;
                            float from[2] = {x, y};
                            float to[2] = {-1.0f, -1.0f};
                            rs2_project_color_pixel_to_depth_pixel(
                                to,
                                static_cast<const uint16_t*>(depth_frame.get_data()),
                                depth_frame.get_units(), 0.1f, 10.0f,
                                &depth_intrin, &color_intrin,
                                &color_to_depth, &depth_to_color, from);
                            dx = static_cast<int>(to[0] + 0.5f);
                            dy = static_cast<int>(to[1] + 0.5f);
                            return dx >= 0 && dy >= 0 &&
                                   dx < depth_frame.get_width() && dy < depth_frame.get_height();
                        };

                        // Depth (metres) at a colour pixel, via colour->depth projection.
                        DepthSampler depthAtColor = [&](float x, float y) -> float {
                            int dx, dy;
                            return colorToDepthPixel(x, y, dx, dy) ? depth_frame.get_distance(dx, dy) : 0.0f;
                        };

                        // Box distance: project only the corners of the box's central
                        // 50% into the depth image (2 searches instead of 49), then take
                        // the median of a 7x7 grid of depth pixels inside that region.
                        // Falls back to per-point projection if a corner can't be mapped.
                        auto boxDepth = [&](const Detection& d) -> float {
                            int x0, y0, x1, y1;
                            if (!colorToDepthPixel(d.x + 0.25f * d.w, d.y + 0.25f * d.h, x0, y0) ||
                                !colorToDepthPixel(d.x + 0.75f * d.w, d.y + 0.75f * d.h, x1, y1)) {
                                return boxDistance(depthAtColor, d);
                            }
                            if (x0 > x1) std::swap(x0, x1);
                            if (y0 > y1) std::swap(y0, y1);
                            const uint16_t* z = static_cast<const uint16_t*>(depth_frame.get_data());
                            const int dw = depth_frame.get_width();
                            float samples[49];
                            int n = 0;
                            for (int iy = 0; iy < 7; ++iy) {
                                int y = y0 + (y1 - y0) * iy / 6;
                                for (int ix = 0; ix < 7; ++ix) {
                                    int x = x0 + (x1 - x0) * ix / 6;
                                    uint16_t v = z[static_cast<size_t>(y) * dw + x];
                                    if (v > 0) samples[n++] = v * depth_frame.get_units();
                                }
                            }
                            if (n == 0) return 0.0f;
                            std::nth_element(samples, samples + n / 2, samples + n);
                            return samples[n / 2];
                        };

                        // Person distance from the instance mask: project the box's
                        // central-50% corners once (2 searches), map colour->depth
                        // pixels linearly from them, and take the median depth over
                        // the mask pixels (background inside the box is ignored).
                        auto maskDepth = [&](const Detection& d) -> float {
                            if (!d.hasMask) return 0.0f;
                            const float ux0 = d.x + 0.25f * d.w, uy0 = d.y + 0.25f * d.h;
                            const float ux1 = d.x + 0.75f * d.w, uy1 = d.y + 0.75f * d.h;
                            int x0, y0, x1, y1;
                            if (!colorToDepthPixel(ux0, uy0, x0, y0) ||
                                !colorToDepthPixel(ux1, uy1, x1, y1)) {
                                return 0.0f;
                            }
                            const float sx = (x1 - x0) / (ux1 - ux0);
                            const float sy = (y1 - y0) / (uy1 - uy0);
                            const uint16_t* z = static_cast<const uint16_t*>(depth_frame.get_data());
                            const int dw = depth_frame.get_width(), dh = depth_frame.get_height();
                            const float units = depth_frame.get_units();
                            DepthSampler fast = [&](float u, float v) -> float {
                                int dx = static_cast<int>(x0 + (u - ux0) * sx + 0.5f);
                                int dy = static_cast<int>(y0 + (v - uy0) * sy + 0.5f);
                                if (dx < 0 || dy < 0 || dx >= dw || dy >= dh) return 0.0f;
                                uint16_t raw = z[static_cast<size_t>(dy) * dw + dx];
                                return raw ? raw * units : 0.0f;
                            };
                            return maskDistance(fast, d);
                        };
                        
                        // Dropped Frames Tracking
                        unsigned long long current_frame_num = color_frame.get_frame_number();
                        if (last_frame_num != 0 && current_frame_num > last_frame_num + 1) {
                            total_dropped_frames += (current_frame_num - last_frame_num - 1);
                        }
                        last_frame_num = current_frame_num;
                        
                        // Latency Measurement (Camera-to-Frame-Available)
                        double frame_timestamp_ms = color_frame.get_timestamp();
                        static bool domain_logged = false;
                        if (!domain_logged && g_timingLogs) {
                            LOGI("E2E: color timestamp domain = %s",
                                 rs2_timestamp_domain_to_string(color_frame.get_frame_timestamp_domain()));
                            domain_logged = true;
                        }
                        auto now_sys = std::chrono::system_clock::now();
                        double sys_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_sys.time_since_epoch()).count();
                        double latency = sys_time_ms - frame_timestamp_ms;
                        // Sensor health lag: time since the frame arrived on the phone (host
                        // clock, so no camera-clock mapping; the camera's global timestamp
                        // is ~1 s off for a few seconds after start). -1 if unsupported.
                        double latency_raw = -1.0;
                        if (color_frame.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)) {
                            const double arrival_ms = static_cast<double>(
                                color_frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL));
                            const double l = sys_time_ms - arrival_ms;
                            if (l >= 0 && l < 10000) latency_raw = l;
                        }
                        // Fallback if hardware clock domain isn't mapped to system clock
                        if (latency < 0 || latency > 1000) latency = 16.6; 
                        
                        // FPS calculation
                        frame_count++;


                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
                        if (elapsed >= 1000) {
                            current_fps = (frame_count * 1000.0) / elapsed;
                            frame_count = 0;
                            last_time = now;
                        }
                        
                        // Walk recording: the latest segmentation goes with the next frames.
                        {
                            static uint64_t rec_seg_seq = 0;
                            static SegMap rec_seg;
                            if (g_recorder.active() && g_semseg.latest(rec_seg) && rec_seg.seq != rec_seg_seq) {
                                rec_seg_seq = rec_seg.seq;
                                g_recorder.setSegmentation(rec_seg.cls.data(), rec_seg.width, rec_seg.height,
                                                           rec_seg.frameTimeSec);
                            }
                        }

                        // --- LAYER 1 REACTIVE SAFETY (live depth only) ---
                        SafetyResult safety;
                        // Scene segmentation roles for the safety layer (fresh maps only).
                        SemanticInput sem_in;
                        bool sem_ok = false;
                        if (calib_ready && g_semseg.latest(seg_map) &&
                            frame_timestamp_ms / 1000.0 - seg_map.frameTimeSec < 0.3) {
                            if (seg_roles_seq != seg_map.seq) {
                                seg_roles.resize(seg_map.cls.size());
                                for (size_t i = 0; i < seg_roles.size(); ++i)
                                    seg_roles[i] = navToSemRole(seg_map.cls[i]);
                                seg_roles_seq = seg_map.seq;
                            }
                            sem_in.roles = seg_roles.data();
                            sem_in.gridW = seg_map.width;
                            sem_in.gridH = seg_map.height;
                            for (int i = 0; i < 9; ++i) sem_in.rot[i] = depth_to_color.rotation[i];
                            for (int i = 0; i < 3; ++i) sem_in.trans[i] = depth_to_color.translation[i];
                            sem_in.fx = color_intrin.fx; sem_in.fy = color_intrin.fy;
                            sem_in.ppx = color_intrin.ppx; sem_in.ppy = color_intrin.ppy;
                            sem_in.colorW = color_intrin.width; sem_in.colorH = color_intrin.height;
                            sem_ok = true;
                        }
                        // --- PHONE IMU AT THIS FRAME ---
                        // Frame capture time on CLOCK_BOOTTIME (the sensors' clock):
                        // global-time stamps are on the system clock, so shift by "now".
                        // frame_imu is always the PHONE reading (phone axes): it feeds the
                        // phone->camera calibration and the recordings. Gravity for the
                        // safety layer comes from the D455 IMU (camera axes) while it delivers
                        // fresh data, else from the phone through the calibration.
                        RecImu frame_imu;
                        // Camera-clock time of this frame: the recorder stores the D455 samples
                        // up to it (VIO replay).
                        if (g_imuEnabled && depth_frame) frame_imu.camTsMs = depth_frame.get_timestamp();
                        float cam_up[3];
                        bool have_cam_up = false;
                        {
                            const bool ts_global = color_frame.get_frame_timestamp_domain() ==
                                                   RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME;
                            const int64_t boot_now = bootTimeNs();
                            const double age_ms = e2e_now_ms() - frame_timestamp_ms;
                            int64_t frame_boot = (ts_global && age_ms > 0 && age_ms < 500)
                                ? boot_now - static_cast<int64_t>(age_ms * 1e6)
                                : boot_now;
                            frame_boot += g_imuOffsetNs;
                            const ImuAtFrame im = g_imuSync.at(frame_boot);
                            const float gl = std::sqrt(im.gravity[0] * im.gravity[0] + im.gravity[1] * im.gravity[1] +
                                                       im.gravity[2] * im.gravity[2]);
                            if (im.valid && gl > 5.0f) {
                                frame_imu.valid = true;
                                for (int i = 0; i < 3; ++i) {
                                    frame_imu.up[i] = im.gravity[i] / gl;   // TYPE_GRAVITY points up
                                    frame_imu.gyro[i] = im.gyro[i];
                                }
                            }
                        }
                        // D455: gravity at this frame's capture time (same camera clock as the
                        // depth frame), else the latest sample; then the floor fine alignment.
                        float d455_up[3], d455_gyro[3];
                        double d455_accel_dev = 99;   // step bounce in this frame (m/s^2)
                        bool have_d455 = false;
                        if (g_imuEnabled) {
                            const double ts = depth_frame ? depth_frame.get_timestamp() : frame_timestamp_ms;
                            double match = -1;
                            if (g_imu.motionAt(ts, d455_up, d455_gyro, match, 40.0, &d455_accel_dev)) {
                                have_d455 = true;
                                g_d455MatchMs = match;
                            } else if (g_imu.getMotion(d455_up, d455_gyro, 0.1)) {
                                have_d455 = true;
                                g_d455MatchMs = -1;
                            }
                        }
                        if (have_d455) {
                            if (!g_d455Calib.toCamera(d455_up, cam_up))
                                for (int i = 0; i < 3; ++i) cam_up[i] = d455_up[i];
                            have_cam_up = true;
                        } else if (frame_imu.valid && g_gravityCalib.toCamera(frame_imu.up, cam_up)) {
                            have_cam_up = true;
                        }

                        if (have_cam_up) {
                            g_safety.setGravity(-cam_up[0], -cam_up[1], -cam_up[2]);
                        } else {
                            g_safety.clearGravity();
                        }

                        // User's own motion (odometry of the previous frame, 33 ms old).
                        g_safety.setEgoVelocity(vo_res.ok, vo_res.vel);
                        if (depth_frame && calib_ready) {
                            const DepthIntrinsics din{depth_intrin.fx, depth_intrin.fy,
                                                      depth_intrin.ppx, depth_intrin.ppy,
                                                      depth_intrin.width, depth_intrin.height};
                            safety = g_safety.process(
                                static_cast<const uint16_t*>(depth_frame.get_data()), din,
                                depth_frame.get_units(), frame_timestamp_ms / 1000.0,
                                sem_ok ? &sem_in : nullptr);
                            // Calibration pairs: phone up vs the measured floor normal,
                            // only on steady frames (slow head motion, floor found).
                            const float wr = std::sqrt(frame_imu.gyro[0] * frame_imu.gyro[0] +
                                                       frame_imu.gyro[1] * frame_imu.gyro[1] +
                                                       frame_imu.gyro[2] * frame_imu.gyro[2]);
                            if (frame_imu.valid && safety.floorFound && wr < 0.5f)
                                g_gravityCalib.add(frame_imu.up, safety.floorNormal, frame_timestamp_ms / 1000.0);
                            if (have_d455) ++g_d455Frames;
                            if (have_d455 && safety.floorFound) {
                                ++g_d455FloorFrames;
                                const float dw = std::sqrt(d455_gyro[0] * d455_gyro[0] + d455_gyro[1] * d455_gyro[1] +
                                                           d455_gyro[2] * d455_gyro[2]);
                                // Steady: slow head AND no step bounce (walking tilts gravity by
                                // ~10 deg; recorded walk: median error 4.6 -> 3.0 deg with this rule).
                                if (dw < 0.2f && d455_accel_dev < 0.5) {
                                    ++g_d455SteadyFrames;
                                    g_d455Calib.add(d455_up, safety.floorNormal, frame_timestamp_ms / 1000.0);
                                }
                            }
                        }

                        // --- INFERENCE (NPU) ---
                        std::vector<Detection> detections;
                        std::vector<TrackInfo> track_info;
                        double inference_ms = 0.0;
                        double depth_ms = 0.0;

                        if (g_qnnInitialized && color_frame) {
                            std::lock_guard<std::mutex> qnnLock(g_qnnMutex);
                            auto t0 = std::chrono::steady_clock::now();

                            g_detector.preprocess(
                                static_cast<const uint8_t*>(color_frame.get_data()),
                                color_frame.get_width(),
                                color_frame.get_height());

                            if (!g_detector.detect(g_qnnRuntime, detections)) {
                                LOGE("Inference failed: %s",
                                     g_qnnRuntime.getLastError().c_str());
                            }

                            inference_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t0).count();

                            // --- SEMANTIC SEGMENTATION (worker thread) ---
                            // Started right after the detector so it uses the NPU
                            // gap before the next frame instead of delaying YOLO.
                            // Every 2nd frame at most; skipped while still busy.
                            if ((current_frame_num & 1) == 0) {
                                g_semseg.submit(static_cast<const uint8_t*>(color_frame.get_data()),
                                                color_frame.get_width(), color_frame.get_height(),
                                                frame_timestamp_ms / 1000.0);
                            }

                            double npu_ms = g_qnnRuntime.lastExecuteMs();
                            stats_frames++;
                            stats_dets += detections.size();
                            stats_total_sum += inference_ms;
                            stats_total_max = std::max(stats_total_max, inference_ms);
                            stats_npu_sum += npu_ms;
                            stats_npu_max = std::max(stats_npu_max, npu_ms);

                            auto stats_now = std::chrono::steady_clock::now();
                            if (stats_now - stats_start >= std::chrono::seconds(1)) {
                              if (g_timingLogs) {
                                LOGI("Inference: %d frames | NPU avg %.1f ms max %.1f | "
                                     "pre+NPU+post avg %.1f ms max %.1f | %.1f dets/frame | fps %.1f",
                                     stats_frames,
                                     stats_npu_sum / stats_frames, stats_npu_max,
                                     stats_total_sum / stats_frames, stats_total_max,
                                     static_cast<double>(stats_dets) / stats_frames,
                                     current_fps.load());
                                {
                                    ImuSnapshot im = g_imu.snapshot();
                                    LOGI("IMU: %s gyro %.0f/%d Hz accel %.0f/%d Hz | pitch %.1f roll %.1f "
                                         "(accel-only %.1f %.1f) | g=(%.2f %.2f %.2f) w=(%.3f %.3f %.3f)",
                                         im.running ? "on" : "off", im.gyroHz, im.gyroProfileHz,
                                         im.accelHz, im.accelProfileHz, im.pitchDeg, im.rollDeg,
                                         im.accelPitchDeg, im.accelRollDeg,
                                         im.accel[0], im.accel[1], im.accel[2],
                                         im.gyro[0], im.gyro[1], im.gyro[2]);
                                }
                                LOGI("Loop: wait %.1f ms | render %.1f ms",
                                     stats_wait_sum / stats_frames,
                                     stats_render_sum / stats_frames);
                                LOGI("SemSeg: %.1f Hz | NPU %.1f ms", g_semseg.hz(), g_semseg.npuMs());
                                if (vo_frames)
                                    LOGI("VO: %d results/s, ok %d | speed avg %.2f m/s | %.1f ms on its thread (incl. depth registration) | inliers %d/%d tracked %d",
                                         vo_ok, vo_frames, vo_speed_sum / vo_frames, vo_ms_sum / vo_frames,
                                         vo_res.inliers, vo_res.withDepth, vo_res.tracked);
                                LOGI("IMU: phone %s | calibration %s: %d pairs, spread %.1f deg, residual %.2f deg | gravity in floor fit: %s",
                                     frame_imu.valid ? "synced" : "NOT synced",
                                     g_gravityCalib.ok() ? "OK" : "pending", g_gravityCalib.samples(),
                                     g_gravityCalib.spreadDeg(), g_gravityCalib.residualDeg(),
                                     safety.gravityUsed ? "yes" : "no");
                                if (map_frames)
                                    LOGI("MAP: %.2f ms/frame, %d learning frames/s | obstacle ahead %.2f m | hazard ahead %.2f m | movers %zu | "
                                         "PLAN: %s %.1f m, turn %.0f deg, clearance %.2f m, %.1f ms",
                                         map_ms_sum / map_frames, map_updates, map_ahead, map_hazard, g_map.movers().size(),
                                         g_plan.ok ? "ok" : g_plan.blocked ? "BLOCKED" : "none", g_plan.length, g_plan.turnDeg,
                                         g_plan.minClearance, plan_ms_last);
                                map_frames = 0;
                                map_updates = 0;
                                map_ms_sum = 0;
                                vo_frames = vo_ok = 0;
                                vo_ms_sum = vo_speed_sum = 0;
                              }
                                stats_wait_sum = stats_render_sum = 0.0;
                                stats_start = stats_now;
                                stats_frames = 0;
                                stats_dets = 0;
                                stats_total_sum = stats_total_max = 0.0;
                                stats_npu_sum = stats_npu_max = 0.0;
                            }

                            auto t_depth = std::chrono::steady_clock::now();
                            for (Detection& d : detections) {
                                d.distanceM = maskDepth(d);
                                if (d.distanceM <= 0.0f) d.distanceM = boxDepth(d);
                            }
                            depth_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t_depth).count();

                            // --- VISUAL ODOMETRY (own thread; newest result, <= ~2 frames old) ---
                            if (g_voEnabled && calib_ready && depth_frame) {
                                // REC: start / stop requests from JNI.
                                {
                                    std::lock_guard<std::mutex> rl(g_recMutex);
                                    if (!g_recDir.empty() && g_vo.running()) {
                                        RecHeader rh{};
                                        rh.grayW = color_intrin.width / 2; rh.grayH = color_intrin.height / 2;
                                        rh.depW = depth_intrin.width / 2; rh.depH = depth_intrin.height / 2;
                                        rh.depthUnits = depth_frame.get_units();
                                        rh.cam = {color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                  color_intrin.width, color_intrin.height};
                                        rh.reg = {depth_intrin.fx, depth_intrin.fy, depth_intrin.ppx, depth_intrin.ppy,
                                                  depth_intrin.width, depth_intrin.height,
                                                  color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                  color_intrin.width, color_intrin.height, {}, {}};
                                        for (int i = 0; i < 9; ++i) rh.reg.rot[i] = depth_to_color.rotation[i];
                                        for (int i = 0; i < 3; ++i) rh.reg.trans[i] = depth_to_color.translation[i];
                                        char name[64];
                                        const time_t now = time(nullptr);
                                        strftime(name, sizeof(name), "/walk_%Y%m%d_%H%M%S.hsr", localtime(&now));
                                        const std::string path = g_recDir + name;
                                        g_recorder.setImuSource([](double after, double upTo, std::vector<RecImuSample>& out) {
                                            std::vector<ImuStreamer::RawSample> raw;
                                            g_imu.rawSamples(after, upTo, raw);
                                            out.reserve(out.size() + raw.size());
                                            for (const auto& r : raw)
                                                out.push_back({r.ts, static_cast<uint8_t>(r.type), {r.v[0], r.v[1], r.v[2]}});
                                        });
                                        if (g_recorder.start(path, rh)) {
                                            g_vo.setRecorder(&g_recorder);
                                            LOGI("REC started: %s", path.c_str());
                                        } else {
                                            LOGE("REC: cannot open %s", path.c_str());
                                        }
                                        g_recDir.clear();
                                    }
                                }
                                // Latest colour frame (half size RGB, ~4 Hz) for "What's ahead?".
                                if (color_frame) {
                                    static double scene_t = -1;
                                    const double ts = frame_timestamp_ms / 1000.0;
                                    if (ts - scene_t >= 0.25) {
                                        scene_t = ts;
                                        const int cw = color_frame.get_width(), ch = color_frame.get_height();
                                        const uint8_t* src = static_cast<const uint8_t*>(color_frame.get_data());
                                        std::lock_guard<std::mutex> lk(g_sceneMutex);
                                        g_sceneW = cw / 2; g_sceneH = ch / 2;
                                        g_sceneRgb.resize(static_cast<size_t>(g_sceneW) * g_sceneH * 3);
                                        for (int y = 0; y < g_sceneH; ++y)
                                            for (int x = 0; x < g_sceneW; ++x) {
                                                const uint8_t* q = src + (static_cast<size_t>(2 * y) * cw + 2 * x) * 4;
                                                uint8_t* o = &g_sceneRgb[(static_cast<size_t>(y) * g_sceneW + x) * 3];
                                                o[0] = q[0]; o[1] = q[1]; o[2] = q[2];
                                            }
                                        g_sceneT = ts;
                                    }
                                }
                                // Colour snapshot twice a second (scene-description tests).
                                if (g_recorder.active() && color_frame) {
                                    static double rec_col_t = -1;
                                    const double tc = frame_timestamp_ms / 1000.0;
                                    if (tc - rec_col_t >= 0.5) {
                                        rec_col_t = tc;
                                        g_recorder.pushColor(tc, static_cast<const uint8_t*>(color_frame.get_data()),
                                                             color_frame.get_width(), color_frame.get_height());
                                    }
                                }
                                if (g_recStopReq.exchange(false) && g_recorder.active()) {
                                    g_recorder.stop();
                                    LOGI("REC stopped: %d frames, %d dropped, %.0f MB -> %s", g_recorder.frames(),
                                         g_recorder.dropped(), g_recorder.megabytes(), g_recorder.path().c_str());
                                }
                                if (!g_vo.running()) {
                                    DepthToColor dc{depth_intrin.fx, depth_intrin.fy, depth_intrin.ppx, depth_intrin.ppy,
                                                    depth_intrin.width, depth_intrin.height,
                                                    color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                    color_intrin.width, color_intrin.height, {}, {}};
                                    for (int i = 0; i < 9; ++i) dc.rot[i] = depth_to_color.rotation[i];
                                    for (int i = 0; i < 3; ++i) dc.trans[i] = depth_to_color.translation[i];
                                    g_vo.start({color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                color_intrin.width, color_intrin.height}, dc);
                                }
#ifdef HSV2_VIO
                                if (g_vioEnabled && g_imuEnabled && !g_vio.running()) {
                                    DepthToColor vdc{depth_intrin.fx, depth_intrin.fy, depth_intrin.ppx, depth_intrin.ppy,
                                                     depth_intrin.width, depth_intrin.height,
                                                     color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                     color_intrin.width, color_intrin.height, {}, {}};
                                    for (int i = 0; i < 9; ++i) vdc.rot[i] = depth_to_color.rotation[i];
                                    for (int i = 0; i < 3; ++i) vdc.trans[i] = depth_to_color.translation[i];
                                    ::mkdir("/data/data/com.hsv2.hsv2_lite/files", 0770);
                                    ::mkdir(kVioWorkDir, 0770);
                                    std::string verr;
                                    if (!g_vio.start(kVioWorkDir,
                                                     {color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy,
                                                      color_intrin.width, color_intrin.height},
                                                     color_intrin.width / 2, color_intrin.height / 2, vdc,
                                                     [](double after, double upTo, std::vector<VioImuSample>& out) {
                                                         std::vector<ImuStreamer::RawSample> raw;
                                                         g_imu.rawSamples(after, upTo, raw);
                                                         for (const auto& r : raw)
                                                             out.push_back({r.ts, r.type, {r.v[0], r.v[1], r.v[2]}});
                                                     },
                                                     verr)) {
                                        LOGE("VIO start failed: %s; VIO off for this session", verr.c_str());
                                        g_vioEnabled = false;
                                    }
                                }
                                if (g_vioEnabled && g_vio.running())
                                    g_vio.submit(color_frame.get_timestamp(), static_cast<const uint8_t*>(color_frame.get_data()),
                                                 color_intrin.width, color_intrin.height, vo_res.ok, vo_res.vel);
#endif
                                g_vo.submit(static_cast<const uint8_t*>(color_frame.get_data()),
                                            static_cast<const uint16_t*>(depth_frame.get_data()),
                                            depth_frame.get_units(), frame_timestamp_ms / 1000.0, &frame_imu);
                            }
                            double vo_t = -1;
                            const double prev_vo_t = vo_last_t;
                            if (!g_vo.latest(vo_res, vo_R, vo_T, vo_t) ||
                                frame_timestamp_ms / 1000.0 - vo_t > 0.2) {
                                vo_res = VoResult{};   // missing or stale: no ego-motion
                            }
                            if (vo_t != prev_vo_t && vo_t > 0) {   // count each new result once
                                vo_last_t = vo_t;
                                vo_frames++;
                                vo_ok += vo_res.ok;
                                vo_ms_sum += vo_res.ms;
                                vo_speed_sum += vo_res.speed;
                            }

                            // --- TRACKING: IDs, velocity, time-to-collision ---
                            if (calib_ready) {
                                g_tracker.setIntrinsics(color_intrin.fx, color_intrin.ppx,
                                                        color_intrin.fy, color_intrin.ppy);
                            }
                            g_tracker.setEgo(vo_res.ok, vo_R, vo_T, vo_res.vel);
                            g_tracker.update(detections, frame_timestamp_ms / 1000.0, track_info);

                            // --- DYNAMIC MAP (Phase 4) ---
                            // Live 3D: every frame with odometry follows the pose and refreshes
                            // the point cloud; the map itself only learns from stable floor.
                            if (g_mapEnabled && vo_res.ok && depth_frame && calib_ready &&
                                (safety.floorStable || g_map.initialized())) {
                                const auto t_map = std::chrono::steady_clock::now();
                                // Depth camera pose = colour pose (odometry) * depth->colour.
                                float Rdc[9];   // depth->colour, row-major (rs2 is column-major)
                                for (int i = 0; i < 3; ++i)
                                    for (int j = 0; j < 3; ++j) Rdc[i * 3 + j] = depth_to_color.rotation[j * 3 + i];
                                MapInput mi;
                                for (int i = 0; i < 3; ++i) {
                                    for (int j = 0; j < 3; ++j)
                                        mi.pose.R[i * 3 + j] = vo_R[i * 3] * Rdc[j] + vo_R[i * 3 + 1] * Rdc[3 + j] +
                                                               vo_R[i * 3 + 2] * Rdc[6 + j];
                                    mi.pose.t[i] = vo_R[i * 3] * depth_to_color.translation[0] +
                                                   vo_R[i * 3 + 1] * depth_to_color.translation[1] +
                                                   vo_R[i * 3 + 2] * depth_to_color.translation[2] + vo_T[i];
                                }
                                mi.depth = static_cast<const uint16_t*>(depth_frame.get_data());
                                mi.width = depth_intrin.width; mi.height = depth_intrin.height;
                                mi.fx = depth_intrin.fx; mi.fy = depth_intrin.fy;
                                mi.cx = depth_intrin.ppx; mi.cy = depth_intrin.ppy;
                                mi.units = depth_frame.get_units();
                                mi.step = 4;
                                for (int i = 0; i < 3; ++i) mi.floorN[i] = safety.floorNormal[i];
                                mi.floorD = safety.floorFound ? safety.cameraHeight : 0.0f;
                                mi.tSec = frame_timestamp_ms / 1000.0;
                                // Movers (people, vehicles, animals) are kept out of the
                                // static layers: their boxes, mapped colour -> depth pixels
                                // by the intrinsics (+15 px for the 6 cm stereo baseline).
                                struct MapCtx {
                                    std::vector<std::array<float, 4>> boxes;
                                    const SegMap* seg;
                                    float sx, sy, ox, oy;   // depth pixel -> colour pixel
                                    int cw, ch;
                                } mctx;
                                mctx.sx = color_intrin.fx / depth_intrin.fx; mctx.sy = color_intrin.fy / depth_intrin.fy;
                                mctx.ox = color_intrin.ppx - depth_intrin.ppx * mctx.sx;
                                mctx.oy = color_intrin.ppy - depth_intrin.ppy * mctx.sy;
                                mctx.cw = color_intrin.width; mctx.ch = color_intrin.height;
                                mctx.seg = seg_map.seq ? &seg_map : nullptr;
                                for (const Detection& d : detections)
                                    if (ObjectTracker::isMover(d.classId))
                                        mctx.boxes.push_back({d.x - 15, d.y - 15, d.x + d.w + 15, d.y + d.h + 15});
                                mi.exclude = [](void* c, int u, int v) {
                                    const MapCtx* m = static_cast<const MapCtx*>(c);
                                    const float cu = u * m->sx + m->ox, cv = v * m->sy + m->oy;
                                    for (const auto& b : m->boxes)
                                        if (cu >= b[0] && cu <= b[2] && cv >= b[1] && cv <= b[3]) return true;
                                    return false;
                                };
                                mi.excludeCtx = &mctx;
                                mi.semantic = [](void* c, int u, int v) -> uint8_t {
                                    const MapCtx* m = static_cast<const MapCtx*>(c);
                                    if (!m->seg) return 255;
                                    const int gx = static_cast<int>((u * m->sx + m->ox) * m->seg->width / m->cw);
                                    const int gy = static_cast<int>((v * m->sy + m->oy) * m->seg->height / m->ch);
                                    if (gx < 0 || gy < 0 || gx >= m->seg->width || gy >= m->seg->height) return 255;
                                    return m->seg->cls[gy * m->seg->width + gx];
                                };
                                mi.semanticCtx = &mctx;
                                g_map.integrate(mi, safety.floorStable);
                                if (safety.floorStable) map_updates++;

                                // Walking direction from odometry (world) on the map plane.
                                float vw[3];
                                for (int i = 0; i < 3; ++i)
                                    vw[i] = vo_R[i * 3] * vo_res.vel[0] + vo_R[i * 3 + 1] * vo_res.vel[1] + vo_R[i * 3 + 2] * vo_res.vel[2];
                                float wx, wy;
                                g_map.toMapDir(vw, wx, wy);
                                const float wl = std::sqrt(wx * wx + wy * wy);
                                g_map.setWalkDirection(wl > 0.3f, wl > 0 ? wx / wl : 0, wl > 0 ? wy / wl : 1);
                                walk_valid = wl > 0.3f;
                                if (walk_valid) { walk_dx = wx / wl; walk_dy = wy / wl; }

                                // Movers layer: tracked movers on the plane with velocity.
                                std::vector<MapMover> movers;
                                for (size_t i = 0; i < detections.size() && i < track_info.size(); ++i) {
                                    const Detection& d = detections[i];
                                    const TrackInfo& ti = track_info[i];
                                    if (!ObjectTracker::isMover(d.classId) || !ti.confirmed || d.distanceM <= 0) continue;
                                    const float Z = d.distanceM;
                                    const float P[3] = {(d.x + d.w * 0.5f - color_intrin.ppx) * Z / color_intrin.fx,
                                                        (d.y + d.h * 0.5f - color_intrin.ppy) * Z / color_intrin.fy, Z};
                                    const float V[3] = {ti.vx, 0, ti.vz};
                                    float pw[3], vwd[3];
                                    for (int k = 0; k < 3; ++k) {
                                        pw[k] = vo_R[k * 3] * P[0] + vo_R[k * 3 + 1] * P[1] + vo_R[k * 3 + 2] * P[2] + vo_T[k];
                                        vwd[k] = vo_R[k * 3] * V[0] + vo_R[k * 3 + 1] * V[1] + vo_R[k * 3 + 2] * V[2];
                                    }
                                    MapMover m{ti.id, 0, 0, 0, 0, d.classId};
                                    g_map.toMapPoint(pw, m.x, m.y);
                                    g_map.toMapDir(vwd, m.vx, m.vy);
                                    movers.push_back(m);
                                }
                                g_map.setMovers(movers);

                                // Remembered obstacles: narrower corridor than the live check,
                                // only well-confirmed cells, and not right at the feet (where
                                // door frames beside the user and drift end up).
                                map_ahead = g_map.obstacleAhead(0.35f, 3.0f, 0.9f, 0.6f);
                                map_hazard = g_map.hazardAhead(0.30f, 3.0f, 0.6f);
                                map_ms_sum += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - t_map).count();
                                map_frames++;
                                map_valid = g_map.initialized();
                            } else {
                                map_ahead = map_hazard = -1;
                            }
                            g_safety.setMapAhead(map_ahead, map_hazard);   // used on the next frame

                            // Planner at ~5 Hz on the current map (also while the map is not
                            // learning, so the path stays live).
                            if (map_valid && ++plan_counter >= 6) {
                                plan_counter = 0;
                                const auto t_plan = std::chrono::steady_clock::now();
                                g_costMap.build(g_map);
                                float pdx, pdy, phx, phy;
                                g_map.user(pdx, pdy, phx, phy);
                                if (walk_valid) { phx = walk_dx; phy = walk_dy; }
                                g_plan = g_planner.plan(g_map, g_costMap, phx, phy);
                                plan_ms_last = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - t_plan).count();
                            }
                        }

                        auto t_render = std::chrono::steady_clock::now();

                        // Render RGB Frame, then the decisions + telemetry.
                        // Only the drawing needs the screen buffer: safety decisions,
                        // guidance and telemetry run every frame even when the window
                        // cannot be locked (screen off, surface busy - phone test: Gemma on
                        // the GPU made the lock fail, telemetry went stale -> false
                        // "Camera stalled" and no guidance meanwhile).
                        if (color_frame) {
                            ANativeWindow_Buffer buffer{};
                            const bool window_locked = nativeWindow && ANativeWindow_lock(nativeWindow, &buffer, nullptr) == 0;
                            {
                                int width = color_frame.get_width();
                                int height = color_frame.get_height();
                                
                                if (window_locked && buffer.width >= width && buffer.height >= height) {
                                    uint8_t* out = static_cast<uint8_t*>(buffer.bits);
                                    const uint8_t* in = static_cast<const uint8_t*>(color_frame.get_data());
                                    
                                    // Direct fast copy because we requested RGBA8
                                    for (int y = 0; y < height; ++y) {
                                        memcpy(out + y * buffer.stride * 4, in + y * width * 4, width * 4);
                                    }

                                    // 3D map view replaces the camera image.
                                    // Redrawn every 2nd frame (15 Hz view; safety / map / planner
                                    // still run at 30 Hz), the cached picture is shown in between.
                                    if (g_viewMode == 3 && map_valid) {
                                        static std::vector<uint8_t> view3d_cache;
                                        static int view3d_tick = 0;
                                        const size_t rowBytes = static_cast<size_t>(width) * 4;
                                        if (view3d_cache.size() != rowBytes * height || (++view3d_tick & 1)) {
                                            view3d_cache.assign(rowBytes * height, 0);
                                            static View3DState view3d_state;
                                            View3DOptions vopt;
                                            vopt.tSec = frame_timestamp_ms / 1000.0;
                                            // Path display follows the guidance (previous frame):
                                            // green only when verified safe.
                                            const GuideResult& gcur = g_guidance.current();
                                            vopt.pathState = gcur.pathVerified ? 1 : 2;
                                            if (gcur.code == GuideCode::ObstacleAhead) vopt.stopAheadDist = gcur.distance;
                                            renderView3D(view3d_cache.data(), width, height, width, g_map, &g_costMap, &g_plan,
                                                         vopt, &view3d_state);
                                        }
                                        for (int y = 0; y < height; ++y)
                                            memcpy(out + static_cast<size_t>(y) * buffer.stride * 4,
                                                   view3d_cache.data() + y * rowBytes, rowBytes);
                                    }
                                    // Segmentation overlay (newest map; it lags by up to ~100 ms).
                                    else if (g_viewMode != 0 && g_viewMode != 3 && g_semseg.latest(seg_map)) {
                                        drawSegmentation(out, width, height, buffer.stride, seg_map,
                                                         g_viewMode == 2 ? 256 : 115);
                                    }
                                    if (g_viewMode != 3)   // the 3D view has its own (map) content
                                        drawDetections(out, width, height, buffer.stride, detections);
                                    // Top-down dynamic map (8 x 8 m, heading up). The portrait
                                    // screen crops ~16% of the image on each side, so keep it
                                    // inside the visible middle, below the alert banner.
                                    if (g_mapEnabled && map_valid && g_viewMode != 3) {
                                        const int ms = 140;
                                        g_map.draw(out, buffer.stride, static_cast<int>(width * 0.83f) - ms,
                                                   static_cast<int>(height * 0.24f), ms);
                                    }
                                }
                                
                                // Extract center distance & Render Depth Frame
                                float center_distance = 0.0f;
                                if (depth_frame) {
                                    int dwidth = depth_frame.get_width();
                                    int dheight = depth_frame.get_height();
                                    center_distance = depthAtColor(
                                        color_frame.get_width() / 2.0f,
                                        color_frame.get_height() / 2.0f);
                                    
                                    if (depthNativeWindow) {
                                        // Reuse the stream thread's colorizer: a new one per frame
                                        // registered a new stream profile every frame, so librealsense
                                        // rebuilt its extrinsics graph and logged "Found 1 unreachable
                                        // streams" ~30x/s (filling the per-app log quota).
                                        rs2::frame colorized_depth = color_map.process(depth_frame);
                                        ANativeWindow_Buffer dbuffer;
                                        if (ANativeWindow_lock(depthNativeWindow, &dbuffer, nullptr) == 0) {
                                            if (dbuffer.width >= dwidth && dbuffer.height >= dheight) {
                                                const uint8_t* in = static_cast<const uint8_t*>(colorized_depth.get_data());
                                                uint8_t* out = static_cast<uint8_t*>(dbuffer.bits);
                                                
                                                for (int y = 0; y < dheight; ++y) {
                                                    for (int x = 0; x < dwidth; ++x) {
                                                        int out_idx = y * dbuffer.stride * 4 + x * 4;
                                                        int in_idx = y * dwidth * 3 + x * 3;
                                                        out[out_idx + 0] = in[in_idx + 0];
                                                        out[out_idx + 1] = in[in_idx + 1];
                                                        out[out_idx + 2] = in[in_idx + 2];
                                                        out[out_idx + 3] = 255;
                                                    }
                                                }
                                            }
                                            ANativeWindow_unlockAndPost(depthNativeWindow);
                                        }
                                    }
                                }
                                
                                // --- TELEMETRY ---
                                ImuSnapshot imu = g_imu.snapshot();
                                {
                                    // One line every 10 s (the IMU state is otherwise only on screen).
                                    static auto last_imu_log = std::chrono::steady_clock::now();
                                    const auto nowl = std::chrono::steady_clock::now();
                                    if (std::chrono::duration<double>(nowl - last_imu_log).count() >= 10.0) {
                                        last_imu_log = nowl;
                                        LOGI("D455IMU,%s,gyro %.0f Hz,accel %.0f Hz,age %.0f ms,match %.1f ms,align %s %.2f deg (%d pairs),failed %d,P %.1f R %.1f,frames %d floor %d steady %d,|a| %.2f",
                                             imu.running ? "on" : "off", imu.gyroHz, imu.accelHz,
                                             imu.accelAgeSec < 0 ? -1.0 : imu.accelAgeSec * 1000.0, g_d455MatchMs,
                                             g_d455Calib.ok() ? "ok" : "no", g_d455Calib.residualDeg(), g_d455Calib.samples(),
                                             g_imuFailed ? 1 : 0, imu.accelPitchDeg, imu.accelRollDeg,
                                             g_d455Frames, g_d455FloorFrames, g_d455SteadyFrames,
                                             std::sqrt(imu.accel[0] * imu.accel[0] + imu.accel[1] * imu.accel[1] + imu.accel[2] * imu.accel[2]));
                                        g_d455Frames = g_d455FloorFrames = g_d455SteadyFrames = 0;
                                    }
                                }
                                char imu_json[400];
                                char vio_json[384] = "{\"running\": false}";
#ifdef HSV2_VIO
                                if (g_vio.running()) {
                                    const VioThread::State vs = g_vio.state();
                                    snprintf(vio_json, sizeof(vio_json),
                                             "{\"running\": true, \"init\": %s, \"p\": [%.3f, %.3f, %.3f], "
                                             "\"speed\": %.2f, \"ms\": %.1f, \"msMax\": %.1f, \"frames\": %d, "
                                             "\"dropped\": %d, \"resets\": %d, \"initSec\": %.1f, \"start\": \"%s\"}",
                                             vs.initialized ? "true" : "false", vs.p[0], vs.p[1], vs.p[2],
                                             std::sqrt(vs.v[0] * vs.v[0] + vs.v[1] * vs.v[1] + vs.v[2] * vs.v[2]),
                                             vs.ms, vs.msMax, vs.frames, vs.dropped, vs.resets, vs.initAfterSec,
                                             vs.startMode == 1 ? "still" : vs.startMode == 2 ? "walking" : "none");
                                    static auto last_vio_log = std::chrono::steady_clock::now();
                                    const auto nv = std::chrono::steady_clock::now();
                                    if (std::chrono::duration<double>(nv - last_vio_log).count() >= 10.0) {
                                        last_vio_log = nv;
                                        LOGI("VIO,%s", vio_json);
                                    }
                                }
#endif
                                snprintf(imu_json, sizeof(imu_json),
                                         "{\"running\": %s, \"pitch\": %.1f, \"roll\": %.1f, "
                                         "\"gyro\": [%.3f, %.3f, %.3f], \"accel\": [%.2f, %.2f, %.2f], "
                                         "\"gyroHz\": %.0f, \"accelHz\": %.0f, \"ageMs\": %.0f, \"d455Failed\": %s, "
                                         "\"matchMs\": %.1f, \"alignOk\": %s, \"alignDeg\": %.2f}",
                                         imu.running ? "true" : "false", imu.accelPitchDeg, imu.accelRollDeg,
                                         imu.gyro[0], imu.gyro[1], imu.gyro[2],
                                         imu.accel[0], imu.accel[1], imu.accel[2],
                                         imu.gyroHz, imu.accelHz,
                                         imu.accelAgeSec < 0 ? -1.0 : imu.accelAgeSec * 1000.0, g_imuFailed ? "true" : "false",
                                         g_d455MatchMs, g_d455Calib.ok() ? "true" : "false", g_d455Calib.residualDeg());

                                // Combined alert: most severe of Layer 1 safety and
                                // moving objects (time-to-collision).
                                // Sensor health: can the live picture be trusted?
                                SensorHealthInput hin;
                                hin.tSec = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now().time_since_epoch()).count();
                                hin.frameTimeMs = frame_timestamp_ms;
                                hin.latencyMs = latency_raw;
                                hin.depthValid = depth_valid_frac;
                                hin.inferenceMs = inference_ms;
                                const SensorHealthResult health = g_health.update(hin);
                                static int health_logged = 0;
                                if (health.level != health_logged) {
                                    health_logged = health.level;
                                    LOGI("HEALTH,%d,%s,depth %.2f,lag %.0f,inf %.0f", health.level,
                                         sensorFaultName(health.reason), depth_valid_frac, latency_raw, inference_ms);
                                }

                                // Safety validator: the single authority. Banner, vibration
                                // and guidance all follow its decision.
                                ValidatorInput vin;
                                vin.tSec = frame_timestamp_ms / 1000.0;
                                vin.sensorLevel = health.level;
                                vin.sensorReason = static_cast<int>(health.reason);
                                vin.safetyLevel = static_cast<int>(safety.state);
                                vin.safetyReason = static_cast<int>(safety.reason);
                                vin.safetyDist = safety.distance;
                                vin.movingLevel = static_cast<int>(g_tracker.alert());
                                vin.movingTtc = g_tracker.alertTtc();
                                vin.mapValid = map_valid;
                                static char ahead_src[32] = "none";   // what set aheadDist (logs, CSV)
                                {
                                    // Nearest object directly ahead, any type: the depth
                                    // corridor (body, head, low, trip) and detected objects that
                                    // overlap the walking corridor (+-0.45 m, as Layer 1) at their
                                    // distance. Phone: a table on the left at 0.9 m, its box centre
                                    // near the image middle, stopped the user with free floor ahead.
                                    float a = -1;
                                    snprintf(ahead_src, sizeof ahead_src, "none");
                                    auto take = [&a](float d, const char* src) {
                                        if (d > 0 && (a < 0 || d < a)) { a = d; snprintf(ahead_src, sizeof ahead_src, "%s", src); }
                                    };
                                    take(safety.obstacleDist, "obstacle"); take(safety.headDist, "head");
                                    take(safety.lowObstacleDist, "low"); take(safety.tripDist, "trip");
                                    constexpr float kCorridorHalf = 0.45f;
                                    for (const auto& d : detections) {
                                        const float Z = d.distanceM;
                                        if (Z <= 0 || color_intrin.fx <= 0) continue;
                                        const float xl = (d.x - color_intrin.ppx) / color_intrin.fx * Z;
                                        const float xr = (d.x + d.w - color_intrin.ppx) / color_intrin.fx * Z;
                                        if (xr > -kCorridorHalf && xl < kCorridorHalf) {
                                            char src[24];
                                            snprintf(src, sizeof src, "det:%d", d.classId);
                                            if (Z > 0 && (a < 0 || Z < a)) { a = Z; snprintf(ahead_src, sizeof ahead_src, "%s", src); }
                                        }
                                    }
                                    vin.aheadDist = a;
                                }
                                const ValidatedDecision dec = g_validator.validate(vin, g_plan);
                                {
                                    static int v_logged = -1;
                                    static uint32_t by_logged = 0xffffffff;
                                    const int v = static_cast<int>(dec.verdict);
                                    if (v != v_logged || dec.decidedBy != by_logged) {
                                        v_logged = v; by_logged = dec.decidedBy;
                                        LOGI("VALID,%d,%s,move %d,path %d,failed 0x%x,dist %.2f,ahead %.2f %s", v,
                                             safetyCheckName(dec.decidedBy), dec.mayMove ? 1 : 0,
                                             dec.pathVerified ? 1 : 0, dec.failed, dec.dist, vin.aheadDist, ahead_src);
                                    }
                                }
                                const int alert_level = dec.alertLevel;
                                const char* alert_reason = dec.alertReason;
                                const float alert_dist = dec.dist;
                                const float alert_ttc = dec.ttc;

                                GuideInput gin;
                                gin.tSec = frame_timestamp_ms / 1000.0;
                                gin.dec = dec;
                                const GuideResult guide = g_guidance.update(g_map, g_plan, gin);
                                static int guide_logged_seq = -1;
                                if (guide.seq != guide_logged_seq) {
                                    guide_logged_seq = guide.seq;
                                    std::string gtext = guide.text;   // CSV log: no commas in the text
                                    for (char& ch : gtext) if (ch == ',') ch = ';';
                                    LOGI("GUIDE,%.2f,%s,%s,%.0f,%.1f,%d,%s,%s", gin.tSec, guideCodeName(guide.code),
                                         gtext.c_str(), guide.angleDeg, guide.distance, alert_level, alert_reason,
                                         g_plan.ok ? "ok" : g_plan.blocked ? "blocked" : "none");
                                }

                                std::string detections_json = "[";
                                for (size_t i = 0; i < detections.size(); ++i) {
                                    const Detection& d = detections[i];
                                    const TrackInfo ti = i < track_info.size() ? track_info[i] : TrackInfo{};
                                    char item[320];
                                    snprintf(item, sizeof(item),
                                             "%s{\"class\": %d, \"confidence\": %.3f, \"distanceMeters\": %.2f, "
                                             "\"bbox\": [%.1f, %.1f, %.1f, %.1f], \"masked\": %s, "
                                             "\"track\": %d, \"speed\": %.2f, \"approaching\": %s, \"ttc\": %.2f}",
                                             i ? ", " : "", d.classId, d.confidence, d.distanceM,
                                             d.x, d.y, d.w, d.h, d.hasMask ? "true" : "false",
                                             ti.confirmed ? ti.id : -1, ti.speed,
                                             ti.approaching ? "true" : "false", ti.ttc);
                                    detections_json += item;
                                }
                                detections_json += "]";

                                // Segmentation summary: class shares of the lower-middle
                                // band of the image (the ground just ahead).
                                char seg_json[256] = "null";
                                if (seg_map.seq) {
                                    int counts[kNavCount] = {0}, total = 0;
                                    for (int y = seg_map.height / 2; y < seg_map.height; ++y)
                                        for (int x = seg_map.width / 3; x < seg_map.width * 2 / 3; ++x, ++total)
                                            counts[seg_map.cls[y * seg_map.width + x]]++;
                                    const float k = total ? 1.0f / total : 0.0f;
                                    snprintf(seg_json, sizeof(seg_json),
                                             "{\"hz\": %.1f, \"ms\": %.1f, \"floor\": %.2f, \"stairs\": %.2f, "
                                             "\"obstacle\": %.2f, \"person\": %.2f, \"door\": %.2f}",
                                             g_semseg.hz(), g_semseg.npuMs(),
                                             (counts[kNavFloor] + counts[kNavSidewalk] + counts[kNavRoad]) * k,
                                             counts[kNavStairs] * k, counts[kNavObstacle] * k,
                                             counts[kNavPerson] * k, counts[kNavDoor] * k);
                                }

                                std::lock_guard<std::mutex> lock(telemetry_mutex);
                                snprintf(telemetry_json, sizeof(telemetry_json),
                                         "{\"status\": \"streaming\", \"fps\": %.1f, \"p50\": %.1f, \"p95\": 35.0, \"p99\": 45.0, "
                                         "\"cpu\": 30.0, \"gpu\": 15.0, \"npu\": 5.0, \"thermal\": 38.0, "
                                         "\"battery\": 100.0, \"centerDistance\": %.2f, \"droppedFrames\": %d, "
                                         "\"inferenceMs\": %.1f, \"imu\": %s, \"vio\": %s, "
                                         "\"alert\": \"%s\", \"alertReason\": \"%s\", \"alertDist\": %.2f, \"alertTtc\": %.2f, "
                                         "\"safety\": {\"floor\": %s, \"camHeight\": %.2f, \"pitch\": %.1f, \"ms\": %.2f, \"semantic\": %s}, "
                                         "\"semseg\": %s, \"viewMode\": %d, "
                                         "\"vo\": {\"ok\": %s, \"speed\": %.2f, \"turn\": %.1f, \"inliers\": %d, \"ms\": %.1f}, "
                                         "\"rec\": {\"on\": %s, \"frames\": %d, \"mb\": %.0f, \"dropped\": %d}, "
                                         "\"imuCal\": {\"synced\": %s, \"ok\": %s, \"pairs\": %d, \"residual\": %.2f, \"gravity\": %s}, "
                                         "\"map\": {\"on\": %s, \"obstacle\": %.2f, \"hazard\": %.2f, \"movers\": %zu}, "
                                         "\"plan\": {\"ok\": %s, \"blocked\": %s, \"turn\": %.0f, \"length\": %.1f, \"clearance\": %.2f}, "
                                         "\"validator\": {\"verdict\": %d, \"by\": \"%s\", \"mayMove\": %s, \"failed\": %u, "
                                         "\"pathOk\": %s, \"dist\": %.2f, \"ahead\": %.2f, \"obst\": %.2f, \"head\": %.2f, \"drop\": %.2f, \"aheadSrc\": \"%s\"}, "
                                         "\"guide\": {\"code\": \"%s\", \"text\": \"%s\", \"angle\": %.0f, \"dist\": %.1f, \"seq\": %d}, "
                                         "\"health\": {\"level\": %d, \"reason\": \"%s\", \"depthValid\": %.2f}, \"wallMs\": %.0f, "
                                         "\"detections\": %s}",
                                         current_fps.load(), latency, center_distance, total_dropped_frames,
                                         inference_ms, imu_json, vio_json,
                                         alert_level == 2 ? "stop" : alert_level == 1 ? "caution" : "none",
                                         alert_reason, alert_dist, alert_ttc,
                                         safety.floorFound ? "true" : "false", safety.cameraHeight,
                                         safety.cameraPitchDeg, safety.ms,
                                         safety.semanticUsed ? "true" : "false", seg_json, g_viewMode.load(),
                                         vo_res.ok ? "true" : "false", vo_res.speed, vo_res.angRate * 57.2958f,
                                         vo_res.inliers, vo_res.ms,
                                         g_recorder.active() ? "true" : "false", g_recorder.frames(),
                                         g_recorder.megabytes(), g_recorder.dropped(),
                                         frame_imu.valid ? "true" : "false", g_gravityCalib.ok() ? "true" : "false",
                                         g_gravityCalib.samples(), g_gravityCalib.residualDeg(),
                                         safety.gravityUsed ? "true" : "false",
                                         map_valid ? "true" : "false", map_ahead, map_hazard, g_map.movers().size(),
                                         g_plan.ok ? "true" : "false", g_plan.blocked ? "true" : "false",
                                         g_plan.turnDeg, g_plan.length, g_plan.minClearance,
                                         static_cast<int>(dec.verdict), safetyCheckName(dec.decidedBy),
                                         dec.mayMove ? "true" : "false", static_cast<unsigned>(dec.failed),
                                         dec.pathVerified ? "true" : "false", dec.dist, vin.aheadDist,
                                         safety.obstacleDist, safety.headDist, safety.dropDist, ahead_src,
                                         guideCodeName(guide.code), guide.text.c_str(), guide.angleDeg, guide.distance, guide.seq,
                                         health.level, sensorFaultName(health.reason), depth_valid_frac,
                                         static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::system_clock::now().time_since_epoch()).count()),
                                         detections_json.c_str());
                                
                                if (window_locked) ANativeWindow_unlockAndPost(nativeWindow);
                                double render_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - t_render).count();
                                stats_render_sum += render_ms;

                                // SAFE,t,alert,reason,dist,ttc,floor,camH,pitch,obs,head,drop,unk,stairs,low,sem,closing,stopDist
                                if (g_timingLogs) LOGI("SAFE,%.2f,%d,%s,%.2f,%.2f,%d,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f",
                                     frame_timestamp_ms / 1000.0, alert_level, alert_reason, alert_dist, alert_ttc,
                                     safety.floorFound ? 1 : 0, safety.cameraHeight, safety.cameraPitchDeg,
                                     safety.obstacleDist, safety.headDist, safety.dropDist, safety.unknownDist,
                                     safety.stairsDist, safety.lowObstacleDist, safety.semanticUsed ? 1 : 0,
                                     safety.closingSpeed, safety.stopDistUsed);
                                if (g_timingLogs) LOGI("VOF,%.2f,%d,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%.2f",
                                     frame_timestamp_ms / 1000.0, vo_res.ok ? 1 : 0, vo_res.vel[0], vo_res.vel[1],
                                     vo_res.vel[2], vo_res.angRate * 57.2958f, vo_res.inliers,
                                     vo_T[0], vo_T[1], vo_T[2]);

                                // E2E,frame,capture->arrive,infer,render,arrive->post,capture->post,dets,depth
                                double t_post_ms = e2e_now_ms();
                                if (g_timingLogs) LOGI("E2E,%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%zu,%.2f",
                                     static_cast<unsigned long long>(current_frame_num),
                                     t_arrive_ms - frame_timestamp_ms, inference_ms, render_ms,
                                     t_post_ms - t_arrive_ms, t_post_ms - frame_timestamp_ms,
                                     detections.size(), depth_ms);
                            }
                        }
                        
                    } catch (const std::exception& e) {
                        LOGE("Frame loop exception: %s", e.what());
                        // Avoid a tight loop if the pipeline is not running.
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        if (++consecutive_timeouts == kNoFrameTimeouts) {
                            // Camera stopped delivering (unplugged or USB stall).
                            std::lock_guard<std::mutex> lock(telemetry_mutex);
                            snprintf(telemetry_json, sizeof(telemetry_json),
                                     "{\"status\": \"no_frames\", \"fps\": 0.0, \"alert\": \"stop\", \"alertReason\": \"sensor\", "
                                     "\"guide\": {\"code\": \"sensor_fault\", \"text\": \"STOP! Camera lost.\", \"seq\": -1}, "
                                     "\"detections\": []}");
                            LOGI("HEALTH,2,no_frames");
                            current_fps = 0.0;
                        }
                        if (consecutive_timeouts % kStallRestartTimeouts == 0) {
                            // Whole stream stalled (no colour either): restart it.
                            LOGE("No frames for %d s; restarting pipeline @ %d fps",
                                 consecutive_timeouts, g_streamFps);
                            g_imu.stop();
                            try {
                                try { rs_pipe->stop(); } catch (...) {}
                                startPipelineAt(g_streamFps);
                                calib_ready = false;
                                LOGI("Pipeline restarted (stall recovery)");
                            } catch (const std::exception& re) {
                                LOGE("Stall recovery restart failed: %s", re.what());
                            }
                        }
                    }
                }
            }
            LOGI("Native stream thread stopped!");
        });
        
    } catch (const std::exception& e) {
        LOGE("Failed to start pipeline: %s", e.what());
        rs_pipe.reset();
        return env->NewStringUTF(e.what());
    }
    return env->NewStringUTF("");
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_stopPipeline(
        JNIEnv* env,
        jobject /* this */) {
    if (!isStreaming) return;
    
    isStreaming = false;
    if (streamThread.joinable()) {
        streamThread.join();
    }

    g_imu.stop();
    g_imuSync.stop();
    g_vo.stop();
#ifdef HSV2_VIO
    g_vio.stop();
#endif
    g_vo.setRecorder(nullptr);
    if (g_recorder.active()) {
        g_recorder.stop();
        LOGI("REC stopped with the pipeline: %d frames -> %s", g_recorder.frames(), g_recorder.path().c_str());
    }
    
    if (rs_pipe) {
        try {
            rs_pipe->stop();
        } catch (...) {}
        rs_pipe.reset();
    }
    g_rsDevice = rs2::device();
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_initSemSeg(
        JNIEnv* env, jobject /* this */, jstring contextBinaryPath, jstring nativeLibDir) {
    const char* ctx = env->GetStringUTFChars(contextBinaryPath, nullptr);
    const char* lib = env->GetStringUTFChars(nativeLibDir, nullptr);
    std::string err;
    const bool ok = g_semseg.init(ctx, lib, err);
    if (!ok) LOGE("SemSeg init failed: %s", err.c_str());
    env->ReleaseStringUTFChars(contextBinaryPath, ctx);
    env->ReleaseStringUTFChars(nativeLibDir, lib);
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_startRecording(JNIEnv* env, jobject /* this */, jstring dir) {
    const char* d = env->GetStringUTFChars(dir, nullptr);
    {
        std::lock_guard<std::mutex> rl(g_recMutex);
        g_recDir = d;
    }
    env->ReleaseStringUTFChars(dir, d);
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_stopRecording(JNIEnv* /* env */, jobject /* this */) {
    g_recStopReq = true;
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_setViewMode(
        JNIEnv* /* env */, jobject /* this */, jint mode) {
    g_viewMode = mode;
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_setStage(
        JNIEnv* env,
        jobject /* this */,
        jint stage) {
    currentStage = stage;
    LOGI("Pipeline Stage set to %d", currentStage);
}

// Latest colour frame for the scene describer: [w(4 bytes LE), h(4), RGB...] or null.
extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_getLatestColor(JNIEnv* env, jobject /* this */) {
    std::lock_guard<std::mutex> lk(g_sceneMutex);
    if (!isStreaming || g_sceneRgb.empty()) return nullptr;
    const jsize n = static_cast<jsize>(8 + g_sceneRgb.size());
    jbyteArray out = env->NewByteArray(n);
    if (!out) return nullptr;
    const int32_t wh[2] = {g_sceneW, g_sceneH};
    env->SetByteArrayRegion(out, 0, 8, reinterpret_cast<const jbyte*>(wh));
    env->SetByteArrayRegion(out, 8, static_cast<jsize>(g_sceneRgb.size()),
                            reinterpret_cast<const jbyte*>(g_sceneRgb.data()));
    return out;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_pollTelemetry(
        JNIEnv* env,
        jobject /* this */) {
    if (!isStreaming) {
        return env->NewStringUTF("{}");
    }
    
    std::string current_telemetry;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        current_telemetry = telemetry_json;
    }
    
    return env->NewStringUTF(current_telemetry.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_setNativeWindow(
        JNIEnv* env,
        jobject /* this */,
        jobject surface) {
    if (nativeWindow) {
        ANativeWindow_release(nativeWindow);
        nativeWindow = nullptr;
    }
    if (surface) {
        nativeWindow = ANativeWindow_fromSurface(env, surface);
        LOGI("ANativeWindow set successfully!");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_releaseNativeWindow(
        JNIEnv* env,
        jobject /* this */) {
    if (nativeWindow) {
        ANativeWindow_release(nativeWindow);
        nativeWindow = nullptr;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_setDepthNativeWindow(
        JNIEnv* env,
        jobject /* this */,
        jobject surface) {
    if (depthNativeWindow) {
        ANativeWindow_release(depthNativeWindow);
        depthNativeWindow = nullptr;
    }
    if (surface) {
        depthNativeWindow = ANativeWindow_fromSurface(env, surface);
        LOGI("ANativeWindow for Depth set successfully!");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_hsv2_hsv2_1lite_MainActivity_releaseDepthNativeWindow(
        JNIEnv* env,
        jobject /* this */) {
    if (depthNativeWindow) {
        ANativeWindow_release(depthNativeWindow);
        depthNativeWindow = nullptr;
    }
}
