#include "imu_sync.h"

#include <android/looper.h>
#include <android/log.h>
#include <android/sensor.h>

#include <algorithm>
#include <ctime>

#define LOG_TAG "HSV2_IMU_SYNC"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {
constexpr int kSamplePeriodUs = 5000;  // request 200 Hz (phone delivers ~236 Hz)
constexpr int kLooperId = 1;
}  // namespace

int64_t bootTimeNs() {
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void PhoneImuSync::Ring::push(const Sample& s) {
    buf[head] = s;
    head = (head + 1) % kSize;
    if (count < kSize) ++count;
}

bool PhoneImuSync::Ring::interp(int64_t t, float out[3], double* gapMs) const {
    if (count < 2) return false;
    const size_t oldest = (head + kSize - count) % kSize;
    const Sample& first = buf[oldest];
    const Sample& last = buf[(head + kSize - 1) % kSize];
    if (t < first.t || t > last.t) return false;

    // Binary search over the ring (timestamps are increasing).
    size_t lo = 0, hi = count - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (buf[(oldest + mid) % kSize].t <= t) lo = mid; else hi = mid;
    }
    const Sample& a = buf[(oldest + lo) % kSize];
    const Sample& b = buf[(oldest + hi) % kSize];
    const double span = static_cast<double>(b.t - a.t);
    const float w = span > 0 ? static_cast<float>((t - a.t) / span) : 0.0f;
    for (int i = 0; i < 3; ++i) out[i] = a.v[i] + w * (b.v[i] - a.v[i]);
    if (gapMs) *gapMs = std::min(t - a.t, b.t - t) / 1e6;
    return true;
}

bool PhoneImuSync::start() {
    if (running_) return true;
    stopRequested_ = false;
    thread_ = std::thread([this] { loop(); });
    return true;
}

void PhoneImuSync::stop() {
    stopRequested_ = true;
    if (thread_.joinable()) thread_.join();
    running_ = false;
}

void PhoneImuSync::loop() {
    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    ASensorManager* manager = ASensorManager_getInstance();
#pragma clang diagnostic pop
    if (!manager) { LOGE("No ASensorManager"); return; }

    const ASensor* gyro = ASensorManager_getDefaultSensor(manager, ASENSOR_TYPE_GYROSCOPE);
    const ASensor* gravity = ASensorManager_getDefaultSensor(manager, 9 /* TYPE_GRAVITY */);
    if (!gyro || !gravity) {
        LOGE("Phone gyro/gravity sensor missing");
        return;
    }

    ASensorEventQueue* queue =
        ASensorManager_createEventQueue(manager, looper, kLooperId, nullptr, nullptr);
    // enableSensor + setEventRate (API 9); registerSensor needs API 26.
    for (const ASensor* sensor : {gyro, gravity}) {
        ASensorEventQueue_enableSensor(queue, sensor);
        ASensorEventQueue_setEventRate(queue, sensor, kSamplePeriodUs);
    }
    running_ = true;
    LOGI("Phone IMU sync started (boottime timestamps)");

    ASensorEvent events[32];
    while (!stopRequested_) {
        int id = ALooper_pollOnce(100 /* ms */, nullptr, nullptr, nullptr);
        if (id != kLooperId) continue;
        ssize_t n;
        while ((n = ASensorEventQueue_getEvents(queue, events, 32)) > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (ssize_t i = 0; i < n; ++i) {
                const ASensorEvent& e = events[i];
                Sample s{e.timestamp, {e.data[0], e.data[1], e.data[2]}};
                if (e.type == ASENSOR_TYPE_GYROSCOPE) gyro_.push(s);
                else if (e.type == 9) gravity_.push(s);
            }
        }
    }

    ASensorEventQueue_disableSensor(queue, gyro);
    ASensorEventQueue_disableSensor(queue, gravity);
    ASensorManager_destroyEventQueue(manager, queue);
    running_ = false;
    LOGI("Phone IMU sync stopped");
}

ImuAtFrame PhoneImuSync::at(int64_t tNs) {
    ImuAtFrame r;
    std::lock_guard<std::mutex> lock(mutex_);
    double gap = 0.0;
    bool g1 = gyro_.interp(tNs, r.gyro, &gap);
    bool g2 = gravity_.interp(tNs, r.gravity, nullptr);
    r.valid = g1 && g2;
    r.gapMs = gap;
    return r;
}
