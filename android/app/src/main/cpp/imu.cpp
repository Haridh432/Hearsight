#include "imu.h"

#include <chrono>

#include <android/log.h>

#include <algorithm>
#include <cmath>
#include <vector>

#define LOG_TAG "HSV2_IMU"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {

constexpr float kRadToDeg = 57.2957795f;
// Weight of the accel angle per accel sample (~100 Hz): time constant ~0.5 s.
constexpr float kAccelBlend = 0.02f;
// Low-pass on accel before deriving angles.
constexpr float kGravityLowPass = 0.2f;
// Accepted |gravity| range, m/s^2 (1 g = 9.81).
constexpr float kMinG = 8.0f, kMaxG = 11.6f;

// Use the highest rates the device offers. Lower D455 rates (gyro 200 /
// accel 100) stalled the camera's USB link on rsusb; the highest rates
// (as used by the working MediaPipe build) stream alongside colour/depth.
constexpr int kPreferredGyroHz = -1;
constexpr int kPreferredAccelHz = -1;

rs2::stream_profile pickProfile(const std::vector<rs2::stream_profile>& profiles,
                                rs2_stream stream, int preferredHz) {
    rs2::stream_profile best;
    for (const auto& p : profiles) {
        if (p.stream_type() != stream || p.format() != RS2_FORMAT_MOTION_XYZ32F) continue;
        if (preferredHz > 0 && p.fps() == preferredHz) return p;
        if (!best || (preferredHz <= 0 && p.fps() > best.fps())) best = p;
    }
    if (!best) {
        for (const auto& p : profiles) {
            if (p.stream_type() == stream && p.format() == RS2_FORMAT_MOTION_XYZ32F) {
                if (!best || std::abs(p.fps() - preferredHz) < std::abs(best.fps() - preferredHz)) best = p;
            }
        }
    }
    return best;
}

// Angles from the accel ("up") vector in the IMU frame (x right, y down, z forward).
void accelAngles(const float a[3], float& pitchDeg, float& rollDeg) {
    pitchDeg = std::atan2(a[2], std::sqrt(a[0] * a[0] + a[1] * a[1])) * kRadToDeg;
    rollDeg = std::atan2(-a[0], -a[1]) * kRadToDeg;
}

}  // namespace


bool ImuStreamer::start(const rs2::device& device, std::string& error, int preferredGyroHz, int preferredAccelHz) {
    stop();

    try {
        for (auto&& s : device.query_sensors()) {
            rs2::motion_sensor motion = s.as<rs2::motion_sensor>();
            if (!motion) continue;


            auto profiles = motion.get_stream_profiles();
            rs2::stream_profile gyro = pickProfile(profiles, RS2_STREAM_GYRO, preferredGyroHz);
            rs2::stream_profile accel = pickProfile(profiles, RS2_STREAM_ACCEL, preferredAccelHz);

            if (!gyro || !accel) {
                error = "Motion sensor has no gyro/accel MOTION_XYZ32F profiles";
                return false;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = ImuSnapshot{};
                state_.gyroProfileHz = gyro.fps();
                state_.accelProfileHz = accel.fps();
                filterInit_ = false;
                haveAccel_ = false;
                lastAccelSteady_ = -1;
                accHist_.clear();
                raw_.clear();
                gyrHist_.clear();
                lastGyroTs_ = 0.0;
                rateWindowStart_ = 0.0;
                rateGyro0_ = rateAccel0_ = 0;
            }

            accelProfile_ = accel;
            motion.open(std::vector<rs2::stream_profile>{gyro, accel});
            motion.start([this](rs2::frame f) { onFrame(f); });

            sensor_ = s;  // keep the sensor (and its callback) alive
            started_ = true;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.running = true;
            }

            LOGI("D455 IMU started: gyro %d Hz (req %d), accel %d Hz (req %d)",
                 gyro.fps(), preferredGyroHz, accel.fps(), preferredAccelHz);
            return true;
        }
    } catch (const rs2::error& e) {
        error = std::string("IMU start failed: ") + e.what();
        return false;
    }

    error = "No motion sensor on this device (D455 expected)";
    return false;
}


void ImuStreamer::stop() {
    if (!started_) return;

    try {
        sensor_.stop();
        sensor_.close();
    } catch (const rs2::error& e) {
        LOGE("IMU stop: %s", e.what());
    }

    sensor_ = rs2::sensor();
    started_ = false;

    std::lock_guard<std::mutex> lock(mutex_);
    state_.running = false;
    LOGI("D455 IMU stopped");
}


ImuSnapshot ImuStreamer::snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    ImuSnapshot s = state_;
    if (lastAccelSteady_ >= 0) s.accelAgeSec = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() - lastAccelSteady_;
    return s;
}


void ImuStreamer::onFrame(const rs2::frame& frame) {
    rs2::motion_frame mf = frame.as<rs2::motion_frame>();
    if (!mf) return;

    const rs2_vector v = mf.get_motion_data();
    const double ts = mf.get_timestamp();  // ms
    const rs2_stream stream = mf.get_profile().stream_type();

    std::lock_guard<std::mutex> lock(mutex_);

    if (stream == RS2_STREAM_GYRO) {
        state_.gyro[0] = v.x; state_.gyro[1] = v.y; state_.gyro[2] = v.z;
        state_.gyroCount++;
        gyrHist_.push_back({ts, {v.x, v.y, v.z}});
        raw_.push_back({ts, 2, {v.x, v.y, v.z}});
        while (!gyrHist_.empty() && ts - gyrHist_.front().ts > 1000.0) gyrHist_.pop_front();

        if (filterInit_ && lastGyroTs_ > 0.0) {
            float dt = static_cast<float>((ts - lastGyroTs_) / 1000.0);
            if (dt > 0.0f && dt < 0.1f) {
                // +x rotation tilts z (forward) up; +z rotation rolls clockwise.
                state_.pitchDeg += v.x * dt * kRadToDeg;
                state_.rollDeg += v.z * dt * kRadToDeg;
            }
        }
        lastGyroTs_ = ts;

    } else if (stream == RS2_STREAM_ACCEL) {
        state_.accel[0] = v.x; state_.accel[1] = v.y; state_.accel[2] = v.z;
        state_.accelCount++;
        raw_.push_back({ts, 1, {v.x, v.y, v.z}});
        lastAccelSteady_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

        const float a[3] = {v.x, v.y, v.z};
        if (!haveAccel_) {
            for (int i = 0; i < 3; ++i) gravity_[i] = a[i];
            haveAccel_ = true;
        } else {
            for (int i = 0; i < 3; ++i) gravity_[i] += kGravityLowPass * (a[i] - gravity_[i]);
        }

        accHist_.push_back({ts, {gravity_[0], gravity_[1], gravity_[2]}});
        while (!accHist_.empty() && ts - accHist_.front().ts > 1000.0) accHist_.pop_front();

        accelAngles(gravity_, state_.accelPitchDeg, state_.accelRollDeg);

        if (!filterInit_) {
            state_.pitchDeg = state_.accelPitchDeg;
            state_.rollDeg = state_.accelRollDeg;
            filterInit_ = true;
        } else {
            // Blend toward the accel angle, taking the short way round for roll.
            state_.pitchDeg += kAccelBlend * (state_.accelPitchDeg - state_.pitchDeg);
            float dr = state_.accelRollDeg - state_.rollDeg;
            if (dr > 180.0f) dr -= 360.0f;
            if (dr < -180.0f) dr += 360.0f;
            state_.rollDeg += kAccelBlend * dr;
            if (state_.rollDeg > 180.0f) state_.rollDeg -= 360.0f;
            if (state_.rollDeg < -180.0f) state_.rollDeg += 360.0f;
        }
    } else {
        return;
    }
    while (!raw_.empty() && ts - raw_.front().ts > 2000.0) raw_.pop_front();

    // Measured sample rates over ~1 s windows (hardware time).
    if (rateWindowStart_ == 0.0) {
        rateWindowStart_ = ts;
        rateGyro0_ = state_.gyroCount;
        rateAccel0_ = state_.accelCount;
    } else if (ts - rateWindowStart_ >= 1000.0) {
        double sec = (ts - rateWindowStart_) / 1000.0;
        state_.gyroHz = static_cast<float>((state_.gyroCount - rateGyro0_) / sec);
        state_.accelHz = static_cast<float>((state_.accelCount - rateAccel0_) / sec);
        rateWindowStart_ = ts;
        rateGyro0_ = state_.gyroCount;
        rateAccel0_ = state_.accelCount;
    }
}


bool ImuStreamer::getMotion(float camUp[3], float gyro[3], double maxAgeSec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.running || !haveAccel_ || lastAccelSteady_ < 0) return false;
    // Stale (the IMU stopped delivering): the caller falls back to the phone.
    if (std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() - lastAccelSteady_ > maxAgeSec) return false;
    const float gl = std::sqrt(gravity_[0] * gravity_[0] + gravity_[1] * gravity_[1] + gravity_[2] * gravity_[2]);
    // Plausibility: at walking pace the low-passed accel is ~1 g. Anything else is a
    // decoding / sensor fault (phone: 38-byte 32-bit reports decoded as 16-bit gave
    // nonsense) - the caller falls back to the phone.
    if (gl < kMinG || gl > kMaxG) return false;
    // Proper acceleration in D455 IMU frame counteracting gravity:
    // When level, gravity pulls down (+y in camera frame), so accel reads (0, -9.81, 0) pointing UP.
    // camUp is the unit vector pointing UP in camera coordinates:
    const float up[3] = {gravity_[0] / gl, gravity_[1] / gl, gravity_[2] / gl};
    rotate(up, camUp);
    rotate(state_.gyro, gyro);
    return true;
}


void ImuStreamer::rotate(const float in[3], float out[3]) const {
    for (int r = 0; r < 3; ++r) out[r] = R_[r * 3] * in[0] + R_[r * 3 + 1] * in[1] + R_[r * 3 + 2] * in[2];
}


void ImuStreamer::setDepthProfile(const rs2::stream_profile& depth) {
    try {
        if (!accelProfile_ || !depth) return;
        const rs2_extrinsics e = accelProfile_.get_extrinsics_to(depth);
        std::lock_guard<std::mutex> lock(mutex_);
        // rs2_extrinsics.rotation is column-major: to = R * from + t.
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) R_[r * 3 + c] = e.rotation[c * 3 + r];
        const float tr = std::max(-1.0f, std::min(3.0f, R_[0] + R_[4] + R_[8]));
        LOGI("D455 IMU -> depth rotation %.2f deg (factory extrinsics): [%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
             std::acos(std::max(-1.0f, std::min(1.0f, (tr - 1) / 2))) * kRadToDeg,
             R_[0], R_[1], R_[2], R_[3], R_[4], R_[5], R_[6], R_[7], R_[8]);
    } catch (const rs2::error& e) {
        LOGE("IMU -> depth extrinsics unavailable (%s); using identity", e.what());
    }
}


bool ImuStreamer::motionAt(double frameTsMs, float up[3], float gyro[3], double& matchMs, double maxGapMs,
                           double* accelDevMax) {
    std::lock_guard<std::mutex> lock(mutex_);
    matchMs = -1;
    if (!state_.running || accHist_.empty()) return false;
    // Accel samples around the frame time.
    size_t k = 0;
    while (k < accHist_.size() && accHist_[k].ts < frameTsMs) ++k;
    float g[3];
    if (k > 0 && k < accHist_.size()) {
        const Sample& a = accHist_[k - 1];
        const Sample& b = accHist_[k];
        const double span = b.ts - a.ts;
        const float w = span > 1e-6 ? static_cast<float>((frameTsMs - a.ts) / span) : 0.0f;
        for (int i = 0; i < 3; ++i) g[i] = a.v[i] + w * (b.v[i] - a.v[i]);
        matchMs = std::min(frameTsMs - a.ts, b.ts - frameTsMs);
    } else {
        const Sample& n = k == 0 ? accHist_.front() : accHist_.back();
        for (int i = 0; i < 3; ++i) g[i] = n.v[i];
        matchMs = std::fabs(n.ts - frameTsMs);
    }
    if (matchMs > maxGapMs) return false;
    const float gl = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if (gl < kMinG || gl > kMaxG) return false;
    const float u[3] = {g[0] / gl, g[1] / gl, g[2] / gl};
    rotate(u, up);
    // Gyro: mean over the frame interval (~33 ms before the frame), else the nearest.
    float w[3] = {0, 0, 0};
    int n = 0;
    double best = 1e18;
    const Sample* nearest = nullptr;
    for (const Sample& s : gyrHist_) {
        const double d = std::fabs(s.ts - frameTsMs);
        if (d < best) { best = d; nearest = &s; }
        if (s.ts > frameTsMs - 33.0 && s.ts <= frameTsMs + 2.0) {
            for (int i = 0; i < 3; ++i) w[i] += s.v[i];
            ++n;
        }
    }
    if (n > 0) for (float& x : w) x /= static_cast<float>(n);
    else if (nearest) for (int i = 0; i < 3; ++i) w[i] = nearest->v[i];
    rotate(w, gyro);
    if (accelDevMax) {
        double dev = 0;
        for (const RawSample& r : raw_)
            if (r.type == 1 && r.ts > frameTsMs - 33.0 && r.ts <= frameTsMs + 2.0)
                dev = std::max(dev, std::fabs(std::sqrt(r.v[0] * r.v[0] + r.v[1] * r.v[1] + r.v[2] * r.v[2]) - 9.81));
        *accelDevMax = dev;
    }
    return true;
}



void ImuStreamer::rawSamples(double after, double upTo, std::vector<RawSample>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const RawSample& r : raw_)
        if (r.ts > after && r.ts <= upTo) out.push_back(r);
}
