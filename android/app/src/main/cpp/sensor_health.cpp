#include "sensor_health.h"

const char* sensorFaultName(SensorFault f) {
    static const char* n[] = {"none", "blocked", "frozen", "lag", "gap", "slow_model", "warmup"};
    return n[static_cast<int>(f)];
}

SensorHealthResult SensorHealth::update(const SensorHealthInput& in) {
    const double t = in.tSec;
    if (firstT_ < 0) firstT_ = t;
    // Frame gap: time since the previous processed frame.
    if (lastT_ >= 0 && t - lastT_ > gapSec) gapUntil_ = t + gapHoldSec;
    lastT_ = t;
    // Frozen: the camera keeps returning the same frame.
    if (lastFrameMs_ >= 0 && in.frameTimeMs == lastFrameMs_) {
        if (frameSameSince_ < 0) frameSameSince_ = t;
    } else {
        frameSameSince_ = -1;
    }
    lastFrameMs_ = in.frameTimeMs;
    // Blocked: depth mostly empty.
    if (in.depthValid < depthMinValid) {
        if (blockedSince_ < 0) blockedSince_ = t;
    } else {
        blockedSince_ = -1;
    }
    // Lag (only when the camera clock maps to the system clock).
    if (in.latencyMs >= 0 && t - firstT_ >= lagGraceSec) lagEma_ = lagEma_ < 0 ? in.latencyMs : lagEma_ + 0.2 * (in.latencyMs - lagEma_);

    SensorHealthResult r;
    if (blockedSince_ >= 0 && t - blockedSince_ >= blockedSec) { r.level = 2; r.reason = SensorFault::Blocked; }
    else if (frameSameSince_ >= 0 && t - frameSameSince_ >= frozenSec) { r.level = 2; r.reason = SensorFault::Frozen; }
    else if (lagEma_ > lagMs) { r.level = 2; r.reason = SensorFault::Lag; }
    else if (t < gapUntil_) { r.level = 1; r.reason = SensorFault::FrameGap; }
    else if (in.inferenceMs > slowModelMs) { r.level = 1; r.reason = SensorFault::SlowModel; }
    // Start-up warm-up (auto exposure, depth filling in, floor fit settling).
    if (!warm_) {
        const double age = t - firstT_;
        warm_ = age >= warmupMaxSec || (age >= warmupSec && in.depthValid >= warmupDepth);
        if (!warm_ && r.level == 0) { r.level = 1; r.reason = SensorFault::Warmup; }
    }

    // A fault clears only after clearSec of fully good frames.
    if (r.level == 0) { if (goodSince_ < 0) goodSince_ = t; }
    else goodSince_ = -1;
    if (cur_.level == 2 && r.level < 2 && (goodSince_ < 0 || t - goodSince_ < clearSec)) return cur_;
    cur_ = r;
    return cur_;
}
