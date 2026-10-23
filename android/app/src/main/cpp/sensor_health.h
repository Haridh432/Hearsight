#ifndef HSV2_SENSOR_HEALTH_H
#define HSV2_SENSOR_HEALTH_H

/*
 * Sensor health (safety validator input): is the live picture trustworthy?
 * Checked every processed frame; the app must never keep showing walking
 * advice from a stale or blind sensor.
 *
 *   Fault (level 2, STOP): depth mostly empty for > blockedSec (lens covered,
 *     too close, depth stream dead); frame time not advancing (frozen
 *     frames); large lag between capture and processing.
 *   Degraded (level 1, no movement advice): a recent frame gap, slow
 *     detection model, the start-up warm-up (first warmupSec, longer while
 *     depth coverage is still below warmupDepth, at most warmupMaxSec).
 *     Phone: the very first frame (11 % valid depth) gave "STOP! Obstacle
 *     directly ahead 0.33 m" with a clear way.
 * Faults clear only after clearSec of good frames.
 *
 * Total loss (no frames at all, or the processing loop hanging) cannot be
 * seen from inside a frame: native-lib reports no_frames after a 1 s
 * timeout, and MainActivity flags telemetry older than 0.7 s as stale.
 */
enum class SensorFault { None = 0, Blocked, Frozen, Lag, FrameGap, SlowModel, Warmup };

struct SensorHealthInput {
    double tSec{0};            // processing time (steady clock), s
    double frameTimeMs{0};     // camera frame timestamp, ms
    double latencyMs{-1};      // capture -> processing, ms (-1 = unknown clock)
    float depthValid{1};       // fraction of valid depth pixels
    double inferenceMs{0};     // detection model time this frame (0 = not run)
};

struct SensorHealthResult {
    int level{0};              // 0 ok, 1 degraded, 2 fault
    SensorFault reason{SensorFault::None};
};

class SensorHealth {
public:
    float depthMinValid{0.10f};
    double blockedSec{0.5};
    double frozenSec{0.5};      // same frame timestamp for this long
    double lagMs{300};          // smoothed capture -> processing lag
    double gapSec{0.3};         // processing gap counted as a frame gap
    double gapHoldSec{1.0};     // degraded this long after a gap
    double slowModelMs{400};
    double clearSec{1.0};
    double lagGraceSec{3.0};    // lag not judged for this long after the first frame
    double warmupSec{1.0};
    float warmupDepth{0.25f};
    double warmupMaxSec{3.0};

    SensorHealthResult update(const SensorHealthInput& in);
    void reset() { *this = SensorHealth(); }
    const SensorHealthResult& current() const { return cur_; }

private:
    SensorHealthResult cur_;
    double lastT_{-1}, lastFrameMs_{-1}, frameSameSince_{-1};
    double blockedSince_{-1}, gapUntil_{-1}, goodSince_{-1};
    double lagEma_{-1};
    double firstT_{-1};
    bool warm_{false};
};

const char* sensorFaultName(SensorFault f);

#endif  // HSV2_SENSOR_HEALTH_H
