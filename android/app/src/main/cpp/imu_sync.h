#ifndef HSV2_IMU_SYNC_H
#define HSV2_IMU_SYNC_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

/*
 * Phone IMU read natively (NDK ASensorManager) with the sensor hardware
 * timestamps, which are CLOCK_BOOTTIME nanoseconds. Camera frames are put on
 * the same clock (see frameBootNs in native-lib.cpp), so the IMU state at a
 * frame's capture time can be interpolated from the ring buffers.
 *
 * Phone axes: x right, y up along the screen, z out of the screen.
 */
struct ImuAtFrame {
    bool valid{false};
    float gravity[3]{};   // m/s^2, interpolated at the frame time
    float gyro[3]{};      // rad/s, interpolated at the frame time
    double gapMs{0.0};    // distance to the nearest real gyro sample
};

class PhoneImuSync {
public:
    bool start();
    void stop();

    // IMU state at boottime [tNs]. valid=false if [tNs] is outside the buffer
    // (e.g. IMU not running yet, or the frame is older than the buffer).
    ImuAtFrame at(int64_t tNs);

    bool running() const { return running_; }

private:
    struct Sample { int64_t t; float v[3]; };

    // Fixed-size ring of the last ~4 s per sensor (at ~200-250 Hz).
    struct Ring {
        static constexpr size_t kSize = 1024;
        std::vector<Sample> buf = std::vector<Sample>(kSize);
        size_t head = 0;   // next write slot
        size_t count = 0;
        void push(const Sample& s);
        // Linear interpolation at t; returns false if t is out of range.
        bool interp(int64_t t, float out[3], double* gapMs) const;
    };

    void loop();

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopRequested_{false};
    std::mutex mutex_;
    Ring gyro_, gravity_;
};

// Current CLOCK_BOOTTIME in nanoseconds (same clock as sensor timestamps).
int64_t bootTimeNs();

#endif  // HSV2_IMU_SYNC_H
