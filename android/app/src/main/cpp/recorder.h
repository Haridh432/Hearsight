#ifndef HSV2_RECORDER_H
#define HSV2_RECORDER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vo.h"

/*
 * Walk recorder for replay on the PC (record once, re-run fixes offline).
 *
 * File: RecHeader, then one RecFrame + payload per frame:
 *   gray  : grayW*grayH bytes   (half-res grey, exactly what VO uses)
 *   depth : depW*depH uint16    (every 2nd depth pixel = what the depth
 *                                registration at step 2 reads)
 *   (version 3) RecSeg + seg.w*seg.h bytes: the phone's latest scene
 *                segmentation (NavClass per cell over the colour image),
 *                w = h = 0 when none is fresh (< 1 s)
 *   (version 4) RecD455 + n RecImuSample: every raw D455 gyro / accel sample
 *                since the previous frame (camera clock, IMU axes; accel m/s^2,
 *                gyro rad/s), for VIO replay. n = 0 when the IMU is off.
 * All little-endian, written by a background thread; frames are dropped (and
 * counted) if the disk can't keep up, never blocking the caller.
 */
#pragma pack(push, 1)
struct RecHeader {
    char magic[8];          // "HSVREC4\0" (3: no D455 IMU; 2: no segmentation; 1: no IMU fields in RecFrame)
    int32_t grayW, grayH;   // half-res colour
    int32_t depW, depH;     // depth subsampled by 2
    float depthUnits;
    VoCamera cam;           // full-res colour intrinsics
    DepthToColor reg;       // full-res depth/colour intrinsics + extrinsics
};
struct RecFrame {
    double t;               // capture time (s)
    // Phone results for comparison.
    uint8_t voOk;
    float vel[3];
    float angRate;
    int32_t inliers, tracked, withDepth;
    float pose[3];
    // Phone IMU at the frame time (version 2).
    uint8_t imuValid;
    float phoneUp[3];       // unit "up" in phone axes
    float gyro[3];          // rad/s, phone axes
};
// Version 3: segmentation after the depth payload.
struct RecSeg {
    uint16_t w, h;          // class grid size, 0 = none
    float age;              // s, frame time - segmentation input time
};
// Version 4: D455 IMU samples after the segmentation.
struct RecD455 {
    double frameTsMs;       // depth frame timestamp, camera clock (ms); -1 = none
    uint32_t n;             // number of RecImuSample that follow
};
struct RecImuSample {
    double ts;              // camera clock, ms
    uint8_t type;           // 1 = accel (m/s^2), 2 = gyro (rad/s)
    float v[3];             // D455 IMU axes (x right, y down, z forward)
};
#pragma pack(pop)

class Recorder {
public:
    ~Recorder() { stop(); }
    // Opens [path] and writes the header. False on error.
    bool start(const std::string& path, const RecHeader& h);
    void stop();
    bool active() const { return active_; }
    // Copies the frame into the queue (depth: full-res; subsampled here).
    void push(double t, const uint8_t* gray, const uint16_t* depthFull, const VoResult& r, const float pose[3],
              const RecImu& imu);

    // Latest scene segmentation (NavClass grid over the colour image); stored with
    // the following frames while it is fresh (< 1 s). Any thread.
    void setSegmentation(const uint8_t* cls, int w, int h, double frameTimeSec);
    // Colour snapshot (RGBA8, any size): saved half-size as <recording>.frames/<t ms>.ppm
    // by the writer thread, for scene-description tests on the PC. Any thread.
    void pushColor(double t, const uint8_t* rgba, int w, int h);
    // D455 IMU samples with after < ts <= upTo (camera clock, ms), appended to out.
    // Called from push() (the caller's thread). The first frame gets the 50 ms before it.
    using ImuSource = std::function<void(double after, double upTo, std::vector<RecImuSample>& out)>;
    void setImuSource(ImuSource src) { imuSource_ = std::move(src); }
    int frames() const { return frames_; }
    int dropped() const { return dropped_; }
    double megabytes() const { return bytes_ / 1e6; }
    const std::string& path() const { return path_; }

private:
    struct Item { RecFrame f; std::vector<uint8_t> gray; std::vector<uint16_t> depth; RecSeg seg{0, 0, 0}; std::vector<uint8_t> cls;
                  RecD455 d455{-1, 0}; std::vector<RecImuSample> imu; };
    ImuSource imuSource_;
    double lastImuTs_{-1};
    struct ColorItem { double t; int w, h; std::vector<uint8_t> rgb; };
    std::deque<ColorItem> colorQ_;
    std::mutex segM_;
    std::vector<uint8_t> segCls_;
    int segW_{0}, segH_{0};
    double segT_{-1};
    void loop();

    RecHeader h_{};
    std::FILE* file_{nullptr};
    std::string path_;
    std::thread thread_;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<Item> q_;
    bool quit_{false};
    std::atomic<bool> active_{false};
    std::atomic<int> frames_{0}, dropped_{0};
    std::atomic<long long> bytes_{0};
    static constexpr size_t kMaxQueue = 45;   // ~1.5 s
};

// Reader for the PC replay tool.
struct RecReader {
    std::FILE* f{nullptr};
    RecHeader h{};
    int version{0};
    bool open(const char* path);
    // Next frame; false at end of file. [seg] / [cls]: the recorded segmentation
    // (version 3; w = h = 0 when the frame has none or the file is older).
    // [d455] / [imu]: the D455 IMU samples since the previous frame (version 4; n = 0 else).
    bool next(RecFrame& fr, std::vector<uint8_t>& gray, std::vector<uint16_t>& depth,
              RecSeg* seg = nullptr, std::vector<uint8_t>* cls = nullptr,
              RecD455* d455 = nullptr, std::vector<RecImuSample>* imu = nullptr);
    ~RecReader() { if (f) std::fclose(f); }
};

#endif  // HSV2_RECORDER_H
