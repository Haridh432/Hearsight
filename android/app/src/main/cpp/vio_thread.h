#ifndef HSV2_VIO_THREAD_H
#define HSV2_VIO_THREAD_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vo.h"

namespace ov_msckf {
class VioManager;
}
class VioStarter;
class VioHealth;

/*
 * OpenVINS (GPL-3.0) visual-inertial odometry on its own thread.
 *
 * Inputs: the colour frame (RGBA, camera-clock timestamp) and the raw D455 gyro/accel
 * samples, pulled from [ImuSource] up to each frame's time. Latest frame only: a frame
 * arriving while the previous one is processed replaces it (the IMU stays continuous).
 *
 * Shadow mode for now: the pose is reported (telemetry, log) but the map and planner
 * still use VO. Start-up and health checks are shared with the PC replay (vio_init.h): the
 * filter is started from the D455 IMU when the user is nearly still or walking steadily, and
 * rebuilt when it runs away, jumps or disagrees with visual odometry.
 *
 * Settings: tools/vio/config (compiled in, see vio_config_gen.h) + the camera calibration
 * generated from the stream intrinsics (vio_calib.h), written to [workDir].
 */
struct VioImuSample { double ts; int type; float v[3]; };   // ts camera ms, type 1 accel / 2 gyro

class VioThread {
public:
    using ImuSource = std::function<void(double after, double upTo, std::vector<VioImuSample>& out)>;

    struct State {
        bool running{false};
        bool initialized{false};
        double t{-1};          // last processed frame (camera clock, s)
        float p[3]{}, v[3]{};  // IMU position / velocity in the VIO world (gravity-aligned)
        float q[4]{0, 0, 0, 1};
        double ms{0}, msMax{0};   // update time per frame (avg over the last second), max
        int frames{0}, dropped{0}, resets{0};
        double initAfterSec{-1};  // seconds from start to the last initialisation
        int startMode{0};         // how the filter last started: 0 not yet, 1 still, 2 walking
    };

    VioThread();
    ~VioThread();   // in the .cpp: VioManager is incomplete here
    // Writes the config into workDir and starts the worker. False on error ([error] says why).
    bool start(const std::string& workDir, const VoCamera& cam, int grayW, int grayH, const DepthToColor& reg,
               ImuSource imu, std::string& error);
    void stop();
    bool running() const { return running_; }
    // Colour frame (RGBA8, full res) + the latest visual-odometry velocity (m/s, camera axes;
    // used to start the filter while walking and to catch divergence). Copied only when the
    // worker is free (else dropped).
    void submit(double camTsMs, const uint8_t* rgba, int width, int height, bool voOk, const float voVel[3]);
    State state();


private:
    void loop();
    bool rebuild(std::string& error);

    std::string workDir_;
    ImuSource imu_;
    int grayW_{0}, grayH_{0};

    std::unique_ptr<ov_msckf::VioManager> sys_;
    std::thread thread_;
    std::mutex m_;
    std::condition_variable cv_;
    bool quit_{false};
    bool havePending_{false};
    double pendingTs_{0};
    bool pendingVoOk_{false};
    float pendingVoVel_[3]{0, 0, 0};
    std::unique_ptr<VioStarter> starter_;
    std::unique_ptr<VioHealth> health_;
    int pendingW_{0}, pendingH_{0};
    std::vector<uint8_t> pending_;
    std::atomic<bool> busy_{false};
    std::atomic<bool> running_{false};

    std::mutex sm_;
    State st_;
    double lastImuTs_{-1};
    double lastAccel_[3]{0, 0, 0};
    bool haveAccel_{false};
    double startT_{-1};
    double msSum_{0};
    int msN_{0};
    double msWindowT_{-1};
};

#endif  // HSV2_VIO_THREAD_H
