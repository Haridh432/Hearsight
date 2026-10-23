#ifndef HSV2_VIO_INIT_H
#define HSV2_VIO_INIT_H

#include <cmath>
#include <deque>
#include <memory>

#include <Eigen/Dense>

#include "core/VioManager.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/helper.h"
#include "utils/quat_ops.h"

/*
 * Starting OpenVINS on a head-mounted camera. OpenVINS's own static initialiser needs the
 * image to stay within ~2 px for ~2 s, which a head never does (recorded walks: 1.5-7 px,
 * 1768 failed attempts), and its dynamic initialiser is disabled on Android. So HearSight
 * starts the filter itself (VioManager::initialize_with_gt) from the D455 IMU:
 *
 *  - STILL:   gyro < stillGyro and |accel| within stillAccelDev of g for stillSec
 *             -> gravity = mean accel, gyro bias = mean gyro, velocity 0.
 *  - WALKING: walkAfterSec after the first frame without a still start, visual odometry tracking
 *             for walkVoSec at a walking speed (walkMinSpeed..walkMaxSpeed), gyro RMS < walkGyroRms
 *             over walkWindowSec -> gravity = mean accel over the window (step bounce averages
 *             out), velocity from VO (camera axes ~ IMU axes), looser covariance.
 *
 * Shared by the app (vio_thread.cpp) and the PC replay (tools/vio/replay_vio.cpp).
 */
class VioStarter {
public:
    enum class Mode { None, Still, Walking };

    float stillGyro{0.25f};        // rad/s, every sample in the window
    float stillAccelDev{0.6f};     // m/s^2 from 9.81, every sample
    double stillSec{0.5};
    double walkAfterSec{2.0};      // try the walking start this long after the first frame
    double walkVoSec{1.5};         // VO tracking continuously for this long
    float walkMinSpeed{0.15f}, walkMaxSpeed{2.0f};   // m/s, VO speed at the start (phone: a start
                                   // at 0.3 m/s VO diverged to 3 m/s within 2 s)
    double walkWindowSec{1.0};
    float walkGyroRms{1.0f};       // rad/s
    double retrySec{2.0};          // after a start, wait this long before starting again

    void reset() { win_.clear(); firstTs_ = -1; firstFrameTs_ = -1; voOkSince_ = -1; lastStartTs_ = -1e18; voOk_ = false; }

    // D455 sample (camera clock ms): type 1 accel m/s^2, 2 gyro rad/s, IMU axes.
    void addImu(double tsMs, int type, const float v[3]) {
        if (firstTs_ < 0) firstTs_ = tsMs;
        win_.push_back({tsMs, type, Eigen::Vector3d(v[0], v[1], v[2])});
        while (!win_.empty() && tsMs - win_.front().ts > 1000.0 * std::max(stillSec, walkWindowSec) + 50.0)
            win_.pop_front();
    }
    // Visual-odometry velocity (m/s, current camera axes) at the frame; ok = VO tracked.
    void setVo(bool ok, const float vel[3]) {
        voOk_ = ok;
        voVel_ = Eigen::Vector3d(vel[0], vel[1], vel[2]);
    }

    // Starts [sys] at frame time tsMs when a still or walking start applies. Returns the mode used.
    Mode tryStart(ov_msckf::VioManager& sys, double tsMs) {
        if (firstFrameTs_ < 0) firstFrameTs_ = tsMs;
        if (voOk_) { if (voOkSince_ < 0) voOkSince_ = tsMs; } else voOkSince_ = -1;
        if (sys.initialized() || tsMs - lastStartTs_ < retrySec * 1000.0 || firstTs_ < 0) return Mode::None;
        Eigen::Vector3d a, w;
        if (window(tsMs, stillSec, a, w, true)) return start(sys, tsMs, a, w, Eigen::Vector3d::Zero(), Mode::Still);
        const double voSpeed = voVel_.norm();
        if (tsMs - firstFrameTs_ >= walkAfterSec * 1000.0 && voOkSince_ >= 0 &&
            tsMs - voOkSince_ >= walkVoSec * 1000.0 && voSpeed >= walkMinSpeed && voSpeed <= walkMaxSpeed &&
            window(tsMs, walkWindowSec, a, w, false))
            return start(sys, tsMs, a, Eigen::Vector3d::Zero(), voVel_, Mode::Walking);
        return Mode::None;
    }

private:
    struct S { double ts; int type; Eigen::Vector3d v; };
    std::deque<S> win_;
    double firstTs_{-1}, firstFrameTs_{-1}, voOkSince_{-1}, lastStartTs_{-1e18};
    bool voOk_{false};
    Eigen::Vector3d voVel_{0, 0, 0};

    // Mean accel / gyro over the last [sec]; still = every sample within the still limits,
    // walking = gyro RMS below walkGyroRms. False if the window is not full or fails.
    bool window(double tsMs, double sec, Eigen::Vector3d& a, Eigen::Vector3d& w, bool still) const {
        a.setZero();
        w.setZero();
        int na = 0, nw = 0;
        double w2 = 0, oldest = tsMs;
        for (const S& s : win_) {
            if (tsMs - s.ts > sec * 1000.0 || s.ts > tsMs) continue;
            oldest = std::min(oldest, s.ts);
            if (s.type == 1) {
                if (still && std::fabs(s.v.norm() - 9.81) > stillAccelDev) return false;
                a += s.v;
                ++na;
            } else {
                if (still && s.v.norm() > stillGyro) return false;
                w += s.v;
                w2 += s.v.squaredNorm();
                ++nw;
            }
        }
        if (na < 50 || nw < 50 || tsMs - oldest < sec * 900.0) return false;   // window ~full
        a /= na;
        w /= nw;
        if (!still && std::sqrt(w2 / nw) > walkGyroRms) return false;
        return std::fabs(a.norm() - 9.81) < 1.5;
    }

    Mode start(ov_msckf::VioManager& sys, double tsMs, const Eigen::Vector3d& accel, const Eigen::Vector3d& bg,
               const Eigen::Vector3d& velI, Mode mode) {
        Eigen::Matrix3d R_GtoI;
        ov_init::InitializerHelper::gram_schmidt(accel, R_GtoI);
        Eigen::Matrix<double, 17, 1> st = Eigen::Matrix<double, 17, 1>::Zero();
        st(0) = tsMs / 1000.0;
        st.block(1, 0, 4, 1) = ov_core::rot_2_quat(R_GtoI);
        st.block(8, 0, 3, 1) = R_GtoI.transpose() * velI;   // v_IinG
        st.block(11, 0, 3, 1) = bg;
        sys.initialize_with_gt(st);
        // Initial uncertainty: initialize_with_gt assumes ground truth; widen it to what we know.
        auto state = sys.get_state();
        std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu};
        Eigen::MatrixXd cov = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(state->_imu->size(), state->_imu->size());
        const bool walking = mode == Mode::Walking;
        cov.block(0, 0, 3, 3) = std::pow(walking ? 0.09 : 0.035, 2) * Eigen::Matrix3d::Identity();   // ~5 / 2 deg
        cov.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity();                    // p
        cov.block(6, 6, 3, 3) = std::pow(walking ? 0.3 : 0.05, 2) * Eigen::Matrix3d::Identity();     // v
        cov.block(9, 9, 3, 3) = std::pow(walking ? 0.02 : 0.005, 2) * Eigen::Matrix3d::Identity();   // bg
        cov.block(12, 12, 3, 3) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();                   // ba
        ov_msckf::StateHelper::set_initial_covariance(state, cov, order);
        lastStartTs_ = tsMs;
        return mode;
    }
};

/*
 * Divergence check, per frame (app and replay): the filter is restarted when its speed is
 * implausible (> maxSpeed), it jumps (> maxStep in one frame), or it disagrees with visual
 * odometry by more than maxVoDiff for voDiffSec (recorded walk: after a 3.9 rad/s head turn
 * VIO ran away from 0.4 to 3 m/s in 2 s while VO stayed at walking pace).
 */
class VioHealth {
public:
    float maxSpeed{3.0f};     // m/s
    float maxStep{0.5f};      // m per frame
    float maxVoDiff{0.8f};    // m/s
    double voDiffSec{1.0};

    void reset() { haveLast_ = false; diffSince_ = -1; }
    // Returns false when the filter must be restarted; [why] says which check failed.
    bool check(double tSec, const Eigen::Vector3d& p, const Eigen::Vector3d& v, bool voOk, double voSpeed,
               const char*& why) {
        why = "";
        const double speed = v.norm();
        if (!std::isfinite(speed) || speed > maxSpeed) { why = "speed"; return false; }
        if (haveLast_ && (p - last_).norm() > maxStep) { why = "jump"; return false; }
        last_ = p;
        haveLast_ = true;
        if (voOk && std::fabs(speed - voSpeed) > maxVoDiff) {
            if (diffSince_ < 0) diffSince_ = tSec;
            if (tSec - diffSince_ > voDiffSec) { why = "disagrees with VO"; return false; }
        } else {
            diffSince_ = -1;
        }
        return true;
    }

private:
    Eigen::Vector3d last_{0, 0, 0};
    bool haveLast_{false};
    double diffSince_{-1};
};

#endif  // HSV2_VIO_INIT_H
