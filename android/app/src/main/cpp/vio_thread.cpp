#include "vio_thread.h"

#include <android/log.h>

#include <chrono>
#include <cmath>
#include <fstream>

#include <opencv2/core.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "vio_calib.h"
#include "vio_init.h"
#include "vio_config_gen.h"   // kVioEstimatorConfig, kVioImuChain (from tools/vio/config)

#define LOG_TAG "HSV2_VIO"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {

double nowSec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool writeText(const std::string& path, const char* text) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f << text;
    return static_cast<bool>(f);
}

}  // namespace

VioThread::VioThread() : starter_(std::make_unique<VioStarter>()), health_(std::make_unique<VioHealth>()) {}
VioThread::~VioThread() { stop(); }

bool VioThread::start(const std::string& workDir, const VoCamera& cam, int grayW, int grayH, const DepthToColor& reg,
                      ImuSource imu, std::string& error) {
    stop();
    workDir_ = workDir;
    imu_ = std::move(imu);
    grayW_ = grayW;
    grayH_ = grayH;
    if (!writeText(workDir_ + "/estimator_config.yaml", kVioEstimatorConfig) ||
        !writeText(workDir_ + "/kalibr_imu_chain.yaml", kVioImuChain) ||
        !writeVioCamchain(workDir_ + "/kalibr_imucam_chain.yaml", cam, grayW, grayH, reg)) {
        error = "cannot write the VIO config into " + workDir_;
        return false;
    }
    if (!rebuild(error)) return false;
    {
        std::lock_guard<std::mutex> lk(sm_);
        st_ = State{};
        st_.running = true;
    }
    lastImuTs_ = -1;
    haveAccel_ = false;
    starter_->reset();
    health_->reset();
    startT_ = -1;
    msSum_ = 0;
    msN_ = 0;
    msWindowT_ = -1;
    quit_ = false;
    havePending_ = false;
    running_ = true;
    thread_ = std::thread(&VioThread::loop, this);
    LOGI("VIO started (%dx%d grey, config %s)", grayW_, grayH_, workDir_.c_str());
    return true;
}

bool VioThread::rebuild(std::string& error) {
    try {
        auto parser = std::make_shared<ov_core::YamlParser>(workDir_ + "/estimator_config.yaml");
        std::string verbosity = "WARNING";
        parser->parse_config("verbosity", verbosity);
        ov_core::Printer::setPrintLevel(verbosity);
        ov_msckf::VioManagerOptions params;
        params.print_and_load(parser);
        params.use_multi_threading_subs = false;
        if (!parser->successful()) {
            error = "VIO config parse failed";
            return false;
        }
        sys_ = std::make_unique<ov_msckf::VioManager>(params);
    } catch (const std::exception& e) {
        error = std::string("VIO init failed: ") + e.what();
        return false;
    }
    return true;
}

void VioThread::stop() {
    if (!running_ && !thread_.joinable()) return;
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    running_ = false;
    sys_.reset();
    std::lock_guard<std::mutex> lk(sm_);
    st_.running = false;
}

void VioThread::submit(double camTsMs, const uint8_t* rgba, int width, int height, bool voOk, const float voVel[3]) {
    if (!running_) return;
    if (busy_) {
        std::lock_guard<std::mutex> lk(sm_);
        ++st_.dropped;
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_);
        pending_.assign(rgba, rgba + static_cast<size_t>(width) * height * 4);
        pendingTs_ = camTsMs;
        pendingVoOk_ = voOk;
        for (int i = 0; i < 3; ++i) pendingVoVel_[i] = voVel[i];
        pendingW_ = width;
        pendingH_ = height;
        havePending_ = true;
    }
    cv_.notify_one();
}

VioThread::State VioThread::state() {
    std::lock_guard<std::mutex> lk(sm_);
    return st_;
}

void VioThread::loop() {
    std::vector<uint8_t> rgba, gray;
    std::vector<VioImuSample> imu;
    while (true) {
        double ts;
        int w, h;
        bool voOk;
        float voVel[3];
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return quit_ || havePending_; });
            if (quit_) break;
            rgba.swap(pending_);
            ts = pendingTs_;
            voOk = pendingVoOk_;
            for (int i = 0; i < 3; ++i) voVel[i] = pendingVoVel_[i];
            w = pendingW_;
            h = pendingH_;
            havePending_ = false;
            busy_ = true;
        }
        const double t0 = nowSec();
        if (startT_ < 0) startT_ = t0;

        // IMU since the previous frame: OpenVINS wants gyro + accel at one time, so emit
        // at each gyro sample with the latest accel (both ~400 Hz).
        imu.clear();
        imu_(lastImuTs_ >= 0 ? lastImuTs_ : ts - 50.0, ts, imu);
        lastImuTs_ = ts;
        bool ok = true;
        try {
            for (const VioImuSample& s : imu) {
                starter_->addImu(s.ts, s.type, s.v);
                if (s.type == 1) {
                    for (int i = 0; i < 3; ++i) lastAccel_[i] = s.v[i];
                    haveAccel_ = true;
                } else if (s.type == 2 && haveAccel_) {
                    ov_core::ImuData m;
                    m.timestamp = s.ts / 1000.0;
                    m.wm << s.v[0], s.v[1], s.v[2];
                    m.am << lastAccel_[0], lastAccel_[1], lastAccel_[2];
                    sys_->feed_measurement_imu(m);
                }
            }
            // Start the filter ourselves (OpenVINS's static start never succeeds head-mounted).
            starter_->setVo(voOk, voVel);
            const VioStarter::Mode mode = starter_->tryStart(*sys_, ts);
            if (mode != VioStarter::Mode::None) {
                LOGI("VIO start (%s)", mode == VioStarter::Mode::Still ? "still" : "walking");
                std::lock_guard<std::mutex> lk(sm_);
                st_.startMode = mode == VioStarter::Mode::Still ? 1 : 2;
            }
            rgbaToHalfGray(rgba.data(), w, h, gray);
            ov_core::CameraData cam;
            cam.timestamp = ts / 1000.0;
            cam.sensor_ids.push_back(0);
            cam.images.push_back(cv::Mat(grayH_, grayW_, CV_8UC1, gray.data()).clone());
            cam.masks.push_back(cv::Mat::zeros(grayH_, grayW_, CV_8UC1));
            sys_->feed_measurement_camera(cam);
        } catch (const std::exception& e) {
            LOGE("VIO update failed: %s", e.what());
            ok = false;
        }

        const double ms = (nowSec() - t0) * 1000.0;
        State prev;
        {
            std::lock_guard<std::mutex> lk(sm_);
            prev = st_;
        }
        State s = prev;
        s.t = ts / 1000.0;
        s.frames++;
        s.msMax = std::max(s.msMax, ms);
        msSum_ += ms;
        ++msN_;
        if (msWindowT_ < 0) msWindowT_ = t0;
        if (t0 - msWindowT_ >= 1.0) {
            s.ms = msSum_ / msN_;
            msSum_ = 0;
            msN_ = 0;
            msWindowT_ = t0;
        }
        bool healthy = ok;
        if (ok && sys_->initialized()) {
            auto st = sys_->get_state();
            const Eigen::Vector3d p = st->_imu->pos(), v = st->_imu->vel();
            const Eigen::Vector4d q = st->_imu->quat();
            const char* why = "";
            const double voSpeed = std::sqrt(voVel[0] * voVel[0] + voVel[1] * voVel[1] + voVel[2] * voVel[2]);
            if (!health_->check(ts / 1000.0, p, v, voOk, voSpeed, why)) {
                LOGE("VIO unhealthy (%s: speed %.2f m/s, VO %.2f m/s): restarting the filter", why, v.norm(), voSpeed);
                healthy = false;
            } else {
                if (!prev.initialized) {
                    s.initAfterSec = t0 - startT_;
                    LOGI("VIO initialised after %.1f s", s.initAfterSec);
                }
                s.initialized = true;
                for (int i = 0; i < 3; ++i) { s.p[i] = static_cast<float>(p[i]); s.v[i] = static_cast<float>(v[i]); }
                for (int i = 0; i < 4; ++i) s.q[i] = static_cast<float>(q[i]);
            }
        }
        if (!healthy) {
            std::string err;
            if (!rebuild(err)) LOGE("VIO restart failed: %s", err.c_str());
            s.initialized = false;
            s.resets++;
            startT_ = nowSec();
            haveAccel_ = false;
            starter_->reset();
            health_->reset();
        }
        {
            std::lock_guard<std::mutex> lk(sm_);
            s.dropped = st_.dropped;   // counted by submit()
            st_ = s;
        }
        busy_ = false;
    }
}
