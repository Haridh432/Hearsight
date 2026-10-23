// Replays a HearSight walk recording (format v4: grey frames + raw D455 IMU samples)
// through OpenVINS on the PC, to tune VIO before it runs on the phone.
//
//   replay_vio walk.hsr outdir [--config tools/vio/config]
//
// Writes outdir/vio_traj.csv (t, initialised, VIO position/velocity, phone VO position)
// and prints: time to initialise, path length VIO vs VO, update time per frame.
// The camera calibration (outdir/kalibr_imucam_chain.yaml) is generated from the
// recording header: half-res colour intrinsics, no distortion, camera->IMU from the
// D455 depth->colour extrinsics with the IMU at the depth origin (IMU->depth rotation
// measured as identity on the phone).

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include "utils/helper.h"
#include "utils/quat_ops.h"
#include <deque>

#include "recorder.h"
#include "vio_calib.h"
#include "vio_init.h"

namespace {

bool copyFile(const std::string& from, const std::string& to) {
    std::ifstream in(from, std::ios::binary);
    std::ofstream out(to, std::ios::binary);
    if (!in || !out) return false;
    out << in.rdbuf();
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s walk.hsr outdir [--config dir]\n", argv[0]);
        return 2;
    }
    const std::string outdir = argv[2];
    std::string cfgdir = "tools/vio/config";
    bool extInit = false;   // start OpenVINS ourselves when nearly still (gravity + gyro bias, v = 0)
    bool fixOld = false;      // recordings before commit 70d35a7 (22-23 Oct 09:14): gyro 3.28x too large
    bool negAccel = false;    // recordings made with 70d35a7 (23 Oct 09:14 - fix): accel wrongly negated
    for (int i = 3; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc) cfgdir = argv[++i];
        else if (!std::strcmp(argv[i], "--fix-old-imu")) fixOld = true;
        else if (!std::strcmp(argv[i], "--negate-accel")) negAccel = true;
        else if (!std::strcmp(argv[i], "--external-init")) extInit = true;
    }

    RecReader rd;
    if (!rd.open(argv[1])) { std::fprintf(stderr, "cannot read %s\n", argv[1]); return 1; }
    if (rd.version < 4) { std::fprintf(stderr, "recording is v%d: no D455 IMU samples (need v4)\n", rd.version); return 1; }

    // Config: estimator + IMU noise from the repo, camchain from the recording.
    if (!copyFile(cfgdir + "/estimator_config.yaml", outdir + "/estimator_config.yaml") ||
        !copyFile(cfgdir + "/kalibr_imu_chain.yaml", outdir + "/kalibr_imu_chain.yaml") ||
        !writeVioCamchain(outdir + "/kalibr_imucam_chain.yaml", rd.h.cam, rd.h.grayW, rd.h.grayH, rd.h.reg)) {
        std::fprintf(stderr, "cannot write config into %s (does it exist?)\n", outdir.c_str());
        return 1;
    }
    auto parser = std::make_shared<ov_core::YamlParser>(outdir + "/estimator_config.yaml");
    std::string verbosity = "WARNING";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    ov_msckf::VioManagerOptions params;
    params.print_and_load(parser);
    params.use_multi_threading_subs = false;
    if (!parser->successful()) { std::fprintf(stderr, "config parse failed\n"); return 1; }
    auto sys = std::make_unique<ov_msckf::VioManager>(params);
    VioHealth health;   // same restart rules as the app (vio_init.h)
    int resets = 0;

    std::FILE* csv = std::fopen((outdir + "/vio_traj.csv").c_str(), "w");
    std::fprintf(csv, "t,init,px,py,pz,vx,vy,vz,vo_x,vo_y,vo_z,update_ms\n");

    RecFrame fr;
    std::vector<uint8_t> gray, cls;
    std::vector<uint16_t> depth;
    RecSeg sg{};
    RecD455 dh{};
    std::vector<RecImuSample> imu;
    double lastAccel[3] = {0, 0, 0};
    bool haveAccel = false;
    int frames = 0, framesWithImu = 0, initFrame = -1, imuSamples = 0;
    double t0 = -1, initT = -1, sumMs = 0, maxMs = 0;
    double vioLen = 0, voLen = 0, lastVio[3] = {0, 0, 0}, lastVo[3] = {0, 0, 0};
    bool haveVio = false, haveVo = false;
    VioStarter starter;   // --external-init (shared with the app, vio_init.h)

    while (rd.next(fr, gray, depth, &sg, &cls, &dh, &imu)) {
        ++frames;
        if (dh.frameTsMs < 0 || imu.empty()) continue;
        ++framesWithImu;
        if (t0 < 0) t0 = dh.frameTsMs / 1000.0;
        // OpenVINS wants gyro + accel at one time: emit at each gyro sample with the
        // latest accel (accel ~400 Hz, so at most ~2.5 ms old).
        if (fixOld)
            for (RecImuSample& m : imu)
                if (m.type == 2) for (float& x : m.v) x *= 0.3052f;
        if (negAccel)
            for (RecImuSample& m : imu)
                if (m.type == 1) for (float& x : m.v) x = -x;
        for (const RecImuSample& s : imu) {
            if (s.type == 1) {
                for (int i = 0; i < 3; ++i) lastAccel[i] = s.v[i];
                haveAccel = true;
            }
            starter.addImu(s.ts, s.type, s.v);
            if (s.type == 1) {
            } else if (s.type == 2 && haveAccel) {
                ov_core::ImuData m;
                m.timestamp = s.ts / 1000.0;
                m.wm << s.v[0], s.v[1], s.v[2];
                m.am << lastAccel[0], lastAccel[1], lastAccel[2];
                sys->feed_measurement_imu(m);
                ++imuSamples;
            }
        }
        if (extInit) {
            starter.setVo(fr.voOk, fr.vel);
            const VioStarter::Mode m = starter.tryStart(*sys, dh.frameTsMs);
            if (m != VioStarter::Mode::None)
                std::printf("external init (%s) at %.1f s\n", m == VioStarter::Mode::Still ? "still" : "walking",
                            dh.frameTsMs / 1000.0 - t0);
        }
        ov_core::CameraData cam;
        cam.timestamp = dh.frameTsMs / 1000.0;
        cam.sensor_ids.push_back(0);
        cam.images.push_back(cv::Mat(rd.h.grayH, rd.h.grayW, CV_8UC1, gray.data()).clone());
        cam.masks.push_back(cv::Mat::zeros(rd.h.grayH, rd.h.grayW, CV_8UC1));
        const auto a = std::chrono::steady_clock::now();
        sys->feed_measurement_camera(cam);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
        sumMs += ms;
        maxMs = std::max(maxMs, ms);

        bool init = sys->initialized();
        double p[3] = {0, 0, 0}, v[3] = {0, 0, 0};
        if (init) {
            if (initFrame < 0) { initFrame = frames; initT = cam.timestamp - t0; }
            auto st = sys->get_state();
            const Eigen::Vector3d pp = st->_imu->pos(), vv = st->_imu->vel();
            const char* why = "";
            const double voSpeed = std::sqrt(fr.vel[0] * fr.vel[0] + fr.vel[1] * fr.vel[1] + fr.vel[2] * fr.vel[2]);
            if (!health.check(cam.timestamp, pp, vv, fr.voOk, voSpeed, why)) {
                std::printf("restart at %.1f s (%s, speed %.2f m/s, VO %.2f m/s)\n", cam.timestamp - t0, why, vv.norm(), voSpeed);
                sys = std::make_unique<ov_msckf::VioManager>(params);
                starter.reset();
                health.reset();
                haveAccel = false;
                ++resets;
                init = false;
                haveVio = false;   // a new filter starts at 0: no jump in the path length
            } else {
                for (int i = 0; i < 3; ++i) { p[i] = pp[i]; v[i] = vv[i]; }
                if (haveVio) vioLen += std::sqrt((p[0] - lastVio[0]) * (p[0] - lastVio[0]) + (p[1] - lastVio[1]) * (p[1] - lastVio[1]) +
                                                 (p[2] - lastVio[2]) * (p[2] - lastVio[2]));
                for (int i = 0; i < 3; ++i) lastVio[i] = p[i];
                haveVio = true;
            }
        }
        if (fr.voOk) {
            if (haveVo) voLen += std::sqrt((fr.pose[0] - lastVo[0]) * (fr.pose[0] - lastVo[0]) +
                                           (fr.pose[1] - lastVo[1]) * (fr.pose[1] - lastVo[1]) +
                                           (fr.pose[2] - lastVo[2]) * (fr.pose[2] - lastVo[2]));
            for (int i = 0; i < 3; ++i) lastVo[i] = fr.pose[i];
            haveVo = true;
        }
        std::fprintf(csv, "%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n", cam.timestamp - t0, init ? 1 : 0,
                     p[0], p[1], p[2], v[0], v[1], v[2], fr.pose[0], fr.pose[1], fr.pose[2], ms);
    }
    std::fclose(csv);

    std::printf("frames %d (with D455 IMU %d), IMU samples fed %d\n", frames, framesWithImu, imuSamples);
    if (initFrame >= 0) std::printf("VIO initialised at frame %d (%.1f s)\n", initFrame, initT);
    else std::printf("VIO never initialised\n");
    std::printf("path length: VIO %.2f m, phone VO %.2f m, restarts %d\n", vioLen, voLen, resets);
    if (framesWithImu) std::printf("update time per frame: avg %.1f ms, max %.1f ms\n", sumMs / framesWithImu, maxMs);
    std::printf("trajectory: %s/vio_traj.csv\n", outdir.c_str());
    return 0;
}
