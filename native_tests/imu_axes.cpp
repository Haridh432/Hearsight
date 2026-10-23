// Which way do the D455 IMU axes point relative to the depth camera?
//
//   imu_axes walk.hsr          (recording format v4: raw D455 IMU samples per frame)
//
// Per frame, "up" is measured twice: from the depth image (the app's own floor fit,
// SafetyLayer without gravity) and from the accelerometer (mean of the frame's accel
// samples). On steady frames (low gyro) every signed axis permutation P (48) is scored:
// the angle between P * accel_up and the floor normal. The right mapping gives a few
// degrees; a wrong sign or order shows up as tens of degrees.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>

#include "recorder.h"
#include "safety.h"
#include "vo.h"
#include "gravity_calib.h"

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: %s walk.hsr [--fix-old-imu] [--negate-accel]\n", argv[0]); return 2; }
    // --fix-old-imu: recordings before commit 70d35a7 (gyro 3.28x too large).
    // --negate-accel: recordings made with 70d35a7 (accel wrongly negated, 23 Oct 09:14 - fix).
    bool fixOld = false, negAccel = false;
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--fix-old-imu") fixOld = true;
        if (std::string(argv[i]) == "--negate-accel") negAccel = true;
    }
    RecReader rd;
    if (!rd.open(argv[1]) || rd.version < 4) { std::fprintf(stderr, "need a v4 recording\n"); return 1; }
    const RecHeader& h = rd.h;
    const DepthIntrinsics din{h.reg.dfx * h.depW / h.reg.dw, h.reg.dfy * h.depH / h.reg.dh,
                              h.reg.dcx * h.depW / h.reg.dw, h.reg.dcy * h.depH / h.reg.dh, h.depW, h.depH};
    SafetyLayer safety;   // no gravity set: floor from depth only
    safety.sampleStep = 2;   // stored depth is half res (as replay_map)

    // Visual odometry on the recorded frames (as replay_map): camera rotation per frame.
    VisualOdometry vo;
    vo.setCamera(h.cam);
    DepthToColor reg = h.reg;
    reg.dfx *= 0.5f; reg.dfy *= 0.5f; reg.dcx *= 0.5f; reg.dcy *= 0.5f;
    reg.dw = h.depW; reg.dh = h.depH;
    std::vector<float> half;
    float Rprev[9];
    bool havePrev = false;
    double prevTs = -1;
    struct GPair { double vo[3], gyro[3]; };
    std::vector<GPair> gpairs;
    struct Pair { float imu[3], floor[3]; };
    std::vector<Pair> pairs;
    RecFrame fr; std::vector<uint8_t> gray, cls; std::vector<uint16_t> depth; RecSeg sg{}; RecD455 dh{};
    std::vector<RecImuSample> imu;
    int frames = 0, withImu = 0, floorFrames = 0;
    while (rd.next(fr, gray, depth, &sg, &cls, &dh, &imu)) {
        ++frames;
        if (fixOld)
            for (RecImuSample& m : imu)
                if (m.type == 2) for (float& x : m.v) x *= 0.3052f;
        if (negAccel)
            for (RecImuSample& m : imu)
                if (m.type == 1) for (float& x : m.v) x = -x;
        const SafetyResult s = safety.process(depth.data(), din, h.depthUnits, fr.t, nullptr);
        registerDepthHalf(depth.data(), h.depthUnits, reg, 1, half);
        struct Ctx { const std::vector<float>* m; int w, hh; } ctx{&half, h.cam.width, h.cam.height};
        const VoResult r = vo.processGray(gray.data(), fr.t,
                                          [](void* c, float u, float v) {
                                              const Ctx* x = static_cast<const Ctx*>(c);
                                              return depthHalfAt(*x->m, x->w, x->hh, u, v);
                                          }, &ctx);
        const float* Rc = vo.poseR();
        if (havePrev && r.ok && dh.frameTsMs > prevTs && !imu.empty()) {
            // R_rel = Rprev^T * Rcur (camera -> world poses): rotation in the previous camera frame.
            double Rr[9];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) {
                    double v = 0;
                    for (int k = 0; k < 3; ++k) v += Rprev[k * 3 + i] * Rc[k * 3 + j];
                    Rr[i * 3 + j] = v;
                }
            const double tr = std::max(-1.0, std::min(3.0, Rr[0] + Rr[4] + Rr[8]));
            const double ang = std::acos(std::max(-1.0, std::min(1.0, (tr - 1) / 2)));
            const double dt = (dh.frameTsMs - prevTs) / 1000.0;
            if (ang > 1e-4 && dt > 0.01 && dt < 0.1) {
                const double sn = 2 * std::sin(ang);
                const double ax3[3] = {(Rr[7] - Rr[5]) / sn, (Rr[2] - Rr[6]) / sn, (Rr[3] - Rr[1]) / sn};
                GPair gp{};
                for (int i = 0; i < 3; ++i) gp.vo[i] = ax3[i] * ang / dt;
                int ng = 0;
                for (const RecImuSample& m : imu)
                    if (m.type == 2 && m.ts > prevTs && m.ts <= dh.frameTsMs) { for (int i = 0; i < 3; ++i) gp.gyro[i] += m.v[i]; ++ng; }
                if (ng) {
                    for (double& x : gp.gyro) x /= ng;
                    const double wv = std::sqrt(gp.vo[0] * gp.vo[0] + gp.vo[1] * gp.vo[1] + gp.vo[2] * gp.vo[2]);
                    if (wv > 0.4) gpairs.push_back(gp);   // clear head rotation
                }
            }
        }
        if (r.ok) { std::copy(Rc, Rc + 9, Rprev); havePrev = true; prevTs = dh.frameTsMs; }
        else havePrev = false;
        if (imu.empty()) continue;
        ++withImu;
        double a[3] = {0, 0, 0}, w2 = 0;
        int na = 0, nw = 0;
        for (const RecImuSample& m : imu) {
            if (m.type == 1) { for (int i = 0; i < 3; ++i) a[i] += m.v[i]; ++na; }
            else { w2 += m.v[0] * m.v[0] + m.v[1] * m.v[1] + m.v[2] * m.v[2]; ++nw; }
        }
        if (!na || !s.floorFound) continue;
        ++floorFrames;
        const double wRms = nw ? std::sqrt(w2 / nw) : 9;
        const double g = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / na;
        // Steady frames only (thresholds adjustable for testing the calibration rules).
        static const double kGyro = getenv("STEADY_GYRO") ? atof(getenv("STEADY_GYRO")) : 0.3;
        static const double kAcc = getenv("STEADY_ACC") ? atof(getenv("STEADY_ACC")) : 1.3;
        if (wRms > kGyro || std::fabs(g - 9.81) > kAcc) continue;
        Pair p;
        const double al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        for (int i = 0; i < 3; ++i) { p.imu[i] = static_cast<float>(a[i] / al); p.floor[i] = s.floorNormal[i]; }
        pairs.push_back(p);
    }
    std::printf("frames %d, with IMU %d, floor found %d, steady pairs %zu\n", frames, withImu, floorFrames, pairs.size());
    if (pairs.size() < 10) { std::printf("too few steady frames\n"); return 1; }

    // Score all signed permutations: cam = P * imu.
    struct Score { double mean, med; std::array<int, 3> perm; std::array<int, 3> sign; };
    std::vector<Score> scores;
    const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    for (const auto& pm : perms)
        for (int sm = 0; sm < 8; ++sm) {
            const int sg3[3] = {(sm & 1) ? -1 : 1, (sm & 2) ? -1 : 1, (sm & 4) ? -1 : 1};
            std::vector<double> ang;
            for (const Pair& p : pairs) {
                double v[3];
                for (int i = 0; i < 3; ++i) v[i] = sg3[i] * p.imu[pm[i]];
                const double d = v[0] * p.floor[0] + v[1] * p.floor[1] + v[2] * p.floor[2];
                ang.push_back(std::acos(std::max(-1.0, std::min(1.0, d))) * 57.2957795);
            }
            double sum = 0;
            for (double x : ang) sum += x;
            std::nth_element(ang.begin(), ang.begin() + ang.size() / 2, ang.end());
            scores.push_back({sum / ang.size(), ang[ang.size() / 2], {pm[0], pm[1], pm[2]}, {sg3[0], sg3[1], sg3[2]}});
        }
    std::sort(scores.begin(), scores.end(), [](const Score& a, const Score& b) { return a.med < b.med; });
    const char* ax = "xyz";
    std::printf("best mappings (camera axis = sign * IMU axis):\n");
    for (int k = 0; k < 5; ++k) {
        const Score& s = scores[k];
        std::printf("  cam x = %+d*imu %c, cam y = %+d*imu %c, cam z = %+d*imu %c : median %.1f deg, mean %.1f deg%s\n",
                    s.sign[0], ax[s.perm[0]], s.sign[1], ax[s.perm[1]], s.sign[2], ax[s.perm[2]], s.med, s.mean,
                    (s.perm == std::array<int, 3>{0, 1, 2} && s.sign == std::array<int, 3>{1, 1, 1}) ? "   <- as used now" : "");
    }
    for (const Score& s : scores)
        if (s.perm == std::array<int, 3>{0, 1, 2} && s.sign == std::array<int, 3>{1, 1, 1})
            std::printf("identity (as used now): median %.1f deg, mean %.1f deg\n", s.med, s.mean);
    {
        double se = 0;
        for (const Pair& p : pairs) {
            const double d = p.imu[0] * p.floor[0] + p.imu[1] * p.floor[1] + p.imu[2] * p.floor[2];
            const double a = std::acos(std::max(-1.0, std::min(1.0, d))) * 57.2957795;
            se += a * a;
        }
        std::printf("identity RMS angle %.1f deg over %zu pairs\n", std::sqrt(se / pairs.size()), pairs.size());
        // The app's calibration (GravityCalib, 5 Hz sampling) on these pairs, in recording order.
        GravityCalib cal;
        double t = 0;
        for (const Pair& p : pairs) { cal.add(p.imu, p.floor, t); t += 0.25; }
        std::printf("GravityCalib: ok %d, residual %.2f deg, inliers %.0f%%, spread %.1f deg, %d samples\n", cal.ok(),
                    cal.residualDeg(), 100 * cal.inlierFrac(), cal.spreadDeg(), cal.samples());
    }
    // Floor normal sign check: sample of raw vectors.
    for (size_t k = 0; k < pairs.size() && k < 3; ++k)
        std::printf("  sample: accel-up (%.2f %.2f %.2f)  floor-up (%.2f %.2f %.2f)\n", pairs[k].imu[0], pairs[k].imu[1],
                    pairs[k].imu[2], pairs[k].floor[0], pairs[k].floor[1], pairs[k].floor[2]);
    // Gyro vs visual-odometry rotation rate (all axes exercised by head turns).
    std::printf("\ngyro: %zu frames with a clear head rotation (> 0.4 rad/s by VO)\n", gpairs.size());
    if (gpairs.size() >= 20) {
        std::vector<Score> gs;
        for (const auto& pm : perms)
            for (int sm = 0; sm < 8; ++sm) {
                const int sg3[3] = {(sm & 1) ? -1 : 1, (sm & 2) ? -1 : 1, (sm & 4) ? -1 : 1};
                std::vector<double> ang;
                double ratio = 0;
                for (const GPair& p : gpairs) {
                    double v[3];
                    for (int i = 0; i < 3; ++i) v[i] = sg3[i] * p.gyro[pm[i]];
                    const double lv = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                    const double lo = std::sqrt(p.vo[0] * p.vo[0] + p.vo[1] * p.vo[1] + p.vo[2] * p.vo[2]);
                    const double d = (v[0] * p.vo[0] + v[1] * p.vo[1] + v[2] * p.vo[2]) / (lv * lo + 1e-12);
                    ang.push_back(std::acos(std::max(-1.0, std::min(1.0, d))) * 57.2957795);
                    ratio += lv / lo;
                }
                double sum = 0;
                for (double x : ang) sum += x;
                std::nth_element(ang.begin(), ang.begin() + ang.size() / 2, ang.end());
                gs.push_back({sum / ang.size(), ang[ang.size() / 2], {pm[0], pm[1], pm[2]}, {sg3[0], sg3[1], sg3[2]}});
                (void)ratio;
            }
        std::sort(gs.begin(), gs.end(), [](const Score& a, const Score& b) { return a.med < b.med; });
        std::printf("gyro best mappings (camera axis = sign * IMU axis):\n");
        for (int k = 0; k < 5; ++k) {
            const Score& s = gs[k];
            std::printf("  cam x = %+d*imu %c, cam y = %+d*imu %c, cam z = %+d*imu %c : median %.1f deg, mean %.1f deg\n",
                        s.sign[0], ax[s.perm[0]], s.sign[1], ax[s.perm[1]], s.sign[2], ax[s.perm[2]], s.med, s.mean);
        }
        for (const Score& s : gs)
            if (s.perm == std::array<int, 3>{0, 1, 2} && s.sign == std::array<int, 3>{1, 1, 1})
                std::printf("gyro identity (as used now): median %.1f deg, mean %.1f deg\n", s.med, s.mean);
        // Magnitude: |gyro| / |VO rate| (scale check, ~1 if the 0.1 deg/s units are right).
        std::vector<double> rr;
        for (const GPair& p : gpairs) {
            const double lg = std::sqrt(p.gyro[0] * p.gyro[0] + p.gyro[1] * p.gyro[1] + p.gyro[2] * p.gyro[2]);
            const double lo = std::sqrt(p.vo[0] * p.vo[0] + p.vo[1] * p.vo[1] + p.vo[2] * p.vo[2]);
            rr.push_back(lg / lo);
        }
        std::nth_element(rr.begin(), rr.begin() + rr.size() / 2, rr.end());
        std::printf("gyro scale |gyro| / |VO rate|: median %.2f\n", rr[rr.size() / 2]);
    }
    return 0;
}
