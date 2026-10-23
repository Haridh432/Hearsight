// Replays a walk recording (.hsr from the app's REC button) through the
// visual odometry on the PC and compares with what the phone computed.
//
//   replay_vo walk.hsr [--every N] [--csv out.csv]
// Tunables via environment: VO_POINTS, VO_ITERS, VO_FB, VO_MININL, VO_SMOOTH.
#include "recorder.h"
#include "vo.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static float envf(const char* n, float d) { const char* v = getenv(n); return v ? static_cast<float>(atof(v)) : d; }

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s walk.hsr [--every N] [--csv out.csv]\n", argv[0]); return 2; }
    int every = 30;
    const char* csv = nullptr;
    for (int i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--every") && i + 1 < argc) every = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csv = argv[++i];
    }
    RecReader rd;
    if (!rd.open(argv[1])) { fprintf(stderr, "cannot read %s\n", argv[1]); return 1; }
    const RecHeader& h = rd.h;
    printf("recording: colour %dx%d (grey %dx%d), depth %dx%d stored %dx%d, units %.5f\n", h.cam.width, h.cam.height,
           h.grayW, h.grayH, h.reg.dw, h.reg.dh, h.depW, h.depH, h.depthUnits);
    printf("colour fx %.1f fy %.1f cx %.1f cy %.1f | depth fx %.1f cx %.1f | extr t (%.4f %.4f %.4f)\n", h.cam.fx,
           h.cam.fy, h.cam.cx, h.cam.cy, h.reg.dfx, h.reg.dcx, h.reg.trans[0], h.reg.trans[1], h.reg.trans[2]);

    VisualOdometry vo;
    vo.setCamera(h.cam);
    vo.maxPoints = static_cast<int>(envf("VO_POINTS", static_cast<float>(vo.maxPoints)));
    vo.fbMaxErr = envf("VO_FB", vo.fbMaxErr);
    vo.minInliers = static_cast<int>(envf("VO_MININL", static_cast<float>(vo.minInliers)));
    vo.velSmooth = envf("VO_SMOOTH", vo.velSmooth);

    // Stored depth is every 2nd pixel: same points as step-2 registration
    // with halved depth intrinsics and step 1.
    DepthToColor reg = h.reg;
    reg.dfx *= 0.5f; reg.dfy *= 0.5f; reg.dcx *= 0.5f; reg.dcy *= 0.5f;
    reg.dw = h.depW; reg.dh = h.depH;

    FILE* out = csv ? fopen(csv, "w") : nullptr;
    if (out) fprintf(out, "t,ok,inliers,withDepth,tracked,vx,vy,vz,speed,turn_dps,x,y,z,phone_ok,phone_x,phone_y,phone_z\n");
    RecFrame fr;
    std::vector<uint8_t> gray;
    std::vector<uint16_t> depth;
    std::vector<float> half;
    int n = 0, ok = 0, phoneOk = 0;
    double t0 = -1, path = 0, maxD = 0, msSum = 0;
    float prev[3] = {0, 0, 0}, start[3] = {0, 0, 0};
    float pstart[3] = {0, 0, 0}, pmax = 0;
    while (rd.next(fr, gray, depth)) {
        if (t0 < 0) { t0 = fr.t; for (int i = 0; i < 3; ++i) pstart[i] = fr.pose[i]; }
        registerDepthHalf(depth.data(), h.depthUnits, reg, 1, half);
        struct Ctx { const std::vector<float>* m; int w, hh; } ctx{&half, h.cam.width, h.cam.height};
        VoResult r = vo.processGray(gray.data(), fr.t,
                                    [](void* c, float u, float v) {
                                        const Ctx* x = static_cast<const Ctx*>(c);
                                        return depthHalfAt(*x->m, x->w, x->hh, u, v);
                                    }, &ctx);
        const float* p = vo.poseT();
        if (n == 0) for (int i = 0; i < 3; ++i) { start[i] = p[i]; prev[i] = p[i]; }
        path += std::sqrt((p[0] - prev[0]) * (p[0] - prev[0]) + (p[1] - prev[1]) * (p[1] - prev[1]) +
                          (p[2] - prev[2]) * (p[2] - prev[2]));
        for (int i = 0; i < 3; ++i) prev[i] = p[i];
        const double d = std::sqrt((p[0] - start[0]) * (p[0] - start[0]) + (p[1] - start[1]) * (p[1] - start[1]) +
                                   (p[2] - start[2]) * (p[2] - start[2]));
        maxD = std::max(maxD, d);
        const float pd = std::sqrt((fr.pose[0] - pstart[0]) * (fr.pose[0] - pstart[0]) +
                                   (fr.pose[1] - pstart[1]) * (fr.pose[1] - pstart[1]) +
                                   (fr.pose[2] - pstart[2]) * (fr.pose[2] - pstart[2]));
        pmax = std::max(pmax, pd);
        ok += r.ok; phoneOk += fr.voOk; msSum += r.ms; ++n;
        if (out)
            fprintf(out, "%.3f,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f\n", fr.t - t0, r.ok,
                    r.inliers, r.withDepth, r.tracked, r.vel[0], r.vel[1], r.vel[2], r.speed, r.angRate * 57.2958f, p[0],
                    p[1], p[2], fr.voOk, fr.pose[0], fr.pose[1], fr.pose[2]);
        if (every > 0 && n % every == 0)
            printf("t %6.2f | ok %d inl %3d/%3d trk %3d | speed %.2f turn %5.1f deg/s | dist from start %.2f m (phone %.2f)\n",
                   fr.t - t0, r.ok, r.inliers, r.withDepth, r.tracked, r.speed, r.angRate * 57.2958f, d, pd);
    }
    printf("\n%d frames (%.1f s): ok %d (phone %d) | path %.2f m | max distance from start %.2f m (phone %.2f m) | %.1f ms/frame\n",
           n, n / 30.0, ok, phoneOk, path, maxD, pmax, n ? msSum / n : 0.0);
    if (out) fclose(out);
    return 0;
}
