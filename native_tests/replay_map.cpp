// Replays a walk recording through odometry + safety + dynamic map + gravity
// calibration (as on the phone) and writes top-down map snapshots.
//
//   replay_map walk.hsr outdir [--every N]
// Writes outdir/map_XXXX.ppm (map + timeline text in stdout).
#include "gravity_calib.h"
#include "local_map.h"
#include "nav_classes.h"
#include "nav_plan.h"
#include "view3d.h"
#include "recorder.h"
#include "safety.h"
#include "vo.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static void writePpm(const std::string& path, const std::vector<uint8_t>& rgba, int w, int h) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int i = 0; i < w * h; ++i) fwrite(&rgba[i * 4], 1, 3, f);
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s walk.hsr outdir [--every N]\n", argv[0]); return 2; }
    const std::string outdir = argv[2];
    int every = 30;
    for (int i = 3; i < argc; ++i)
        if (!strcmp(argv[i], "--every") && i + 1 < argc) every = atoi(argv[++i]);
    RecReader rd;
    if (!rd.open(argv[1])) { fprintf(stderr, "cannot read %s\n", argv[1]); return 1; }
    const RecHeader& h = rd.h;

    VisualOdometry vo;
    vo.setCamera(h.cam);
    DepthToColor reg = h.reg;   // stored depth = every 2nd pixel
    reg.dfx *= 0.5f; reg.dfy *= 0.5f; reg.dcx *= 0.5f; reg.dcy *= 0.5f;
    reg.dw = h.depW; reg.dh = h.depH;
    // Safety at full res uses step 4 = step 2 on the stored half-res depth.
    SafetyLayer safety;
    safety.sampleStep = 2;
    const DepthIntrinsics din{reg.dfx, reg.dfy, reg.dcx, reg.dcy, reg.dw, reg.dh};
    LocalMap map;
    GravityCalib cal;
    // depth -> colour rotation (row-major) for the depth camera pose.
    float Rdc[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Rdc[i * 3 + j] = h.reg.rot[j * 3 + i];

    RecFrame fr;
    std::vector<uint8_t> gray;
    std::vector<uint16_t> depth;
    std::vector<float> half;
    std::vector<uint8_t> img(200 * 200 * 4);
    int n = 0, stops = 0, cautions = 0, mapCautions = 0, snap = 0;
    double t0 = -1;
    // Recorded scene segmentation (version 3 files): fed to the safety layer and the
    // map exactly as on the phone (fresh maps only, < 0.3 s).
    RecSeg sg{0, 0, 0};
    std::vector<uint8_t> cls, roles;
    int segFrames = 0, semUsed = 0;
    struct SegCtx { const std::vector<uint8_t>* cls; int gw, gh, cw, ch; float sx, sy, ox, oy; } sctx{};
    sctx.cw = h.cam.width; sctx.ch = h.cam.height;
    sctx.sx = h.cam.fx / reg.dfx; sctx.sy = h.cam.fy / reg.dfy;
    sctx.ox = h.cam.cx - reg.dcx * sctx.sx; sctx.oy = h.cam.cy - reg.dcy * sctx.sy;
    while (rd.next(fr, gray, depth, &sg, &cls)) {
        if (t0 < 0) t0 = fr.t;
        const bool segOk = sg.w > 0 && sg.h > 0 && sg.age < 0.3f && cls.size() == static_cast<size_t>(sg.w) * sg.h;
        SemanticInput sem;
        if (segOk) {
            ++segFrames;
            roles.resize(cls.size());
            for (size_t i = 0; i < cls.size(); ++i) roles[i] = navToSemRole(cls[i]);
            sem.roles = roles.data();
            sem.gridW = sg.w; sem.gridH = sg.h;
            for (int i = 0; i < 9; ++i) sem.rot[i] = h.reg.rot[i];
            for (int i = 0; i < 3; ++i) sem.trans[i] = h.reg.trans[i];
            sem.fx = h.cam.fx; sem.fy = h.cam.fy; sem.ppx = h.cam.cx; sem.ppy = h.cam.cy;
            sem.colorW = h.cam.width; sem.colorH = h.cam.height;
            sctx.cls = &cls; sctx.gw = sg.w; sctx.gh = sg.h;
        }
        registerDepthHalf(depth.data(), h.depthUnits, reg, 1, half);
        struct Ctx { const std::vector<float>* m; int w, hh; } ctx{&half, h.cam.width, h.cam.height};
        VoResult r = vo.processGray(gray.data(), fr.t,
                                    [](void* c, float u, float v) {
                                        const Ctx* x = static_cast<const Ctx*>(c);
                                        return depthHalfAt(*x->m, x->w, x->hh, u, v);
                                    }, &ctx);
        safety.setEgoVelocity(r.ok, r.vel);
        SafetyResult s = safety.process(depth.data(), din, h.depthUnits, fr.t, segOk ? &sem : nullptr);
        if (s.semanticUsed) ++semUsed;
        if (fr.imuValid && s.floorFound) {
            const float g = std::sqrt(fr.gyro[0] * fr.gyro[0] + fr.gyro[1] * fr.gyro[1] + fr.gyro[2] * fr.gyro[2]);
            // Copy out of the packed recording struct (alignment).
            const float up[3] = {fr.phoneUp[0], fr.phoneUp[1], fr.phoneUp[2]};
            if (g < 0.5f) cal.add(up, s.floorNormal, fr.t);
        }
        float mapAhead = -1, mapHazard = -1;
        if (r.ok && s.floorStable) {
            const float* R = vo.poseR();
            const float* T = vo.poseT();
            MapInput mi;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j)
                    mi.pose.R[i * 3 + j] = R[i * 3] * Rdc[j] + R[i * 3 + 1] * Rdc[3 + j] + R[i * 3 + 2] * Rdc[6 + j];
                mi.pose.t[i] = R[i * 3] * h.reg.trans[0] + R[i * 3 + 1] * h.reg.trans[1] + R[i * 3 + 2] * h.reg.trans[2] + T[i];
            }
            mi.depth = depth.data(); mi.width = reg.dw; mi.height = reg.dh;
            mi.fx = reg.dfx; mi.fy = reg.dfy; mi.cx = reg.dcx; mi.cy = reg.dcy;
            mi.units = h.depthUnits; mi.step = 2;
            for (int i = 0; i < 3; ++i) mi.floorN[i] = s.floorNormal[i];
            mi.floorD = s.cameraHeight;
            mi.tSec = fr.t;
            if (segOk) {
                mi.semantic = [](void* c, int u, int v) -> uint8_t {
                    const SegCtx* m = static_cast<const SegCtx*>(c);
                    const int gx = static_cast<int>((u * m->sx + m->ox) * m->gw / m->cw);
                    const int gy = static_cast<int>((v * m->sy + m->oy) * m->gh / m->ch);
                    if (gx < 0 || gy < 0 || gx >= m->gw || gy >= m->gh) return 255;
                    return (*m->cls)[static_cast<size_t>(gy) * m->gw + gx];
                };
                mi.semanticCtx = &sctx;
            }
            map.integrate(mi);
            float vw[3];
            for (int i = 0; i < 3; ++i) vw[i] = R[i * 3] * r.vel[0] + R[i * 3 + 1] * r.vel[1] + R[i * 3 + 2] * r.vel[2];
            float wx, wy; map.toMapDir(vw, wx, wy);
            const float wl = std::sqrt(wx * wx + wy * wy);
            map.setWalkDirection(wl > 0.3f, wl > 0 ? wx / wl : 0, wl > 0 ? wy / wl : 1);
            mapAhead = map.obstacleAhead(0.35f, 3.0f, 0.9f, 0.6f);
            mapHazard = map.hazardAhead(0.30f, 3.0f, 0.6f);
        }
        safety.setMapAhead(mapAhead, mapHazard);
        if (s.state == SafetyState::Stop) ++stops;
        if (s.state == SafetyState::Caution) ++cautions;
        if (s.mapObstacle >= 0 || s.mapHazard >= 0) ++mapCautions;
        ++n;
        if (every > 0 && n % every == 0) {
            std::fill(img.begin(), img.end(), 0);
            map.draw(img.data(), 200, 0, 0, 200);
            char p[512];
            snprintf(p, sizeof p, "%s/map_%04d.ppm", outdir.c_str(), snap);
            writePpm(p, img, 200, 200);
            // 3D view with a fresh plan.
            CostMap cm;
            static Planner pl;
            static View3DState vstate;
            cm.build(map);
            float ux, uy, hx, hy;
            map.user(ux, uy, hx, hy);
            // DSCHECK: compare every incremental D* Lite plan with a full search from the
            // same planner state (goal memory, walking direction).
            Planner ref = pl;
            PlanResult plan = pl.plan(map, cm, hx, hy);
            if (getenv("DSCHECK") && !plan.fullReplan) {
                ref.invalidateSearch();
                const PlanResult f = ref.plan(map, cm, hx, hy);
                const float gd = std::hypot(plan.goalX - f.goalX, plan.goalY - f.goalY);
                if (plan.ok != f.ok || plan.blocked != f.blocked || std::fabs(plan.length - f.length) > 0.3f || gd > 0.3f)
                    printf("        DSCHECK MISMATCH t %.1f: incremental ok %d len %.2f goal (%.2f %.2f) | full ok %d len %.2f goal (%.2f %.2f)\n",
                           fr.t - t0, plan.ok, plan.length, plan.goalX, plan.goalY, f.ok, f.length, f.goalX, f.goalY);
            }
            std::vector<uint8_t> v3(640 * 480 * 4, 0);
            const auto tr0 = std::chrono::steady_clock::now();
            View3DOptions vopt; vopt.tSec = fr.t;
            renderView3D(v3.data(), 640, 480, 640, map, &cm, &plan, vopt, &vstate);
            printf("        render3d ms %.2f\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tr0).count());
            snprintf(p, sizeof p, "%s/view3d_%04d.ppm", outdir.c_str(), snap++);
            writePpm(p, v3, 640, 480);
            printf("        plan %s: %.1f m, turn %.0f deg, clearance %.2f m, %.1f ms, expanded %d%s\n", plan.ok ? "ok" : "none",
                   plan.length, plan.turnDeg, plan.minClearance, plan.ms, plan.expanded, plan.fullReplan ? " FULL" : "");
            const float* T = vo.poseT();
            printf("t %5.1f | vo %d inl %3d | floor %d camH %.2f pitch %5.1f | safety %d %-9s %.2f | map obst %.2f haz %.2f | pos %.2f %.2f %.2f\n",
                   fr.t - t0, r.ok, r.inliers, s.floorFound, s.cameraHeight, s.cameraPitchDeg, (int)s.state,
                   s.reason == SafetyReason::Obstacle ? "obstacle" : s.reason == SafetyReason::HeadHeight ? "head"
                   : s.reason == SafetyReason::DropOff ? "drop" : s.reason == SafetyReason::UnknownFloor ? "unknown"
                   : s.reason == SafetyReason::NoFloor ? "nofloor" : "-", s.distance, mapAhead, mapHazard, T[0], T[1], T[2]);
        }
    }
    printf("\n%d frames (%.1f s): STOP %d, CAUTION %d frames; map-driven CAUTION input %d frames\n", n, n / 30.0,
           stops, cautions, mapCautions);
    if (rd.version >= 3)
        printf("segmentation: recorded for %d / %d frames, used by the safety layer in %d\n", segFrames, n, semUsed);
    else
        printf("segmentation: not in this recording (format v%d; v3 records it)\n", rd.version);
    printf("gravity calibration: %s, %d pairs, spread %.1f deg, residual %.2f deg\n", cal.ok() ? "OK" : "refused",
           cal.samples(), cal.spreadDeg(), cal.residualDeg());
    return 0;
}
