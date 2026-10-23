// Host test: Phase 4 dynamic local map on synthetic depth with a known pose.
#include "local_map.h"
#include "safety.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

const int W = 320, H = 240;
const float FX = 195, FY = 195, CX = 160, CY = 120;
const float PI = 3.14159265f;

static float g_noise = 0.001f;   // depth noise: sigma = g_noise * z^2 (m)
struct Box { float x0, y0, z0, x1, y1, z1; int id; };
struct Scene { std::vector<Box> boxes; float stepZ = -1, stepDrop = 0; };
struct Cam { float p[3]; float r[3], d[3], f[3]; };

static Cam makeCam(float x, float y, float z, float yawDeg, float pitchDeg) {
    const float yaw = yawDeg * PI / 180, pitch = pitchDeg * PI / 180;
    Cam c{};
    c.p[0] = x; c.p[1] = y; c.p[2] = z;
    const float f[3] = {std::sin(yaw) * std::cos(pitch), -std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    const float r[3] = {std::cos(yaw), 0, -std::sin(yaw)};
    // Camera y points down: d = r x f.
    const float d[3] = {r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]};
    for (int i = 0; i < 3; ++i) { c.f[i] = f[i]; c.r[i] = r[i]; c.d[i] = d[i]; }
    return c;
}

static float rayBox(const float o[3], const float d[3], const Box& b) {
    float tmin = 0, tmax = 1e9f; const float lo[3] = {b.x0, b.y0, b.z0}, hi[3] = {b.x1, b.y1, b.z1};
    for (int i = 0; i < 3; i++) {
        if (std::fabs(d[i]) < 1e-9f) { if (o[i] < lo[i] || o[i] > hi[i]) return -1; continue; }
        float t1 = (lo[i] - o[i]) / d[i], t2 = (hi[i] - o[i]) / d[i]; if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1); tmax = std::min(tmax, t2); if (tmin > tmax) return -1;
    }
    return tmin;
}

// Renders depth (mm) and an object-id buffer (0 = floor/background).
static void render(const Cam& c, const Scene& s, std::vector<uint16_t>& depth, std::vector<uint8_t>& ids,
                   std::mt19937& rng) {
    depth.assign(W * H, 0); ids.assign(W * H, 0);
    std::normal_distribution<float> nz(0, 1);
    for (int v = 0; v < H; ++v)
        for (int u = 0; u < W; ++u) {
            const float xc = (u - CX) / FX, yc = (v - CY) / FY;
            float d[3];
            for (int i = 0; i < 3; ++i) d[i] = c.r[i] * xc + c.d[i] * yc + c.f[i];
            float t = 1e9f; int id = 0;
            if (d[1] < 0) {
                float tf = -c.p[1] / d[1];
                const float zf = c.p[2] + d[2] * tf;
                if (s.stepZ > 0 && zf > s.stepZ) {
                    tf = (-s.stepDrop - c.p[1]) / d[1];
                    const float tEdge = (s.stepZ - c.p[2]) / d[2], yEdge = c.p[1] + d[1] * tEdge;
                    if (yEdge < 0 && yEdge > -s.stepDrop) tf = tEdge;
                }
                t = tf;
            }
            for (const Box& b : s.boxes) { const float tb = rayBox(c.p, d, b); if (tb > 0 && tb < t) { t = tb; id = static_cast<uint8_t>(b.id); } }
            if (t > 6.0f) continue;
            depth[v * W + u] = static_cast<uint16_t>((t + g_noise * t * t * nz(rng)) * 1000);
            ids[v * W + u] = static_cast<uint8_t>(id);
        }
}

struct Sim {
    LocalMap map;
    std::mt19937 rng{7};
    std::vector<uint16_t> depth; std::vector<uint8_t> ids;
    int excludeId = 0;
    // Optional segmentation: true class per object id (0 = floor), with a
    // fraction of random wrong labels.
    bool semOn = false;
    float semNoise = 0.0f;
    uint8_t idClass[8] = {1, 2, 2, 0, 0, 0, 0, 0};   // id 0 floor, 1-2 wall, 3+ object
    std::uniform_real_distribution<float> U01{0, 1};
    double t = 0;
    void frame(const Cam& c, const Scene& s) { frame(c, c, s); }
    // Render from the true camera [c]; give the map the (drifted) pose [pc].
    void frame(const Cam& c, const Cam& pc, const Scene& s) {
        render(c, s, depth, ids, rng);
        MapInput in;
        in.depth = depth.data(); in.width = W; in.height = H;
        in.fx = FX; in.fy = FY; in.cx = CX; in.cy = CY; in.units = 0.001f; in.step = 2;
        // Pose: camera axes in world as columns.
        for (int i = 0; i < 3; ++i) { in.pose.R[i * 3] = pc.r[i]; in.pose.R[i * 3 + 1] = pc.d[i]; in.pose.R[i * 3 + 2] = pc.f[i]; in.pose.t[i] = pc.p[i]; }
        // Floor plane in camera axes: up (0,1,0) -> camera, height = camera y.
        in.floorN[0] = c.r[1]; in.floorN[1] = c.d[1]; in.floorN[2] = c.f[1];
        in.floorD = c.p[1];
        in.tSec = t;
        in.exclude = [](void* ctx, int u, int v) {
            const Sim* sim = static_cast<const Sim*>(ctx);
            return sim->excludeId && sim->ids[v * W + u] == sim->excludeId;
        };
        in.excludeCtx = this;
        if (semOn) {
            in.semantic = [](void* ctx, int u, int v) -> uint8_t {
                Sim* sim = static_cast<Sim*>(ctx);
                if (sim->U01(sim->rng) < sim->semNoise) {
                    static const uint8_t wrong[4] = {0, 1, 2, 3};
                    return wrong[static_cast<int>(sim->U01(sim->rng) * 4) & 3];
                }
                return sim->idClass[sim->ids[v * W + u] & 7];
            };
            in.semanticCtx = this;
        }
        map.integrate(in);
        t += 1.0 / 30;
    }
    // Max occupancy probability over world rectangle (x0..x1, z0..z1).
    float occIn(float x0, float x1, float z0, float z1) const {
        float best = 0;
        for (float x = x0; x <= x1; x += 0.05f)
            for (float z = z0; z <= z1; z += 0.05f) {
                const float w[3] = {x, 0, z};
                float a, b; map.toPlane(w, a, b);
                float ox, oy; map.origin(ox, oy);
                const int i = static_cast<int>(std::floor((a - ox) / map.cellSize()));
                const int j = static_cast<int>(std::floor((b - oy) / map.cellSize()));
                if (i < 0 || j < 0 || i >= map.cells() || j >= map.cells()) continue;
                best = std::max(best, map.occupancy(i, j));
            }
        return best;
    }
};

int main() {
    int fail = 0;
    auto check = [&](const char* name, bool ok, const char* detail) {
        printf("%-58s: %s  %s\n", name, detail, ok ? "OK" : "FAIL");
        if (!ok) ++fail;
    };
    char buf[200];

    // 1. Walk 0 -> 2.5 m toward a box at z 4.0-4.4, then turn 90 deg left for 3 s.
    {
        Sim s; Scene sc; sc.boxes.push_back({-0.3f, 0, 4.0f, 0.3f, 0.6f, 4.4f, 1});
        for (int f = 0; f < 75; ++f) s.frame(makeCam(0, 1.6f, 2.5f * f / 75, 0, 27), sc);
        const float aheadSeen = s.map.obstacleAhead(0.45f, 3.0f);
        for (int f = 0; f < 15; ++f) s.frame(makeCam(0, 1.6f, 2.5f, 90.0f * f / 15, 27), sc);
        for (int f = 0; f < 90; ++f) s.frame(makeCam(0, 1.6f, 2.5f, 90, 27), sc);
        const float pOut = s.occIn(-0.3f, 0.3f, 4.0f, 4.4f);
        for (int f = 0; f < 15; ++f) s.frame(makeCam(0, 1.6f, 2.5f, 90.0f - 90.0f * f / 15, 60), sc);
        const float aheadBack = s.map.obstacleAhead(0.45f, 3.0f);
        snprintf(buf, sizeof buf, "ahead %.2f m before turn; P(box) %.2f after 3 s looking away; ahead %.2f m back", aheadSeen, pOut, aheadBack);
        check("1 box stays in the map when out of view", aheadSeen > 1.3f && aheadSeen < 1.7f && pOut > 0.9f &&
              aheadBack > 1.3f && aheadBack < 1.7f, buf);
    }
    // 2. A person crossing 2.5 m ahead, excluded (as the tracker would): no trace.
    for (int excl = 1; excl >= 0; --excl) {
        Sim s; s.excludeId = excl ? 2 : 0;
        for (int f = 0; f < 120; ++f) {
            const float px = -2.0f + 4.0f * f / 120;
            Scene sc; sc.boxes.push_back({px - 0.25f, 0, 2.4f, px + 0.25f, 1.75f, 2.7f, 2});
            s.frame(makeCam(0, 1.6f, 0, 0, 27), sc);
        }
        const float p = s.occIn(-2.0f, 2.0f, 2.3f, 2.8f);
        snprintf(buf, sizeof buf, "max P on the crossing path %.2f", p);
        if (excl) check("2 crossing person excluded: no trace left", p < 0.6f, buf);
        else check("2 (control) same person NOT excluded leaves a trace", p > 0.7f, buf);
    }
    // 3. A box removed: its cells clear once seen as floor.
    {
        Sim s; Scene with; with.boxes.push_back({-0.3f, 0, 2.6f, 0.3f, 0.4f, 3.0f, 1});
        Scene without;
        for (int f = 0; f < 60; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 30), with);
        const float before = s.occIn(-0.3f, 0.3f, 2.6f, 3.0f);
        int clearedAt = -1;
        for (int f = 0; f < 90; ++f) {
            s.frame(makeCam(0, 1.6f, 0, 0, 30), without);
            if (clearedAt < 0 && s.occIn(-0.3f, 0.3f, 2.6f, 3.0f) < 0.5f) clearedAt = f;
        }
        snprintf(buf, sizeof buf, "P before %.2f; cleared after %.1f s", before, clearedAt / 30.0);
        check("3 removed box clears", before > 0.9f && clearedAt >= 0 && clearedAt < 60, buf);
    }
    // 4. Step down 3 m ahead: hazard at the right distance, kept while looking away.
    {
        Sim s; Scene sc; sc.stepZ = 3.0f; sc.stepDrop = 0.17f;
        // Walk to 1.5 m before the step (drops are mapped within ~2 m).
        for (int f = 0; f < 45; ++f) s.frame(makeCam(0, 1.6f, 1.5f * f / 45, 0, 27), sc);
        const float hz = s.map.hazardAhead(0.45f, 4.0f);
        for (int f = 0; f < 30; ++f) s.frame(makeCam(0, 1.6f, 1.5f, 0, -5), sc);   // look up: step out of view
        const float hz2 = s.map.hazardAhead(0.45f, 4.0f);
        snprintf(buf, sizeof buf, "hazard ahead %.2f m (true 1.5); after looking up %.2f m", hz, hz2);
        check("4 step down marked as a hazard", hz > 1.4f && hz < 1.8f && hz2 > 1.4f && hz2 < 1.8f, buf);
    }
    // 5. Fading: a box not seen for 30 s drops to half its log-odds.
    {
        Sim s; Scene sc; sc.boxes.push_back({-0.3f, 0, 2.6f, 0.3f, 0.6f, 3.0f, 1});
        for (int f = 0; f < 30; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 30), sc);
        const float p0 = s.occIn(-0.3f, 0.3f, 2.6f, 3.0f);
        for (int f = 0; f < 900; ++f) s.frame(makeCam(0, 1.6f, 0, 180, 30), sc);   // facing away 30 s
        const float p1 = s.occIn(-0.3f, 0.3f, 2.6f, 3.0f);
        const float lo0 = std::log(p0 / (1 - p0)), lo1 = std::log(p1 / (1 - p1));
        snprintf(buf, sizeof buf, "P %.3f -> %.3f after 30 s unseen (log-odds ratio %.2f)", p0, p1, lo1 / lo0);
        check("5 unseen evidence fades (half-life 30 s)", lo1 / lo0 > 0.4f && lo1 / lo0 < 0.6f, buf);
    }
    // 6. Safety with a remembered obstacle 0.8 m ahead on a clear floor:
    //    CAUTION (never STOP: odometry drift must not stop the user).
    {
        Sim s; Scene clear; SafetyLayer L; SafetyResult r;
        DepthIntrinsics in{FX, FY, CX, CY, W, H};
        for (int f = 0; f < 45; ++f) {
            render(makeCam(0, 1.6f, 0, 0, 27), clear, s.depth, s.ids, s.rng);
            L.setMapAhead(f >= 30 ? 0.8f : -1.0f, -1.0f);
            r = L.process(s.depth.data(), in, 0.001f, f / 30.0);
        }
        snprintf(buf, sizeof buf, "state %d (1 = CAUTION) reason %d dist %.2f", (int)r.state, (int)r.reason, r.distance);
        check("6 remembered obstacle -> CAUTION only", r.state == SafetyState::Caution && r.distance > 0.7f && r.distance < 0.9f, buf);
    }
    // 7. Odometry drift (2 deg/s yaw error, +10% distance) on a 6 m walk out
    //    and back in a 2 m wide corridor: walls must stay sharp with
    //    alignment; compare with alignment off.
    float wallP[2], inside[2];
    for (int align = 0; align < 2; ++align) {
        Sim s; s.map.alignEnabled = align;
        Scene sc;
        sc.boxes.push_back({-1.1f, 0, -1, -1.0f, 2.0f, 9, 1});      // left wall
        sc.boxes.push_back({1.0f, 0, -1, 1.1f, 2.0f, 9, 1});        // right wall
        sc.boxes.push_back({-1.0f, 0, 2.5f, -0.6f, 0.8f, 2.9f, 1}); // boxes along the walls
        sc.boxes.push_back({0.6f, 0, 4.5f, 1.0f, 0.8f, 4.9f, 1});
        sc.boxes.push_back({-1.0f, 0, 6.5f, 1.0f, 2.0f, 6.6f, 1});  // end wall
        const int N = 450;   // 15 s: 6 s out, 3 s turn, 6 s back
        for (int f = 0; f < N; ++f) {
            const double t = f / 30.0;
            float z, yaw;
            if (t < 6) { z = 0.8f * (float)t; yaw = 0; }
            else if (t < 9) { z = 4.8f; yaw = 180.0f * (float)(t - 6) / 3; }
            else { z = 4.8f - 0.8f * (float)(t - 9); yaw = 180; }
            const float pitch = 27 + 6 * std::sin(2 * PI * (float)t);
            const Cam truth = makeCam(0, 1.6f, z, yaw, pitch);
            // Drifted odometry: yaw error grows 2 deg/s, distance +10%.
            const float err = 2.0f * (float)t, e = err * PI / 180;
            const float px = 0, pz = 1.1f * z;
            const Cam drift = makeCam(px * std::cos(e) + pz * std::sin(e), 1.6f, -px * std::sin(e) + pz * std::cos(e),
                                      yaw + err, pitch);
            s.frame(truth, drift, sc);
        }
        // Mean occupancy on the true side walls and false obstacles inside
        // the corridor (where the user walked), both z 0..6 m.
        float sum = 0; int cnt = 0, bad = 0, tot = 0;
        for (float z = 0.2f; z < 6.0f; z += 0.1f) {
            sum += s.occIn(-1.1f, -1.0f, z, z + 0.1f) + s.occIn(1.0f, 1.1f, z, z + 0.1f); cnt += 2;
            for (float x = -0.45f; x < 0.45f; x += 0.1f) {
                if (z > 2.4f && z < 3.0f) continue;   // box areas excluded
                if (z > 4.4f && z < 5.0f) continue;
                ++tot; bad += s.occIn(x, x + 0.1f, z, z + 0.1f) > 0.7f;
            }
        }
        wallP[align] = sum / cnt; inside[align] = 100.0f * bad / tot;
        float dx, dy, dyaw; s.map.correction(dx, dy, dyaw);
        printf("   alignment %s: mean P on true walls %.2f, false obstacles in walked corridor %.1f%%, correction (%.2f, %.2f) m %.1f deg\n",
               align ? "on " : "off", wallP[align], inside[align], dx, dy, dyaw);
    }
    snprintf(buf, sizeof buf, "walls %.2f -> %.2f, corridor false obstacles %.1f%% -> %.1f%%", wallP[0], wallP[1], inside[0], inside[1]);
    check("7 drift correction keeps walls sharp", wallP[1] > 0.8f && inside[1] < 3.0f && inside[1] < inside[0], buf);
    // 8. A 1.5 m cabinet is removed while its floor is out of view (head
    //    level): rays to the wall behind it must clear it.
    for (int rc = 1; rc >= 0; --rc) {
        Sim s; s.map.rayClear = rc;
        Scene with; with.boxes.push_back({-0.3f, 0, 2.6f, 0.3f, 1.5f, 3.0f, 1}); with.boxes.push_back({-3, 0, 5.0f, 3, 2.5f, 5.1f, 1});
        Scene without; without.boxes.push_back({-3, 0, 5.0f, 3, 2.5f, 5.1f, 1});
        for (int f = 0; f < 45; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 30), with);
        const float before = s.occIn(-0.3f, 0.3f, 2.6f, 3.0f);
        int clearedAt = -1;
        for (int f = 0; f < 90; ++f) {
            s.frame(makeCam(0, 1.6f, 0, 0, 0), without);   // looking level: floor at 2.6 m not visible
            if (clearedAt < 0 && s.occIn(-0.3f, 0.3f, 2.6f, 3.0f) < 0.5f) clearedAt = f;
        }
        snprintf(buf, sizeof buf, "P before %.2f; cleared after %.1f s", before, clearedAt < 0 ? -1.0 : clearedAt / 30.0);
        if (rc) check("8 removed cabinet, floor out of view: rays clear it", before > 0.9f && clearedAt >= 0 && clearedAt < 45, buf);
        else check("8 (control) without ray clearing it stays", clearedAt < 0, buf);
    }
    // 9. Overhanging board 1.5-1.9 m: its cells keep free space underneath.
    {
        Sim s; Scene sc; sc.boxes.push_back({-0.5f, 1.5f, 2.0f, 0.5f, 1.9f, 2.1f, 1});
        for (int f = 0; f < 45; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 5), sc);
        float bottom = -1, top = -1;
        {
            const float w[3] = {0, 0, 2.05f}; float a, b; s.map.toPlane(w, a, b);
            float ox, oy; s.map.origin(ox, oy);
            const int i = (int)std::floor((a - ox) / s.map.cellSize()), j = (int)std::floor((b - oy) / s.map.cellSize());
            bottom = s.map.obstacleBottom(i, j); top = s.map.obstacleHeight(i, j);
        }
        snprintf(buf, sizeof buf, "cell bottom %.2f m, top %.2f m", bottom, top);
        check("9 overhang keeps free space under it", bottom > 1.4f && top < 2.0f && top > 1.5f, buf);
    }
    // 10. A 1.0 m box replaced by a 0.4 m one: the cell height comes down.
    {
        Sim s; Scene tall; tall.boxes.push_back({-0.3f, 0, 2.0f, 0.3f, 1.0f, 2.4f, 1});
        Scene shortB; shortB.boxes.push_back({-0.3f, 0, 2.0f, 0.3f, 0.4f, 2.4f, 1});
        for (int f = 0; f < 30; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 30), tall);
        auto topAt = [&]() {
            const float w[3] = {0, 0, 2.05f}; float a, b; s.map.toPlane(w, a, b);
            float ox, oy; s.map.origin(ox, oy);
            return s.map.obstacleHeight((int)std::floor((a - ox) / s.map.cellSize()), (int)std::floor((b - oy) / s.map.cellSize()));
        };
        const float h0 = topAt();
        for (int f = 0; f < 30; ++f) s.frame(makeCam(0, 1.6f, 0, 0, 30), shortB);
        const float h1 = topAt();
        snprintf(buf, sizeof buf, "height %.2f m -> %.2f m after 1 s", h0, h1);
        check("10 cell height shrinks to the new object", h0 > 0.9f && h1 < 0.5f, buf);
    }
    // 11. Class per cell with 30% wrong labels: walls stay 'wall', a box
    //     stays 'object' (floor points at their foot must not flip them).
    {
        Sim s; s.semOn = true; s.semNoise = 0.30f;
        Scene sc;
        sc.boxes.push_back({-1.1f, 0, 0.5f, -1.0f, 2.2f, 5, 1});   // walls (id 1 -> wall)
        sc.boxes.push_back({1.0f, 0, 0.5f, 1.1f, 2.2f, 5, 1});
        sc.boxes.push_back({-0.3f, 0, 2.5f, 0.3f, 0.6f, 2.9f, 3});  // box (id 3 -> object)
        for (int f = 0; f < 90; ++f) s.frame(makeCam(0, 1.6f, 0.8f * f / 90, 0, 27), sc);
        float ox, oy; s.map.origin(ox, oy);
        auto classAt = [&](float x0, float x1, float z0, float z1, int want, int& right, int& total) {
            right = total = 0;
            for (float x = x0; x <= x1; x += 0.1f)
                for (float z = z0; z <= z1; z += 0.1f) {
                    const float w[3] = {x, 0, z}; float a, b; s.map.toPlane(w, a, b);
                    const int i = (int)std::floor((a - ox) / s.map.cellSize()), j = (int)std::floor((b - oy) / s.map.cellSize());
                    if (i < 0 || j < 0 || i >= s.map.cells() || j >= s.map.cells() || s.map.occupancy(i, j) < 0.7f) continue;
                    ++total; right += s.map.semantic(i, j) == want;
                }
        };
        int wr, wt, br, bt;
        classAt(-1.1f, -1.0f, 1.5f, 4.5f, 2, wr, wt);
        int wr2, wt2; classAt(1.0f, 1.1f, 1.5f, 4.5f, 2, wr2, wt2); wr += wr2; wt += wt2;
        classAt(-0.3f, 0.3f, 2.5f, 2.9f, 0, br, bt);
        snprintf(buf, sizeof buf, "wall cells labelled wall %d/%d, box cells labelled object %d/%d", wr, wt, br, bt);
        check("11 class per cell survives 30% wrong labels", wt > 10 && wr >= wt * 8 / 10 && bt > 2 && br >= bt * 7 / 10, buf);
    }
    printf("map failures: %d\n", fail);
    return fail;
}
