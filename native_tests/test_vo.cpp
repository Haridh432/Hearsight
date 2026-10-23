// Host test: visual odometry on a synthetic textured corridor with known
// head-mounted camera motion (walking, nods, sway), plus a crossing person.
#include "vo.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

const int W = 640, H = 480;
const float FX = 385, FY = 385, CX = 320, CY = 240;
const float PI = 3.14159265f;

// --- Procedural texture: random grey tiles (0.2 m) with noise: lots of corners.
static float hash2(int a, int b, int s) {
    unsigned h = static_cast<unsigned>(a) * 73856093u ^ static_cast<unsigned>(b) * 19349663u ^ static_cast<unsigned>(s) * 83492791u;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return (h & 0xffff) / 65535.0f;
}
static float g_contrast = 1.0f;   // texture contrast (1 = normal, small = plain walls)
static float texture(float a, float b, int surface) {
    const float tile = 0.2f;
    const int ia = static_cast<int>(std::floor(a / tile)), ib = static_cast<int>(std::floor(b / tile));
    float v = 0.25f + 0.6f * hash2(ia, ib, surface);
    const int ja = static_cast<int>(std::floor(a / 0.05f)), jb = static_cast<int>(std::floor(b / 0.05f));
    v += 0.15f * (hash2(ja, jb, surface + 7) - 0.5f);
    v = 0.55f + g_contrast * (v - 0.55f);
    return std::max(0.0f, std::min(1.0f, v));
}

struct Box { float x0, y0, z0, x1, y1, z1; int tex; };
struct Pose { float p[3]; float r[3], d[3], f[3]; };   // camera axes in world

static Pose makePose(float x, float y, float z, float yaw, float pitch) {
    Pose P;
    P.p[0] = x; P.p[1] = y; P.p[2] = z;
    const float f[3] = {std::sin(yaw) * std::cos(pitch), -std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    const float r[3] = {std::cos(yaw), 0, -std::sin(yaw)};
    // d = f x r (camera y down) so that r x d = f.
    const float d[3] = {f[1] * r[2] - f[2] * r[1], f[2] * r[0] - f[0] * r[2], f[0] * r[1] - f[1] * r[0]};
    for (int i = 0; i < 3; ++i) { P.f[i] = f[i]; P.r[i] = r[i]; P.d[i] = d[i]; }
    return P;
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

// Corridor: floor y=0, ceiling 2.8, walls x=+-1.5, end wall z=14.
static void render(const Pose& P, const std::vector<Box>& boxes, std::vector<uint8_t>& rgba,
                   std::vector<float>& depth, std::mt19937& rng) {
    rgba.assign(W * H * 4, 255);
    depth.assign(W * H, 0);
    std::normal_distribution<float> nz(0, 1);
    for (int v = 0; v < H; ++v)
        for (int u = 0; u < W; ++u) {
            const float xc = (u + 0.5f - CX) / FX, yc = (v + 0.5f - CY) / FY;
            float d[3];
            for (int i = 0; i < 3; ++i) d[i] = P.r[i] * xc + P.d[i] * yc + P.f[i];
            float t = 1e9f; float a = 0, b = 0; int s = 0;
            auto plane = [&](int axis, float val, int surf, int ua, int ub) {
                if (std::fabs(d[axis]) < 1e-9f) return;
                const float tt = (val - P.p[axis]) / d[axis];
                if (tt > 0.05f && tt < t) { t = tt; a = P.p[ua] + d[ua] * tt; b = P.p[ub] + d[ub] * tt; s = surf; }
            };
            plane(1, 0.0f, 1, 0, 2); plane(1, 2.8f, 2, 0, 2);
            plane(0, -1.5f, 3, 2, 1); plane(0, 1.5f, 4, 2, 1); plane(2, 14.0f, 5, 0, 1);
            for (const Box& bx : boxes) {
                const float tb = rayBox(P.p, d, bx);
                if (tb > 0.05f && tb < t) {
                    t = tb; s = bx.tex;
                    const float hp[3] = {P.p[0] + d[0] * tb, P.p[1] + d[1] * tb, P.p[2] + d[2] * tb};
                    a = hp[0] + hp[2]; b = hp[1];
                }
            }
            const float g = texture(a, b, s);
            uint8_t* px = &rgba[(v * W + u) * 4];
            px[0] = px[1] = px[2] = static_cast<uint8_t>(g * 255);
            // t is the depth along the camera z axis (ray has unit z).
            if (t < 8.0f) depth[v * W + u] = t + 0.001f * t * t * nz(rng);
        }
}

struct DepthCtx { const std::vector<float>* d; };
static float depthAt(void* ctx, float u, float v) {
    const auto& d = *static_cast<DepthCtx*>(ctx)->d;
    const int x = static_cast<int>(u), y = static_cast<int>(v);
    if (x < 0 || y < 0 || x >= W || y >= H) return -1;
    return d[y * W + x];
}

struct Motion { const char* name; float speed; float nodDeg; float swayDeg; float yawRate; bool person; double secs;
                float contrast = 1.0f; float swayHz = 0.5f; };

static int run(const Motion& m, std::mt19937& rng) {
    VisualOdometry vo;
    vo.setCamera({FX, FY, CX, CY, W, H});
    g_contrast = m.contrast;
    std::vector<uint8_t> rgba; std::vector<float> depth;
    DepthCtx ctx{&depth};
    double speedErr = 0, msSum = 0; int n = 0, okFrames = 0, frames = 0, inl = 0;
    Pose P0{}, P{};
    const int N = static_cast<int>(m.secs * 30);
    float gtPrev[3] = {0, 0, 0};
    for (int f = 0; f <= N; ++f) {
        const double t = f / 30.0;
        const float z = 1.0f + m.speed * static_cast<float>(t);
        const float y = 1.6f + 0.03f * std::sin(2 * PI * 2.0f * static_cast<float>(t));
        const float pitch = (25.0f + m.nodDeg * std::sin(2 * PI * 1.0f * static_cast<float>(t))) * PI / 180;
        const float yaw = (m.swayDeg * std::sin(2 * PI * m.swayHz * static_cast<float>(t))) * PI / 180 +
                          m.yawRate * static_cast<float>(t) * PI / 180;
        P = makePose(0.0f, y, z, yaw, pitch);
        if (f == 0) P0 = P;
        std::vector<Box> boxes = {{-1.5f, 0, 6.0f, -0.9f, 1.0f, 6.6f, 6}, {0.8f, 0, 9.0f, 1.5f, 0.8f, 9.5f, 7}};
        if (m.person) {
            const float px = -1.4f + 1.2f * static_cast<float>(t);   // crossing at 1.2 m/s
            boxes.push_back({px - 0.25f, 0, z + 2.5f, px + 0.25f, 1.75f, z + 2.8f, 8});
        }
        render(P, boxes, rgba, depth, rng);
        VoResult r = vo.process(rgba.data(), t, depthAt, &ctx);
        msSum += r.ms; ++frames;
        const float gt[3] = {P.p[0], P.p[1], P.p[2]};
        if (f < 10) { for (int i = 0; i < 3; ++i) gtPrev[i] = gt[i]; continue; }   // let the velocity settle
        // True camera velocity in current camera axes (finite difference).
        float vw[3]; for (int i = 0; i < 3; ++i) vw[i] = (gt[i] - gtPrev[i]) * 30.0f;
        const float vc[3] = {vw[0] * P.r[0] + vw[1] * P.r[1] + vw[2] * P.r[2],
                             vw[0] * P.d[0] + vw[1] * P.d[1] + vw[2] * P.d[2],
                             vw[0] * P.f[0] + vw[1] * P.f[1] + vw[2] * P.f[2]};
        const float trueSpeed = std::sqrt(vc[0] * vc[0] + vc[1] * vc[1] + vc[2] * vc[2]);
        if (r.ok) { ++okFrames; inl += r.inliers; }
        speedErr += std::fabs(r.speed - trueSpeed); ++n;
        for (int i = 0; i < 3; ++i) gtPrev[i] = gt[i];
    }
    // Position drift: VO position (world = first camera frame) vs truth
    // expressed in the first camera frame.
    float dw[3] = {P.p[0] - P0.p[0], P.p[1] - P0.p[1], P.p[2] - P0.p[2]};
    const float truth[3] = {dw[0] * P0.r[0] + dw[1] * P0.r[1] + dw[2] * P0.r[2],
                            dw[0] * P0.d[0] + dw[1] * P0.d[1] + dw[2] * P0.d[2],
                            dw[0] * P0.f[0] + dw[1] * P0.f[1] + dw[2] * P0.f[2]};
    const float* est = vo.poseT();
    const float err = std::sqrt((est[0] - truth[0]) * (est[0] - truth[0]) + (est[1] - truth[1]) * (est[1] - truth[1]) +
                                (est[2] - truth[2]) * (est[2] - truth[2]));
    const float dist = std::sqrt(truth[0] * truth[0] + truth[1] * truth[1] + truth[2] * truth[2]);
    const double mse = speedErr / std::max(1, n);
    const float driftPct = dist > 0.5f ? 100 * err / dist : 0;
    const bool pass = okFrames >= n * 95 / 100 && mse < 0.10 && (dist < 0.5f ? err < 0.10f : driftPct < 5.0f);
    printf("%-34s: ok %3d/%3d, inliers %4.0f, speed err %.3f m/s, travelled %.2f m, drift %.3f m (%.1f%%), %.1f ms/frame  %s\n",
           m.name, okFrames, n, okFrames ? double(inl) / okFrames : 0.0, mse, dist, err, driftPct, msSum / frames,
           pass ? "OK" : "FAIL");
    return pass ? 0 : 1;
}

int main() {
    std::mt19937 rng(3);
    const Motion cases[] = {
        {"standing still, nodding", 0.0f, 7, 0, 0, false, 4},
        {"walking 1.0 m/s, nods + sway", 1.0f, 7, 5, 0, false, 6},
        {"walking 1.4 m/s, nods + sway", 1.4f, 10, 8, 0, false, 6},
        {"turning on the spot 45 deg/s", 0.0f, 3, 0, 45, false, 4},
        {"walking 1.0 m/s, person crossing", 1.0f, 7, 5, 0, true, 6},
        {"walking, plain surfaces (10% contrast)", 1.0f, 7, 5, 0, false, 6, 0.10f},
        {"walking, plain surfaces (4% contrast)", 1.0f, 7, 5, 0, false, 6, 0.04f},
        {"walking, head turns +-35 deg @0.8Hz", 1.0f, 7, 35, 0, false, 6, 1.0f, 0.8f},
    };
    int fail = 0;
    for (const Motion& m : cases) fail += run(m, rng);
    printf("vo failures: %d\n", fail);
    return fail;
}
