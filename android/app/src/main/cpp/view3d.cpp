#include "view3d.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

struct V3 { float x, y, z; };
inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline V3 norm(V3 a) { const float l = std::sqrt(dot(a, a)); return {a.x / l, a.y / l, a.z / l}; }

struct Cam {
    V3 eye, R, U, F;
    float f, cx, cy;
    // World (map plane x, y; height z) -> pixel; false if behind.
    bool project(V3 p, float& u, float& v, float& depth) const {
        const V3 d = sub(p, eye);
        depth = dot(d, F);
        if (depth < 0.1f) return false;
        u = cx + f * dot(d, R) / depth;
        v = cy - f * dot(d, U) / depth;
        return true;
    }
};

struct Canvas {
    uint8_t* px; int w, h, stride;
    void put(int x, int y, const uint8_t c[3]) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        uint8_t* p = px + static_cast<size_t>(y) * stride * 4 + static_cast<size_t>(x) * 4;
        p[0] = c[0]; p[1] = c[1]; p[2] = c[2]; p[3] = 255;
    }
    void rect(float x0, float y0, float x1, float y1, const uint8_t c[3]) {
        const int a = std::max(0, static_cast<int>(std::min(x0, x1))), b = std::min(w - 1, static_cast<int>(std::max(x0, x1)));
        const int d = std::max(0, static_cast<int>(std::min(y0, y1))), e = std::min(h - 1, static_cast<int>(std::max(y0, y1)));
        for (int y = d; y <= e; ++y)
            for (int x = a; x <= b; ++x) put(x, y, c);
    }
    void line(float x0, float y0, float x1, float y1, int thick, const uint8_t c[3]) {
        const float len = std::max(std::fabs(x1 - x0), std::fabs(y1 - y0));
        const int steps = std::max(1, static_cast<int>(len));
        for (int s = 0; s <= steps; ++s) {
            const float x = x0 + (x1 - x0) * s / steps, y = y0 + (y1 - y0) * s / steps;
            for (int j = -thick / 2; j <= thick / 2; ++j)
                for (int i = -thick / 2; i <= thick / 2; ++i) put(static_cast<int>(x) + i, static_cast<int>(y) + j, c);
        }
    }
    void tri(float ax, float ay, float bx, float by, float cx, float cy, const uint8_t c[3]) {
        const int x0 = std::max(0, static_cast<int>(std::min({ax, bx, cx}))), x1 = std::min(w - 1, static_cast<int>(std::max({ax, bx, cx})));
        const int y0 = std::max(0, static_cast<int>(std::min({ay, by, cy}))), y1 = std::min(h - 1, static_cast<int>(std::max({ay, by, cy})));
        auto edge = [](float x0, float y0, float x1, float y1, float x, float y) { return (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0); };
        const float area = edge(ax, ay, bx, by, cx, cy);
        if (std::fabs(area) < 1e-3f) return;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const float w0 = edge(bx, by, cx, cy, x, y), w1 = edge(cx, cy, ax, ay, x, y), w2 = edge(ax, ay, bx, by, x, y);
                if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) put(x, y, c);
            }
    }
};

// Display colours per navigation class (NavClass ids, see nav_classes.h).
const uint8_t* classColour(uint8_t c) {
    static const uint8_t kObstacle[3] = {185, 140, 95};   // furniture / objects: tan (drop-offs are bright orange)
    static const uint8_t kWall[3] = {120, 135, 165};      // walls: slate blue-grey
    static const uint8_t kDoor[3] = {240, 200, 40};       // doors: yellow
    static const uint8_t kStairs[3] = {255, 170, 0};      // stairs: gold
    static const uint8_t kPerson[3] = {190, 90, 255};     // people: purple
    static const uint8_t kVehicle[3] = {60, 120, 255};    // vehicles: blue
    static const uint8_t kPlant[3] = {60, 170, 80};       // vegetation: green
    static const uint8_t kUnknown[3] = {200, 60, 50};     // unlabelled obstacle: red
    switch (c) {
        case 0: return kObstacle;
        case 2: case 14: return kWall;   // wall, building
        case 3: return kDoor;
        case 4: return kStairs;
        case 6: return kPerson;
        case 7: return kVehicle;
        case 13: return kPlant;
        default: return kUnknown;
    }
}

}  // namespace

void renderView3D(uint8_t* rgba, int width, int height, int stride, const LocalMap& map, const CostMap* cost,
                  const PlanResult* plan, const View3DOptions& opt, View3DState* state) {
    float ux, uy, hx, hy, ox, oy;
    map.user(ux, uy, hx, hy);
    // The user marker uses the true pose; the virtual camera a smoothed one.
    const float tux = ux, tuy = uy, thx = hx, thy = hy;
    if (state) {
        // Time-based: position follows within ~0.2 s, heading within ~0.6 s,
        // whatever the redraw rate.
        const double dt = (opt.tSec >= 0 && state->lastT >= 0) ? std::min(1.0, std::max(0.0, opt.tSec - state->lastT)) : 0.066;
        state->lastT = opt.tSec;
        const float aPos = 1.0f - std::exp(-static_cast<float>(dt) / 0.2f);
        const float aHead = 1.0f - std::exp(-static_cast<float>(dt) / 0.6f);
        if (!state->init) { state->ux = ux; state->uy = uy; state->hx = hx; state->hy = hy; state->init = true; }
        else {
            state->ux += aPos * (ux - state->ux);
            state->uy += aPos * (uy - state->uy);
            state->hx += aHead * (hx - state->hx);
            state->hy += aHead * (hy - state->hy);
            const float l = std::hypot(state->hx, state->hy);
            if (l > 1e-3f) { state->hx /= l; state->hy /= l; }
        }
        ux = state->ux; uy = state->uy; hx = state->hx; hy = state->hy;
    }
    map.origin(ox, oy);
    const float cell = map.cellSize();
    const int n = map.cells();
    Cam cam;
    cam.eye = {ux - opt.back * hx, uy - opt.back * hy, opt.up};
    const V3 target = {ux + opt.lookAhead * hx, uy + opt.lookAhead * hy, 0};
    cam.F = norm(sub(target, cam.eye));
    cam.R = norm(cross(cam.F, V3{0, 0, 1}));
    cam.U = cross(cam.R, cam.F);
    cam.f = opt.fovScale * width;
    cam.cx = width * 0.5f;
    cam.cy = height * 0.5f;
    Canvas cv{rgba, width, height, stride};
    // The cost map is built by the planner (5 Hz): until its first run it is
    // empty, so only use it when it matches the map.
    if (cost && cost->cells() != n) cost = nullptr;

    // Per-cell tables, computed once per frame (pixels and the noise filter
    // read them instead of re-evaluating occupancy / cost per pixel).
    std::vector<uint8_t> occT(static_cast<size_t>(n) * n, 0);
    std::vector<uint8_t> floorT(static_cast<size_t>(n) * n * 3, 28);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const size_t k = static_cast<size_t>(j) * n + i;
            const float p = map.occupancy(i, j);
            occT[k] = p >= 0.7f;
            uint8_t* c = &floorT[k * 3];
            if (map.hazard(i, j)) { c[0] = 255; c[1] = 140; c[2] = 0; }
            else if (map.semantic(i, j) == 4) { c[0] = 240; c[1] = 210; c[2] = 0; }   // stairs
            else if (map.known(i, j) && p < 0.5f) {
                float g = 205;
                if (cost) {
                    const float kc = cost->cost(i, j);
                    g = kc >= CostMap::kLethal ? 70.0f : 205.0f - std::min(120.0f, (kc - 1.0f) * 22.0f);
                }
                c[0] = c[1] = c[2] = static_cast<uint8_t>(g);
            }
        }

    // 1. Floor: ray-cast every 2x2 block onto z = 0. The ray direction is
    // linear along a row, so it is stepped; pixels are written directly.
    const float inf = 1.0f / cam.f, invCell = 1.0f / cell;
    for (int y = 0; y < height; y += 2) {
        const float vy = (y - cam.cy) * inf;
        V3 d = {cam.F.x - cam.cx * inf * cam.R.x - vy * cam.U.x, cam.F.y - cam.cx * inf * cam.R.y - vy * cam.U.y,
                cam.F.z - cam.cx * inf * cam.R.z - vy * cam.U.z};
        const V3 dStep = {2 * inf * cam.R.x, 2 * inf * cam.R.y, 2 * inf * cam.R.z};
        uint8_t* row0 = rgba + static_cast<size_t>(y) * stride * 4;
        uint8_t* row1 = (y + 1 < height) ? row0 + static_cast<size_t>(stride) * 4 : row0;
        const uint8_t sky = static_cast<uint8_t>(18 + 20 * y / height);
        for (int x = 0; x < width; x += 2, d.x += dStep.x, d.y += dStep.y, d.z += dStep.z) {
            uint8_t c[3] = {sky, sky, static_cast<uint8_t>(sky + 10)};
            if (d.z < -1e-3f) {
                const float t = -cam.eye.z / d.z;
                c[0] = c[1] = c[2] = 28;
                if (t < 12.0f) {
                    const float px = cam.eye.x + t * d.x, py = cam.eye.y + t * d.y;
                    // Bilinear blend between cell centres: smooth shading, no 10 cm squares.
                    const float fi = (px - ox) * invCell - 0.5f, fj = (py - oy) * invCell - 0.5f;
                    if (t > 6.0f) {   // far: a cell is ~1 px, plain lookup is enough
                        const float gi = fi + 0.5f, gj = fj + 0.5f;
                        if (gi >= 0 && gj >= 0 && gi < n && gj < n) {
                            const uint8_t* fc = &floorT[(static_cast<size_t>(gj) * n + static_cast<size_t>(gi)) * 3];
                            c[0] = fc[0]; c[1] = fc[1]; c[2] = fc[2];
                        }
                    } else if (fi >= 0 && fj >= 0 && fi < n - 1 && fj < n - 1) {
                        const int i0 = static_cast<int>(fi), j0 = static_cast<int>(fj);
                        const float ax = fi - i0, ay = fj - j0;
                        const uint8_t* c00 = &floorT[(static_cast<size_t>(j0) * n + i0) * 3];
                        const uint8_t* c10 = c00 + 3;
                        const uint8_t* c01 = c00 + static_cast<size_t>(n) * 3;
                        const uint8_t* c11 = c01 + 3;
                        for (int q = 0; q < 3; ++q)
                            c[q] = static_cast<uint8_t>((1 - ay) * ((1 - ax) * c00[q] + ax * c10[q]) +
                                                        ay * ((1 - ax) * c01[q] + ax * c11[q]));
                    }
                    // 1 m floor grid and subtle 1/2/3 m rings around the user (scale).
                    const float gx = px - static_cast<float>(static_cast<int>(px + 1000.0f) - 1000);
                    const float gy = py - static_cast<float>(static_cast<int>(py + 1000.0f) - 1000);
                    const float lw = 0.012f * t;   // ~constant line width on screen
                    float k = 1.0f;
                    if (gx < lw || gx > 1 - lw || gy < lw || gy > 1 - lw) k = 0.72f;
                    const float rx = px - ux, ry = py - uy, dr2 = rx * rx + ry * ry;
                    bool ring = false;
                    if (dr2 < 10.0f) {
                        const float dr = std::sqrt(dr2);
                        const float fr = dr - static_cast<float>(static_cast<int>(dr + 0.5f));
                        ring = dr > 0.5f && std::fabs(fr) < lw * 0.6f;
                    }
                    const float fade = std::max(0.35f, 1.0f - t / 14.0f) * k;
                    for (int q = 0; q < 3; ++q) c[q] = static_cast<uint8_t>(c[q] * fade);
                    if (ring) {
                        c[0] = static_cast<uint8_t>(c[0] * 0.7f + 30);
                        c[1] = static_cast<uint8_t>(c[1] * 0.7f + 60);
                        c[2] = static_cast<uint8_t>(c[2] * 0.7f + 90);
                    }
                }
            }
            for (int q = 0; q < 2 && x + q < width; ++q) {
                uint8_t* p0 = row0 + static_cast<size_t>(x + q) * 4;
                uint8_t* p1 = row1 + static_cast<size_t>(x + q) * 4;
                p0[0] = p1[0] = c[0]; p0[1] = p1[1] = c[1]; p0[2] = p1[2] = c[2]; p0[3] = p1[3] = 255;
            }
        }
    }

    // Planned path, drawn before the blocks so walls in front of it hide it.
    // It starts at the user's current position (the plan is up to 0.2 s
    // old): from the closest path point onward. Green = verified safe,
    // thin grey = planned but not verified, hidden during a STOP ahead.
    std::vector<float> path;
    if (plan && plan->ok && plan->path.size() >= 4 && opt.pathState != 0 && opt.stopAheadDist <= 0) {
        const auto& pp = plan->path;
        size_t best = 0; float bd = 1e9f;
        for (size_t i = 0; i + 1 < pp.size() && i < 40; i += 2) {
            const float d = std::hypot(pp[i] - tux, pp[i + 1] - tuy);
            if (d < bd) { bd = d; best = i; }
        }
        path.push_back(tux); path.push_back(tuy);
        for (size_t i = best + 2; i + 1 < pp.size(); i += 2) { path.push_back(pp[i]); path.push_back(pp[i + 1]); }
        if (path.size() < 4) path.clear();
    }
    if (!path.empty()) {
        const bool ok = opt.pathState == 1;
        // Amber (not faint grey): the planned way is visible, but not yet safe to follow.
        const uint8_t green[3] = {40, 230, 60}, dark[3] = {10, 80, 20}, grey[3] = {255, 170, 40};
        for (int pass = ok ? 0 : 1; pass < 2; ++pass)
            for (size_t i = 2; i + 1 < path.size(); i += 2) {
                float u0, v0, d0, u1, v1, d1;
                if (!cam.project({path[i - 2], path[i - 1], 0.03f}, u0, v0, d0)) continue;
                if (!cam.project({path[i], path[i + 1], 0.03f}, u1, v1, d1)) continue;
                if (ok) cv.line(u0, v0, u1, v1, pass ? 5 : 9, pass ? green : dark);
                else if ((i / 2) % 2) cv.line(u0, v0, u1, v1, 5, grey);   // dashed
            }
    }

    if (plan && !path.empty() && opt.pathState == 1) {   // goal ring
        const uint8_t g2[3] = {120, 255, 140};
        for (int k = 0; k < 16; ++k) {
            const float a0 = k * 6.2832f / 16, a1 = (k + 1) * 6.2832f / 16;
            float u0, v0, d0, u1, v1, d1;
            if (cam.project({plan->goalX + 0.25f * std::cos(a0), plan->goalY + 0.25f * std::sin(a0), 0.03f}, u0, v0, d0) &&
                cam.project({plan->goalX + 0.25f * std::cos(a1), plan->goalY + 0.25f * std::sin(a1), 0.03f}, u1, v1, d1))
                cv.line(u0, v0, u1, v1, 3, g2);
        }
    }

    // 2. Obstacles: shaded blocks (top + visible sides), coloured by class,
    //    far to near. Isolated single cells (noise) are not drawn.
    struct Col { float depth; int i, j; };
    std::vector<Col> cols;
    auto occ = [&](int i, int j) { return i >= 0 && j >= 0 && i < n && j < n && occT[static_cast<size_t>(j) * n + i]; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            if (!occ(i, j)) continue;
            int nb = 0;
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di) nb += (di || dj) && occ(i + di, j + dj);
            if (nb < 2) continue;
            const V3 c = {ox + (i + 0.5f) * cell, oy + (j + 0.5f) * cell, 0};
            // Walls behind the user would hide the view from the chase camera.
            if ((c.x - ux) * hx + (c.y - uy) * hy < -opt.behindCull) continue;
            float u, v, dpt;
            if (!cam.project(c, u, v, dpt) || dpt > 9.0f) continue;
            cols.push_back({dpt, i, j});
        }
    std::sort(cols.begin(), cols.end(), [](const Col& a, const Col& b) { return a.depth > b.depth; });
    auto quad = [&](V3 a, V3 b, V3 c2, V3 d, const uint8_t col[3]) {
        float ua, va, da, ub, vb, db, uc, vc, dc, ud, vd, dd;
        if (!cam.project(a, ua, va, da) || !cam.project(b, ub, vb, db) || !cam.project(c2, uc, vc, dc) ||
            !cam.project(d, ud, vd, dd)) return;
        cv.tri(ua, va, ub, vb, uc, vc, col);
        cv.tri(ua, va, uc, vc, ud, vd, col);
    };
    for (const Col& c : cols) {
        const float x0 = ox + c.i * cell, y0 = oy + c.j * cell, x1 = x0 + cell, y1 = y0 + cell;
        // Even wall tops: median height of the occupied 3x3 neighbourhood.
        float hs[9]; int nh = 0;
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di)
                if (occ(c.i + di, c.j + dj)) hs[nh++] = map.obstacleHeight(c.i + di, c.j + dj);
        std::nth_element(hs, hs + nh / 2, hs + nh);
        const float top = std::max(0.25f, std::min(2.2f, hs[nh / 2]));
        // Overhangs (free space underneath) float; low bottoms snap to the floor.
        float bottom = map.obstacleBottom(c.i, c.j);
        // Only head-level overhangs (signs, shelves) float: a wall whose foot
        // was out of view would otherwise look broken into pieces.
        if (bottom < 1.2f || bottom > top - 0.05f) bottom = 0;
        const uint8_t* base = classColour(map.semantic(c.i, c.j));
        const float fade = std::max(0.45f, 1.0f - c.depth / 13.0f);
        auto shade = [&](float k, uint8_t out[3]) {
            for (int q = 0; q < 3; ++q) out[q] = static_cast<uint8_t>(std::min(255.0f, base[q] * k * fade));
        };
        uint8_t topC[3], sideA[3], sideB[3];
        shade(1.0f, topC); shade(0.70f, sideA); shade(0.52f, sideB);
        const float cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        // Far cells (only a few pixels wide): one shaded column is enough.
        {
            float ub, vb, db, ut, vt, dt;
            if (!cam.project({cx, cy, bottom}, ub, vb, db) || !cam.project({cx, cy, top}, ut, vt, dt)) continue;
            const float halfW = 0.5f * cam.f * cell / db;
            if (halfW < 4.0f) {
                cv.rect(ub - halfW - 0.5f, vt - 1, ub + halfW + 0.5f, vt + 1, topC);   // lit top edge
                cv.rect(ub - halfW - 0.5f, vt + 1, ub + halfW + 0.5f, vb, sideA);
                continue;
            }
        }
        // Visible vertical faces: those facing the virtual camera.
        if (cam.eye.x < x0 && !occ(c.i - 1, c.j)) quad({x0, y0, bottom}, {x0, y1, bottom}, {x0, y1, top}, {x0, y0, top}, sideA);
        if (cam.eye.x > x1 && !occ(c.i + 1, c.j)) quad({x1, y0, bottom}, {x1, y1, bottom}, {x1, y1, top}, {x1, y0, top}, sideA);
        if (cam.eye.y < y0 && !occ(c.i, c.j - 1)) quad({x0, y0, bottom}, {x1, y0, bottom}, {x1, y0, top}, {x0, y0, top}, sideB);
        if (cam.eye.y > y1 && !occ(c.i, c.j + 1)) quad({x0, y1, bottom}, {x1, y1, bottom}, {x1, y1, top}, {x0, y1, top}, sideB);
        quad({x0, y0, top}, {x1, y0, top}, {x1, y1, top}, {x0, y1, top}, topC);
        (void)cx; (void)cy;
    }

    // 3. Live point cloud (latest frame), above-floor points coloured by height.
    size_t cloudIdx = 0;
    for (const LocalMap::CloudPt& p : map.cloud()) {
        if (p.h < 0.3f || (cloudIdx++ & 1)) continue;   // obstacles only, every 2nd point
        if ((p.x - ux) * hx + (p.y - uy) * hy < -opt.behindCull) continue;
        float u, v, d;
        if (!cam.project({p.x, p.y, p.h}, u, v, d) || d > 10.0f) continue;
        // Class colour, lightened so points stand out on the blocks.
        const uint8_t* b = classColour(p.sem);
        const uint8_t col[3] = {static_cast<uint8_t>(90 + b[0] / 2), static_cast<uint8_t>(90 + b[1] / 2),
                                static_cast<uint8_t>(90 + b[2] / 2)};
        cv.put(static_cast<int>(u), static_cast<int>(v), col);
    }

    // 4. Trajectory arrows.
    const uint8_t red[3] = {220, 40, 30};
    for (const LocalMap::TrailPose& t : map.trail()) {
        const float px = -t.hy, py = t.hx;   // left perpendicular
        float au, av, ad, bu, bv, bd, cu, cvv, cd;
        if (!cam.project({t.x + 0.12f * t.hx, t.y + 0.12f * t.hy, 0.02f}, au, av, ad)) continue;
        if (!cam.project({t.x - 0.06f * t.hx + 0.07f * px, t.y - 0.06f * t.hy + 0.07f * py, 0.02f}, bu, bv, bd)) continue;
        if (!cam.project({t.x - 0.06f * t.hx - 0.07f * px, t.y - 0.06f * t.hy - 0.07f * py, 0.02f}, cu, cvv, cd)) continue;
        cv.tri(au, av, bu, bv, cu, cvv, red);
    }

    // Markers below use the user's true pose (the camera used a smoothed one).
    ux = tux; uy = tuy; hx = thx; hy = thy;

    // 5b. Goal marker (verified path only), or a red X ahead when there is
    // no clear way / an obstacle directly ahead.
    if (opt.stopAheadDist > 0) {
        const uint8_t rx[3] = {255, 40, 40};
        const float d = std::max(0.3f, opt.stopAheadDist), px = -hy, py = hx;
        float a, b, dd, c, e, f2;
        // Red bar across the walking direction at the obstacle, and an X on it.
        if (cam.project({ux + d * hx + 0.4f * px, uy + d * hy + 0.4f * py, 0.05f}, a, b, dd) &&
            cam.project({ux + d * hx - 0.4f * px, uy + d * hy - 0.4f * py, 0.05f}, c, e, f2)) cv.line(a, b, c, e, 8, rx);
        const float s2 = 0.25f, cxp = ux + (d - 0.35f) * hx, cyp = uy + (d - 0.35f) * hy;
        if (cam.project({cxp + s2 * (hx + px), cyp + s2 * (hy + py), 0.05f}, a, b, dd) &&
            cam.project({cxp - s2 * (hx + px), cyp - s2 * (hy + py), 0.05f}, c, e, f2)) cv.line(a, b, c, e, 6, rx);
        if (cam.project({cxp + s2 * (hx - px), cyp + s2 * (hy - py), 0.05f}, a, b, dd) &&
            cam.project({cxp - s2 * (hx - px), cyp - s2 * (hy - py), 0.05f}, c, e, f2)) cv.line(a, b, c, e, 6, rx);
    } else if (plan && plan->blocked) {
        const uint8_t rx[3] = {255, 40, 40};
        const float cxp = ux + 1.5f * hx, cyp = uy + 1.5f * hy, s = 0.35f;
        const float px = -hy, py = hx;
        float a, b, d, c, e, f2;
        if (cam.project({cxp + s * (hx + px), cyp + s * (hy + py), 0.05f}, a, b, d) &&
            cam.project({cxp - s * (hx + px), cyp - s * (hy + py), 0.05f}, c, e, f2)) cv.line(a, b, c, e, 6, rx);
        if (cam.project({cxp + s * (hx - px), cyp + s * (hy - py), 0.05f}, a, b, d) &&
            cam.project({cxp - s * (hx - px), cyp - s * (hy - py), 0.05f}, c, e, f2)) cv.line(a, b, c, e, 6, rx);
    }

    // 6. Movers (purple columns) and the user's axes.
    const uint8_t purple[3] = {190, 90, 255};
    for (const MapMover& m : map.movers()) {
        float u0, v0, d0, u1, v1, d1;
        if (!cam.project({m.x, m.y, 0}, u0, v0, d0) || !cam.project({m.x, m.y, 1.7f}, u1, v1, d1)) continue;
        const float hw = 0.25f * cam.f / d0;
        cv.rect(u0 - hw, v1, u0 + hw, v0, purple);
    }
    // User: cyan arrow on the floor pointing along the heading.
    {
        const float px = -hy, py = hx;
        float a, b, d, c, e, f2, g, h2, k;
        const uint8_t cyan[3] = {0, 230, 255};
        if (cam.project({ux + 0.35f * hx, uy + 0.35f * hy, 0.04f}, a, b, d) &&
            cam.project({ux - 0.2f * hx + 0.18f * px, uy - 0.2f * hy + 0.18f * py, 0.04f}, c, e, f2) &&
            cam.project({ux - 0.2f * hx - 0.18f * px, uy - 0.2f * hy - 0.18f * py, 0.04f}, g, h2, k))
            cv.tri(a, b, c, e, g, h2, cyan);
    }
    float uu, uv, ud;
    if (cam.project({ux, uy, 0.05f}, uu, uv, ud)) {
        const uint8_t cr[3] = {255, 60, 60}, cg[3] = {60, 255, 60}, cb[3] = {80, 120, 255};
        float a, b, d;
        if (cam.project({ux + 0.5f * hx, uy + 0.5f * hy, 0.05f}, a, b, d)) cv.line(uu, uv, a, b, 3, cr);      // forward
        if (cam.project({ux - 0.4f * hy, uy + 0.4f * hx, 0.05f}, a, b, d)) cv.line(uu, uv, a, b, 3, cg);      // left
        if (cam.project({ux, uy, 0.55f}, a, b, d)) cv.line(uu, uv, a, b, 3, cb);                               // up
    }

    // 7. Inset: heading-up top-down navigation map (depth-built cost map),
    // 4 m around the user, with the planned path. Green = free walking,
    // yellow -> red = getting close to obstacles, black = obstacle / lethal,
    // dark blue-grey = not seen yet.
    {
        const int S = std::max(80, std::min(width, height) * 3 / 8);
        const int x0 = width - S - 8, y0 = 8;
        const float R = 4.0f, mpp = 2 * R / S;            // metres per pixel
        const float px = -thy, py = thx;                  // right of heading (y up = forward)
        const float invCell = 1.0f / cell;
        for (int v = 0; v < S; ++v) {
            uint8_t* row = rgba + (static_cast<size_t>(y0 + v) * stride + x0) * 4;
            const float fy = (S * 0.65f - v) * mpp;       // user sits at 65 % down
            for (int u = 0; u < S; ++u, row += 4) {
                const float fx = (u - S * 0.5f) * mpp;
                const float wx = tux + fy * thx - fx * px, wy = tuy + fy * thy - fx * py;
                const int i = static_cast<int>((wx - ox) * invCell), j = static_cast<int>((wy - oy) * invCell);
                uint8_t c[3] = {30, 34, 44};
                if (i >= 0 && j >= 0 && i < n && j < n && map.known(i, j)) {
                    if (map.occupancy(i, j) > 0.7f || (cost && cost->lethal(i, j))) { c[0] = c[1] = c[2] = 10; }
                    else if (cost) {
                        const float t = std::min(1.0f, std::max(0.0f, (0.8f - cost->distance(i, j)) / 0.58f));
                        c[0] = static_cast<uint8_t>(60 + 195 * std::min(1.0f, 2 * t));
                        c[1] = static_cast<uint8_t>(200 - 160 * std::max(0.0f, 2 * t - 1));
                        c[2] = 60;
                    } else { c[0] = 90; c[1] = 160; c[2] = 90; }
                }
                row[0] = static_cast<uint8_t>((c[0] * 3 + row[0]) / 4);
                row[1] = static_cast<uint8_t>((c[1] * 3 + row[1]) / 4);
                row[2] = static_cast<uint8_t>((c[2] * 3 + row[2]) / 4);
                row[3] = 255;
            }
        }
        auto toInset = [&](float wx, float wy, float& u, float& v) {
            const float dx = wx - tux, dy = wy - tuy;
            u = x0 + S * 0.5f + (dx * px + dy * py) / mpp;
            v = y0 + S * 0.65f - (dx * thx + dy * thy) / mpp;
        };
        Canvas ic{rgba, width, height, stride};
        if (!path.empty()) {
            const uint8_t g[3] = {0, 255, 90}, gr[3] = {255, 170, 40};
            for (size_t k2 = 0; k2 + 3 < path.size(); k2 += 2) {
                float a, b, c2, d;
                toInset(path[k2], path[k2 + 1], a, b);
                toInset(path[k2 + 2], path[k2 + 3], c2, d);
                auto in = [&](float x, float y) { return x >= x0 && x < x0 + S && y >= y0 && y < y0 + S; };
                if (in(a, b) && in(c2, d)) ic.line(a, b, c2, d, opt.pathState == 1 ? 3 : 2, opt.pathState == 1 ? g : gr);
            }
        }
        const float uc = x0 + S * 0.5f, vc = y0 + S * 0.65f;
        const uint8_t cy[3] = {0, 230, 255}, wh[3] = {230, 230, 230};
        ic.tri(uc, vc - 9, uc - 6, vc + 6, uc + 6, vc + 6, cy);
        ic.line(x0, y0, x0 + S, y0, 1, wh); ic.line(x0, y0 + S, x0 + S, y0 + S, 1, wh);
        ic.line(x0, y0, x0, y0 + S, 1, wh); ic.line(x0 + S, y0, x0 + S, y0 + S, 1, wh);
    }
}
