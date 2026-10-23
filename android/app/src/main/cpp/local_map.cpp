#include "local_map.h"
#include "nav_classes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace {

inline float dot3(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void normalize3(float* v) {
    const float l = std::sqrt(dot3(v, v));
    if (l > 1e-9f) { v[0] /= l; v[1] /= l; v[2] /= l; }
}
inline int floorDiv(float v, float cell) { return static_cast<int>(std::floor(v / cell)); }

}  // namespace

LocalMap::LocalMap(int cells, float cellM) : n_(cells), cell_(cellM), grid_(static_cast<size_t>(cells) * cells) {}

void LocalMap::correction(float& dx, float& dy, float& yawDeg) const {
    dx = corrTx_; dy = corrTy_;
    yawDeg = std::atan2(corrS_, corrC_) * 57.2957795f;
}

float LocalMap::alignScore(const std::vector<float>& pts, float c, float s, float tx, float ty) const {
    float score = 0;
    for (size_t i = 0; i + 1 < pts.size(); i += 2) {
        const float a = c * pts[i] - s * pts[i + 1] + tx;
        const float b = s * pts[i] + c * pts[i + 1] + ty;
        const int idx = index(floorDiv(a, cell_), floorDiv(b, cell_));
        if (idx >= 0 && grid_[idx].lo > 0) score += grid_[idx].lo;
    }
    return score;
}

void LocalMap::reset() {
    corrC_ = 1; corrS_ = 0; corrTx_ = corrTy_ = 0;
    trail_.clear();
    cloud_.clear();
    std::fill(grid_.begin(), grid_.end(), Cell{});
    ox_ = oy_ = 0;
    haveUp_ = false;
    lastDecay_ = -1;
    movers_.clear();
}

int LocalMap::index(int ci, int cj) const {
    const int i = ci - ox_, j = cj - oy_;
    if (i < 0 || j < 0 || i >= n_ || j >= n_) return -1;
    return j * n_ + i;
}

void LocalMap::recentre(int cx, int cy) {
    // Keep the user inside the middle half of the grid; shift otherwise.
    const int mx = ox_ + n_ / 2, my = oy_ + n_ / 2;
    if (std::abs(cx - mx) <= n_ / 4 && std::abs(cy - my) <= n_ / 4) return;
    const int nox = cx - n_ / 2, noy = cy - n_ / 2;
    std::vector<Cell> g(grid_.size());
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const int src = index(nox + i, noy + j);
            if (src >= 0) g[static_cast<size_t>(j) * n_ + i] = grid_[src];
        }
    grid_.swap(g);
    ox_ = nox;
    oy_ = noy;
}

void LocalMap::decay(double tSec) {
    if (lastDecay_ < 0) { lastDecay_ = tSec; return; }
    const double dt = tSec - lastDecay_;
    if (dt <= 0) return;
    lastDecay_ = tSec;
    const float f = static_cast<float>(std::pow(0.5, dt / decayHalfLifeSec));
    for (Cell& c : grid_) { c.lo *= f; c.drop *= f; }
}

void LocalMap::toMapPoint(const float w[3], float& x, float& y) const {
    float a, b;
    toPlane(w, a, b);
    x = corrC_ * a - corrS_ * b + corrTx_;
    y = corrS_ * a + corrC_ * b + corrTy_;
}

void LocalMap::toMapDir(const float v[3], float& x, float& y) const {
    float a, b;
    toPlane(v, a, b);
    x = corrC_ * a - corrS_ * b;
    y = corrS_ * a + corrC_ * b;
}

void LocalMap::toPlane(const float w[3], float& x, float& y) const {
    x = dot3(w, e1_);
    y = dot3(w, e2_);
}

bool LocalMap::integrate(const MapInput& in, bool update) {
    if (!in.depth || in.width <= 0) return false;
    const bool haveFloor = in.floorD > 0;
    if (!update && !haveUp_) return false;
    if (update && !haveFloor) return false;
    const float* R = in.pose.R;
    const float* T = in.pose.t;

    // World "up" from the floor normal (camera axes -> world); the map plane
    // basis is fixed on the first frame so cells don't move afterwards.
    float upW[3];
    for (int i = 0; i < 3; ++i) upW[i] = R[i * 3] * in.floorN[0] + R[i * 3 + 1] * in.floorN[1] + R[i * 3 + 2] * in.floorN[2];
    normalize3(upW);
    if (update) lastCamH_ = in.floorD;
    if (!haveUp_) {
        for (int i = 0; i < 3; ++i) up_[i] = upW[i];
        float ref[3] = {1, 0, 0};
        if (std::fabs(dot3(ref, up_)) > 0.9f) { ref[0] = 0; ref[2] = 1; }
        const float k = dot3(ref, up_);
        for (int i = 0; i < 3; ++i) e1_[i] = ref[i] - k * up_[i];
        normalize3(e1_);
        e2_[0] = up_[1] * e1_[2] - up_[2] * e1_[1];
        e2_[1] = up_[2] * e1_[0] - up_[0] * e1_[2];
        e2_[2] = up_[0] * e1_[1] - up_[1] * e1_[0];
        haveUp_ = true;
    }

    // Raw user position / heading on the plane (odometry).
    float rux, ruy;
    toPlane(T, rux, ruy);
    const float fw[3] = {R[2], R[5], R[8]};   // camera +z in world
    float rhx = dot3(fw, e1_), rhy = dot3(fw, e2_);
    {
        const float hl = std::sqrt(rhx * rhx + rhy * rhy);
        if (hl > 1e-3f) { rhx /= hl; rhy /= hl; } else { rhx = hx_; rhy = hy_; }
    }

    // Collect this frame's points on the plane (raw pose).
    struct Pt { float a, b, h; uint8_t sem; };
    std::vector<Pt> pts;
    pts.reserve(static_cast<size_t>(in.width / in.step) * (in.height / in.step));
    const float* n = in.floorN;
    for (int v = 0; v < in.height; v += in.step) {
        const uint16_t* row = in.depth + static_cast<size_t>(v) * in.width;
        const float yn = (v - in.cy) / in.fy;
        for (int u = 0; u < in.width; u += in.step) {
            if (!row[u]) continue;
            const float z = row[u] * in.units;
            if (z < 0.3f || z > maxRange) continue;
            if (in.exclude && in.exclude(in.excludeCtx, u, v)) continue;
            const float p[3] = {(u - in.cx) / in.fx * z, yn * z, z};
            float h;
            if (haveFloor) h = dot3(n, p) + in.floorD;
            else {
                // No floor this frame: height along the map's up axis.
                const float rp[3] = {R[0] * p[0] + R[1] * p[1] + R[2] * p[2], R[3] * p[0] + R[4] * p[1] + R[5] * p[2],
                                     R[6] * p[0] + R[7] * p[1] + R[8] * p[2]};
                h = dot3(rp, up_) + lastCamH_;
            }
            if (h > headMaxH) continue;   // ceiling
            // Drop-off evidence only near, with a distance-dependent threshold.
            if (h < -dropBelow && !(z < dropRange && h < -(dropBelow + dropPerMetre * z))) continue;
            float w[3];
            for (int i = 0; i < 3; ++i) w[i] = R[i * 3] * p[0] + R[i * 3 + 1] * p[1] + R[i * 3 + 2] * p[2] + T[i];
            Pt q;
            toPlane(w, q.a, q.b);
            q.h = h;
            q.sem = in.semantic ? in.semantic(in.semanticCtx, u, v) : 255;
            pts.push_back(q);
        }
    }

    // Drift correction: match obstacle points (current correction applied)
    // against the map over small shifts / turns about the user.
    if (alignEnabled && update) {
        std::vector<float> obs;   // corrected plane coords of obstacle points
        size_t nObs = 0;
        for (const Pt& q : pts) nObs += q.h > bodyMinH;
        // At most ~300 points (evenly spread) keep the 225-pose search cheap.
        const size_t stride = std::max<size_t>(1, nObs / alignMaxPoints);
        size_t k = 0;
        for (const Pt& q : pts) {
            if (q.h <= bodyMinH) continue;
            if (k++ % stride) continue;
            obs.push_back(corrC_ * q.a - corrS_ * q.b + corrTx_);
            obs.push_back(corrS_ * q.a + corrC_ * q.b + corrTy_);
        }
        int known = 0;
        for (const Cell& c : grid_) known += c.lo > 1.0f;
        if (obs.size() >= 2 * 100 && known >= 30) {
            const float ucx = corrC_ * rux - corrS_ * ruy + corrTx_;
            const float ucy = corrS_ * rux + corrC_ * ruy + corrTy_;
            float best = alignScore(obs, 1, 0, 0, 0), base = best;
            float bc = 1, bs = 0, btx = 0, bty = 0;
            for (float yd = -alignYawRangeDeg; yd <= alignYawRangeDeg + 1e-3f; yd += alignYawStepDeg) {
                const float c = std::cos(yd / 57.2957795f), sn = std::sin(yd / 57.2957795f);
                for (float dx = -alignRange; dx <= alignRange + 1e-4f; dx += alignStep)
                    for (float dy = -alignRange; dy <= alignRange + 1e-4f; dy += alignStep) {
                        // Rotate about the user, then shift.
                        const float tx = ucx - (c * ucx - sn * ucy) + dx;
                        const float ty = ucy - (sn * ucx + c * ucy) + dy;
                        const float sc = alignScore(obs, c, sn, tx, ty);
                        if (sc > best) { best = sc; bc = c; bs = sn; btx = tx; bty = ty; }
                    }
            }
            // Accept only a clear improvement (flat corridors are ambiguous along their length).
            if (best > base * 1.02f + 0.5f) {
                const float nc = bc * corrC_ - bs * corrS_, ns = bs * corrC_ + bc * corrS_;
                const float ntx = bc * corrTx_ - bs * corrTy_ + btx, nty = bs * corrTx_ + bc * corrTy_ + bty;
                const float l = std::sqrt(nc * nc + ns * ns);
                corrC_ = nc / l; corrS_ = ns / l; corrTx_ = ntx; corrTy_ = nty;
            }
        }
    }

    // Corrected user pose.
    ux_ = corrC_ * rux - corrS_ * ruy + corrTx_;
    uy_ = corrS_ * rux + corrC_ * ruy + corrTy_;
    hx_ = corrC_ * rhx - corrS_ * rhy;
    hy_ = corrS_ * rhx + corrC_ * rhy;

    if (!update) {
        // Live view only: refresh the point cloud, keep the map as it is.
        cloud_.clear();
        const size_t stride = std::max<size_t>(1, pts.size() / 3000);
        for (size_t i = 0; i < pts.size(); i += stride) {
            const Pt& q = pts[i];
            cloud_.push_back({corrC_ * q.a - corrS_ * q.b + corrTx_, corrS_ * q.a + corrC_ * q.b + corrTy_, q.h, q.sem});
        }
        return true;
    }
    recentre(floorDiv(ux_, cell_), floorDiv(uy_, cell_));
    decay(in.tSec);

    // Trajectory: a pose every 0.3 m or 15 degrees.
    if (trail_.empty() ||
        std::hypot(ux_ - trail_.back().x, uy_ - trail_.back().y) > 0.3f ||
        hx_ * trail_.back().hx + hy_ * trail_.back().hy < 0.966f) {
        trail_.push_back({ux_, uy_, hx_, hy_});
        if (trail_.size() > 200) trail_.erase(trail_.begin());
    }

    // Per-frame evidence per cell, then one update per cell.
    const size_t N = grid_.size();
    std::vector<uint16_t> hits(N, 0), floors(N, 0), drops(N, 0), passes(N, 0);
    std::vector<float> top1(N, 0), top2(N, 0), low(N, 1e9f);
    std::vector<uint8_t> semCls(N, 255);
    // Per-frame class counts from obstacle points (and stairs from any point).
    auto semBin = [](uint8_t c) -> int {
        switch (c) {
            case 0: return 0;            // object / furniture
            case 2: case 14: return 1;   // wall, building
            case 3: return 2;            // door
            case 4: return 3;            // stairs
            case 6: return 4;            // person
            case 7: return 5;            // vehicle
            case 13: return 6;           // vegetation
            case 255: return -1;         // unknown
            default: return 7;
        }
    };
    std::vector<std::array<uint8_t, 8>> semCnt;
    if (in.semantic) semCnt.assign(N, std::array<uint8_t, 8>{});
    cloud_.clear();
    const size_t cloudStride = std::max<size_t>(1, pts.size() / 3000);
    size_t ci = 0;
    for (const Pt& q : pts) {
        const float a = corrC_ * q.a - corrS_ * q.b + corrTx_;
        const float b = corrS_ * q.a + corrC_ * q.b + corrTy_;
        if (ci++ % cloudStride == 0) cloud_.push_back({a, b, q.h, q.sem});
        const int idx = index(floorDiv(a, cell_), floorDiv(b, cell_));
        if (idx < 0) continue;
        if (std::fabs(q.h) < floorTol) floors[idx]++;
        else if (q.h > bodyMinH) {
            hits[idx]++;
            if (q.h > top1[idx]) { top2[idx] = top1[idx]; top1[idx] = q.h; }
            else if (q.h > top2[idx]) top2[idx] = q.h;
            low[idx] = std::min(low[idx], q.h);
        }
        // Glossy-floor reflections of walls / pillars / ceiling: deep under the floor,
        // labelled neither floor, stairs nor obstacle -> not a drop.
        else if (q.h < -dropBelow && !(q.h < -reflectBelow && q.sem != 255 && navToSemRole(q.sem) == kSemOther))
            drops[idx]++;
        if (q.sem != 255) semCls[idx] = q.sem;
        if (!semCnt.empty()) {
            const int b = semBin(q.sem);
            // Obstacle points vote; floor-level points only for stairs.
            if (b >= 0 && (q.h > bodyMinH || b == 3) && semCnt[idx][b] < 255) semCnt[idx][b]++;
        }
    }
    // Ray clearing: walk each sampled ray from the camera to its point; a cell
    // crossed at a height inside its obstacle range [minH, maxH] cannot hold
    // that obstacle any more (it moved away).
    if (rayClear) {
        const float camH = in.floorD;
        for (size_t k = 0; k < pts.size(); k += rayStride) {
            const Pt& q = pts[k];
            const float a = corrC_ * q.a - corrS_ * q.b + corrTx_;
            const float b = corrS_ * q.a + corrC_ * q.b + corrTy_;
            const float dx = a - ux_, dy = b - uy_;
            const float len = std::sqrt(dx * dx + dy * dy);
            const float stopAt = len - 0.15f;   // not the cells next to the hit
            if (stopAt <= 0.2f) continue;
            // Step in half-cell increments in grid units (no floor()/index per step).
            const float inv = 1.0f / cell_;
            float gx = (ux_ + dx * (0.2f / len)) * inv - ox_, gy = (uy_ + dy * (0.2f / len)) * inv - oy_;
            const int steps = static_cast<int>((stopAt - 0.2f) / (cell_ * 0.5f));
            const float sx = dx / len * 0.5f, sy = dy / len * 0.5f;   // half a cell per step
            const float hStep = (q.h - camH) / len * (cell_ * 0.5f);
            float hr = camH + (q.h - camH) * (0.2f / len);
            int lastIdx = -1;
            for (int k2 = 0; k2 < steps; ++k2, gx += sx, gy += sy, hr += hStep) {
                if (gx < 0 || gy < 0) continue;
                const int ci2 = static_cast<int>(gx), cj2 = static_cast<int>(gy);
                if (ci2 >= n_ || cj2 >= n_) continue;
                const int idx = cj2 * n_ + ci2;
                if (idx == lastIdx) continue;
                lastIdx = idx;
                const Cell& c = grid_[idx];
                if (c.lo <= 0) continue;
                if (hr > c.minH - 0.05f && hr < c.maxH + 0.05f) passes[idx]++;
            }
        }
    }

    const float t = static_cast<float>(in.tSec);
    for (size_t i = 0; i < N; ++i) {
        if (!hits[i] && !floors[i] && !drops[i] && !passes[i]) continue;
        Cell& c = grid_[i];
        if (hits[i] >= 2) {
            // Robust height: the 2nd-highest point (one noise spike can't set
            // it); grows at once, shrinks toward lower observations quickly.
            const float top = top2[i];
            const bool fresh = c.lo <= 0;
            c.lo = std::min(maxLogOdds, c.lo + hitLogOdds);
            if (fresh || top > c.maxH) c.maxH = top;
            else c.maxH += heightTrack * (top - c.maxH);
            if (fresh || low[i] < c.minH) c.minH = low[i];
            else c.minH += heightTrack * (low[i] - c.minH);
        }
        else if (passes[i] >= rayClearVotes) c.lo = std::max(-maxLogOdds, c.lo + 2.0f * freeLogOdds);
        else if (floors[i] >= 2) c.lo = std::max(-maxLogOdds, c.lo + freeLogOdds);
        if (drops[i] >= 2) c.drop = std::min(5.0f, c.drop + 1.0f);
        else if (floors[i] >= 2) c.drop *= 0.5f;
        if (!semCnt.empty()) {
            int total = 0;
            for (int b = 0; b < 8; ++b) total += semCnt[i][b];
            if (total > 0) {
                // Fade old counts a little; add 8 votes per frame shared in
                // proportion to this frame's classes (a per-class cap would let
                // every class saturate on cells with many points).
                for (int b = 0; b < 8; ++b) {
                    const int add = (semCnt[i][b] * 8 + total / 2) / total;
                    c.semHist[b] = static_cast<uint8_t>(std::min(255, c.semHist[b] * 15 / 16 + add));
                }
                int best = 0, sum = 0;
                for (int b = 0; b < 8; ++b) { sum += c.semHist[b]; if (c.semHist[b] > c.semHist[best]) best = b; }
                static const uint8_t kBinClass[8] = {0, 2, 3, 4, 6, 7, 13, 255};
                if (sum >= 3 && c.semHist[best] * 10 >= sum * 4) { c.sem = kBinClass[best]; c.semVotes = 10; }
                else c.semVotes = 0;
            }
        } else if (semCls[i] != 255) {
            if (semCls[i] == c.sem) c.semVotes = static_cast<uint8_t>(std::min(10, c.semVotes + 1));
            else if (c.semVotes > 0) c.semVotes--;
            else { c.sem = semCls[i]; c.semVotes = 1; }
        }
        c.seen = t;
    }
    return true;
}

float LocalMap::occupancy(int i, int j) const {
    const float lo = grid_[static_cast<size_t>(j) * n_ + i].lo;
    return 1.0f / (1.0f + std::exp(-lo));
}

bool LocalMap::known(int i, int j) const {
    const Cell& c = grid_[static_cast<size_t>(j) * n_ + i];
    return std::fabs(c.lo) > 0.2f || c.drop > 0.5f;
}

uint8_t LocalMap::hazard(int i, int j) const {
    return grid_[static_cast<size_t>(j) * n_ + i].drop > 1.5f ? 1 : 0;
}

uint8_t LocalMap::semantic(int i, int j) const {
    const Cell& c = grid_[static_cast<size_t>(j) * n_ + i];
    return c.semVotes >= 2 ? c.sem : 255;
}

float LocalMap::obstacleAhead(float halfWidth, float maxDist, float minProb, float minDist) const {
    float dx, dy;
    aheadDir(dx, dy);
    const float minLo = std::log(minProb / (1 - minProb));
    float best = -1;
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const Cell& c = grid_[static_cast<size_t>(j) * n_ + i];
            if (c.lo < minLo) continue;
            const float x = (ox_ + i + 0.5f) * cell_ - ux_, y = (oy_ + j + 0.5f) * cell_ - uy_;
            const float along = x * dx + y * dy;
            const float lat = -x * dy + y * dx;
            if (along < minDist || along > maxDist || std::fabs(lat) > halfWidth) continue;
            if (best < 0 || along < best) best = along;
        }
    return best;
}

float LocalMap::hazardAhead(float halfWidth, float maxDist, float minDist) const {
    float dx, dy;
    aheadDir(dx, dy);
    float best = -1;
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const Cell& c = grid_[static_cast<size_t>(j) * n_ + i];
            if (c.drop <= 1.5f) continue;
            const float x = (ox_ + i + 0.5f) * cell_ - ux_, y = (oy_ + j + 0.5f) * cell_ - uy_;
            const float along = x * dx + y * dy;
            const float lat = -x * dy + y * dx;
            if (along < minDist || along > maxDist || std::fabs(lat) > halfWidth) continue;
            if (best < 0 || along < best) best = along;
        }
    return best;
}

void LocalMap::draw(uint8_t* rgba, int stride, int x0, int y0, int size) const {
    // size px shows an 8 m x 8 m square, user at the centre, heading up.
    const float span = 8.0f, s = span / size;
    for (int py = 0; py < size; ++py) {
        uint8_t* row = rgba + static_cast<size_t>(y0 + py) * stride * 4 + static_cast<size_t>(x0) * 4;
        for (int px = 0; px < size; ++px, row += 4) {
            const float rx = (px - size / 2) * s;     // right of heading
            const float ry = (size / 2 - py) * s;     // along heading
            // Plane: heading h, right = (hy, -hx).
            const float x = ux_ + ry * hx_ + rx * hy_;
            const float y = uy_ + ry * hy_ - rx * hx_;
            const int idx = index(floorDiv(x, cell_), floorDiv(y, cell_));
            uint8_t c[3] = {20, 20, 20};
            if (idx >= 0) {
                const Cell& cl = grid_[idx];
                if (cl.drop > 1.5f) { c[0] = 255; c[1] = 140; c[2] = 0; }
                else if (cl.sem == 4 && cl.semVotes >= 2 && cl.lo > -0.5f) { c[0] = 255; c[1] = 220; c[2] = 0; }  // stairs
                else if (cl.lo > 0.85f) { c[0] = 220; c[1] = 40; c[2] = 40; }
                else if (cl.lo < -0.4f) { c[0] = 30; c[1] = 90; c[2] = 45; }
            }
            // Blend 80% over the camera image.
            for (int k = 0; k < 3; ++k) row[k] = static_cast<uint8_t>((row[k] + 4 * c[k]) / 5);
        }
    }
    auto dot = [&](float x, float y, const uint8_t col[3], int r) {
        const float dx = x - ux_, dy = y - uy_;
        const float rx = dx * hy_ - dy * hx_, ry = dx * hx_ + dy * hy_;
        const int cx = size / 2 + static_cast<int>(rx / s), cy = size / 2 - static_cast<int>(ry / s);
        for (int j = -r; j <= r; ++j)
            for (int i = -r; i <= r; ++i) {
                const int px = cx + i, py = cy + j;
                if (px < 0 || py < 0 || px >= size || py >= size || i * i + j * j > r * r) continue;
                uint8_t* p = rgba + static_cast<size_t>(y0 + py) * stride * 4 + static_cast<size_t>(x0 + px) * 4;
                p[0] = col[0]; p[1] = col[1]; p[2] = col[2];
            }
    };
    const uint8_t purple[3] = {190, 90, 255}, white[3] = {255, 255, 255}, cyan[3] = {0, 220, 255};
    for (const MapMover& m : movers_) dot(m.x, m.y, purple, 4);
    dot(ux_, uy_, white, 4);
    dot(ux_ + 0.4f * hx_, uy_ + 0.4f * hy_, cyan, 2);   // heading tick
}
