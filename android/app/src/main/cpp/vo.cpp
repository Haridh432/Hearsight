#include "vo.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "recorder.h"
#include "rigid.h"

namespace {

constexpr int kLevels = 3;
constexpr int kWin = 4;          // KLT half window (9x9)
constexpr int kIters = 10;
constexpr float kMargin = 6.0f;  // keep points away from the image border

inline float sample(const std::vector<float>& img, int w, int h, float x, float y) {
    x = std::max(0.0f, std::min(x, w - 1.001f));
    y = std::max(0.0f, std::min(y, h - 1.001f));
    const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    const float ax = x - x0, ay = y - y0;
    const float* r0 = img.data() + y0 * w + x0;
    const float* r1 = r0 + w;
    return (1 - ay) * ((1 - ax) * r0[0] + ax * r0[1]) + ay * ((1 - ax) * r1[0] + ax * r1[1]);
}

}  // namespace

void registerDepthHalf(const uint16_t* depth, float units, const DepthToColor& c, int step,
                       std::vector<float>& out) {
    const int hw = c.cw / 2, hh = c.ch / 2;
    out.assign(static_cast<size_t>(hw) * hh, 0.0f);
    const float* R = c.rot;
    for (int v = 0; v < c.dh; v += step) {
        const uint16_t* row = depth + static_cast<size_t>(v) * c.dw;
        const float yn = (v - c.dcy) / c.dfy;
        for (int u = 0; u < c.dw; u += step) {
            if (!row[u]) continue;
            const float z = row[u] * units;
            if (z < 0.2f || z > 8.0f) continue;
            const float x = (u - c.dcx) / c.dfx * z, y = yn * z;
            const float X = R[0] * x + R[3] * y + R[6] * z + c.trans[0];
            const float Y = R[1] * x + R[4] * y + R[7] * z + c.trans[1];
            const float Z = R[2] * x + R[5] * y + R[8] * z + c.trans[2];
            if (Z < 0.1f) continue;
            const int cu = static_cast<int>((c.cfx * X / Z + c.ccx) * 0.5f);
            const int cv = static_cast<int>((c.cfy * Y / Z + c.ccy) * 0.5f);
            if (cu < 0 || cv < 0 || cu >= hw || cv >= hh) continue;
            float& o = out[static_cast<size_t>(cv) * hw + cu];
            if (o == 0.0f || Z < o) o = Z;
        }
    }
}

float depthHalfAt(const std::vector<float>& map, int cw, int ch, float u, float v) {
    const int hw = cw / 2, hh = ch / 2;
    const int x = static_cast<int>(u * 0.5f), y = static_cast<int>(v * 0.5f);
    float best = 0;
    for (int j = -1; j <= 1; ++j)
        for (int i = -1; i <= 1; ++i) {
            const int xx = x + i, yy = y + j;
            if (xx < 0 || yy < 0 || xx >= hw || yy >= hh) continue;
            const float z = map[static_cast<size_t>(yy) * hw + xx];
            if (z > 0 && (best == 0 || z < best)) best = z;
        }
    return best;
}

void VisualOdometry::setCamera(const VoCamera& cam) { cam_ = cam; }

void VisualOdometry::reset() {
    prev_.clear();
    cur_.clear();
    tracks_.clear();
    havePrev_ = false;
    for (int i = 0; i < 9; ++i) Rw_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    tw_[0] = tw_[1] = tw_[2] = 0;
    vel_[0] = vel_[1] = vel_[2] = 0;
    angRate_ = 0;
}

void rgbaToHalfGray(const uint8_t* rgba, int W, int H, std::vector<uint8_t>& out) {
    const int w = W / 2, h = H / 2;
    out.resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
        const uint8_t* a = rgba + static_cast<size_t>(2 * y) * W * 4;
        const uint8_t* b = a + static_cast<size_t>(W) * 4;
        for (int x = 0; x < w; ++x) {
            int s = 0;
            for (const uint8_t* p : {a + 8 * x, a + 8 * x + 4, b + 8 * x, b + 8 * x + 4})
                s += 77 * p[0] + 150 * p[1] + 29 * p[2];
            out[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(s / (4 * 256));
        }
    }
}

void VisualOdometry::buildPyramid(const uint8_t* gray, std::vector<Level>& pyr) const {
    pyr.resize(kLevels);
    Level& l0 = pyr[0];
    l0.w = cam_.width / 2;
    l0.h = cam_.height / 2;
    l0.img.resize(static_cast<size_t>(l0.w) * l0.h);
    for (size_t i = 0; i < l0.img.size(); ++i) l0.img[i] = gray[i] * (1.0f / 255);
    for (int l = 1; l < kLevels; ++l) {
        const Level& s = pyr[l - 1];
        Level& d = pyr[l];
        d.w = s.w / 2;
        d.h = s.h / 2;
        d.img.resize(static_cast<size_t>(d.w) * d.h);
        for (int y = 0; y < d.h; ++y)
            for (int x = 0; x < d.w; ++x) {
                const float* r = s.img.data() + (2 * y) * s.w + 2 * x;
                d.img[y * d.w + x] = 0.25f * (r[0] + r[1] + r[s.w] + r[s.w + 1]);
            }
    }
}

bool VisualOdometry::trackPoint(const std::vector<Level>& from, const std::vector<Level>& to,
                                float x, float y, float& ox, float& oy) const {
    float dx = 0, dy = 0;   // displacement at the current level
    for (int l = kLevels - 1; l >= 0; --l) {
        const Level& A = from[l];
        const Level& B = to[l];
        const float s = 1.0f / (1 << l);
        const float px = x * s, py = y * s;
        if (l < kLevels - 1) { dx *= 2; dy *= 2; }
        // Template and gradients.
        float T[(2 * kWin + 1) * (2 * kWin + 1)], Gx[(2 * kWin + 1) * (2 * kWin + 1)],
            Gy[(2 * kWin + 1) * (2 * kWin + 1)];
        double gxx = 0, gxy = 0, gyy = 0;
        int k = 0;
        for (int j = -kWin; j <= kWin; ++j)
            for (int i = -kWin; i <= kWin; ++i, ++k) {
                const float u = px + i, v = py + j;
                T[k] = sample(A.img, A.w, A.h, u, v);
                Gx[k] = 0.5f * (sample(A.img, A.w, A.h, u + 1, v) - sample(A.img, A.w, A.h, u - 1, v));
                Gy[k] = 0.5f * (sample(A.img, A.w, A.h, u, v + 1) - sample(A.img, A.w, A.h, u, v - 1));
                gxx += Gx[k] * Gx[k];
                gxy += Gx[k] * Gy[k];
                gyy += Gy[k] * Gy[k];
            }
        const double det = gxx * gyy - gxy * gxy;
        if (det < 1e-10) return false;
        for (int it = 0; it < kIters; ++it) {
            double bx = 0, by = 0;
            k = 0;
            for (int j = -kWin; j <= kWin; ++j)
                for (int i = -kWin; i <= kWin; ++i, ++k) {
                    const float e = T[k] - sample(B.img, B.w, B.h, px + dx + i, py + dy + j);
                    bx += e * Gx[k];
                    by += e * Gy[k];
                }
            const float ux = static_cast<float>((gyy * bx - gxy * by) / det);
            const float uy = static_cast<float>((gxx * by - gxy * bx) / det);
            dx += ux;
            dy += uy;
            if (ux * ux + uy * uy < 1e-4f) break;
        }
    }
    ox = x + dx;
    oy = y + dy;
    const Level& L0 = to[0];
    return ox >= kMargin && oy >= kMargin && ox < L0.w - kMargin && oy < L0.h - kMargin;
}

void VisualOdometry::detect(const std::vector<Level>& pyr) {
    const Level& L = pyr[0];
    const int cw = L.w / gridCols, ch = L.h / gridRows;
    std::vector<int> occupied(gridCols * gridRows, 0);
    for (const Track& t : tracks_) {
        const int c = std::min(gridCols - 1, static_cast<int>(t.x) / cw);
        const int r = std::min(gridRows - 1, static_cast<int>(t.y) / ch);
        occupied[r * gridCols + c]++;
    }
    // Best corner per free cell (Shi-Tomasi min eigenvalue), then a threshold
    // relative to the strongest corner of the frame so plain walls / floors
    // still give points, with a small absolute floor against sensor noise.
    struct Cand { float e, x, y; };
    std::vector<Cand> best(gridCols * gridRows, Cand{0, -1, -1});
    float frameMax = 0;
    for (int r = 0; r < gridRows; ++r) {
        for (int c = 0; c < gridCols; ++c) {
            if (occupied[r * gridCols + c] >= 2) continue;
            Cand& bc = best[r * gridCols + c];
            for (int y = std::max(r * ch, static_cast<int>(kMargin));
                 y < std::min((r + 1) * ch, L.h - static_cast<int>(kMargin)); y += 2) {
                for (int x = std::max(c * cw, static_cast<int>(kMargin));
                     x < std::min((c + 1) * cw, L.w - static_cast<int>(kMargin)); x += 2) {
                    double a = 0, b = 0, d = 0;
                    for (int j = -2; j <= 2; ++j)
                        for (int i = -2; i <= 2; ++i) {
                            const float* p = L.img.data() + (y + j) * L.w + (x + i);
                            const float gx = 0.5f * (p[1] - p[-1]);
                            const float gy = 0.5f * (p[L.w] - p[-L.w]);
                            a += gx * gx; b += gx * gy; d += gy * gy;
                        }
                    a /= 25; b /= 25; d /= 25;
                    const float e = static_cast<float>(0.5 * (a + d - std::sqrt((a - d) * (a - d) + 4 * b * b)));
                    if (e > bc.e) bc = {e, static_cast<float>(x), static_cast<float>(y)};
                }
            }
            frameMax = std::max(frameMax, bc.e);
        }
    }
    const float thr = std::max(minEigenAbs, minEigenRel * frameMax);
    for (const Cand& bc : best) {
        if (static_cast<int>(tracks_.size()) >= maxPoints) break;
        if (bc.x >= 0 && bc.e > thr) tracks_.push_back({bc.x, bc.y, {0, 0, 0}, false});
    }
}

bool VisualOdometry::backProject(float hx, float hy, VoDepthFn depth, void* ctx, float P[3]) const {
    // Half-res pixel (hx, hy) covers full-res pixels 2hx..2hx+1: centre 2hx + 0.5.
    const float u = 2 * hx + 0.5f, v = 2 * hy + 0.5f;
    const float z = depth(ctx, u, v);
    if (!(z > 0.3f && z < 6.0f)) return false;
    P[0] = (u - cam_.cx) * z / cam_.fx;
    P[1] = (v - cam_.cy) * z / cam_.fy;
    P[2] = z;
    return true;
}

VoResult VisualOdometry::process(const uint8_t* rgba, double tSec, VoDepthFn depth, void* ctx) {
    std::vector<uint8_t> gray;
    rgbaToHalfGray(rgba, cam_.width, cam_.height, gray);
    return processGray(gray.data(), tSec, depth, ctx);
}

VoResult VisualOdometry::processGray(const uint8_t* gray, double tSec, VoDepthFn depth, void* ctx) {
    const auto t0 = std::chrono::steady_clock::now();
    VoResult r;
    buildPyramid(gray, cur_);

    if (havePrev_) {
        // Track every point forward, then back, and keep consistent ones.
        std::vector<Track> next;
        std::vector<std::pair<int, int>> pairs;   // (index into tracks_, index into next)
        next.reserve(tracks_.size());
        for (size_t i = 0; i < tracks_.size(); ++i) {
            const Track& t = tracks_[i];
            float nx, ny, bx, by;
            if (!trackPoint(prev_, cur_, t.x, t.y, nx, ny)) continue;
            if (!trackPoint(cur_, prev_, nx, ny, bx, by)) continue;
            if ((bx - t.x) * (bx - t.x) + (by - t.y) * (by - t.y) > fbMaxErr * fbMaxErr) continue;
            Track n{nx, ny, {0, 0, 0}, false};
            n.hasP = backProject(nx, ny, depth, ctx, n.P);
            pairs.push_back({static_cast<int>(i), static_cast<int>(next.size())});
            next.push_back(n);
        }
        r.tracked = static_cast<int>(pairs.size());

        // 3D-3D pairs (previous -> current camera frame).
        std::vector<std::array<float, 3>> P, Q;
        for (const auto& pr : pairs) {
            const Track& a = tracks_[pr.first];
            const Track& b = next[pr.second];
            if (!a.hasP || !b.hasP) continue;
            P.push_back({a.P[0], a.P[1], a.P[2]});
            Q.push_back({b.P[0], b.P[1], b.P[2]});
        }
        r.withDepth = static_cast<int>(P.size());

        const double dt = tSec - prevT_;
        if (r.withDepth >= minInliers && dt > 1e-3 && dt < 0.5) {
            auto thr = [](float z) { return 0.02f + 0.012f * z * z; };   // stereo error grows ~z^2
            std::uniform_int_distribution<size_t> pick(0, P.size() - 1);
            std::vector<char> best, cur(P.size());
            int bestCount = 0;
            float R[9], t[3];
            for (int it = 0; it < ransacIters; ++it) {
                const size_t a = pick(rng_), b = pick(rng_), c = pick(rng_);
                if (a == b || b == c || a == c) continue;
                const float p3[3][3] = {{P[a][0], P[a][1], P[a][2]}, {P[b][0], P[b][1], P[b][2]},
                                        {P[c][0], P[c][1], P[c][2]}};
                const float q3[3][3] = {{Q[a][0], Q[a][1], Q[a][2]}, {Q[b][0], Q[b][1], Q[b][2]},
                                        {Q[c][0], Q[c][1], Q[c][2]}};
                if (!fitRigid(p3, q3, 3, R, t)) continue;
                int count = 0;
                for (size_t i = 0; i < P.size(); ++i) {
                    float e[3];
                    for (int k = 0; k < 3; ++k)
                        e[k] = R[k * 3] * P[i][0] + R[k * 3 + 1] * P[i][1] + R[k * 3 + 2] * P[i][2] + t[k] - Q[i][k];
                    const float th = thr(Q[i][2]);
                    cur[i] = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) < th * th;
                    count += cur[i];
                }
                if (count > bestCount) { bestCount = count; best = cur; }
            }
            // Refit on inliers (twice: refit, re-select, refit).
            for (int pass = 0; pass < 2 && bestCount >= minInliers; ++pass) {
                std::vector<std::array<float, 3>> pi, qi;
                for (size_t i = 0; i < P.size(); ++i)
                    if (best[i]) { pi.push_back(P[i]); qi.push_back(Q[i]); }
                fitRigid(reinterpret_cast<const float(*)[3]>(pi.data()),
                         reinterpret_cast<const float(*)[3]>(qi.data()), pi.size(), R, t);
                bestCount = 0;
                for (size_t i = 0; i < P.size(); ++i) {
                    float e[3];
                    for (int k = 0; k < 3; ++k)
                        e[k] = R[k * 3] * P[i][0] + R[k * 3 + 1] * P[i][1] + R[k * 3 + 2] * P[i][2] + t[k] - Q[i][k];
                    const float th = thr(Q[i][2]);
                    best[i] = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) < th * th;
                    bestCount += best[i];
                }
            }
            r.inliers = bestCount;
            bool plausible = true;
            if (bestCount >= minInliers) {
                const float step = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
                const float tr0 = std::max(-1.0f, std::min(3.0f, R[0] + R[4] + R[8]));
                const float ang0 = std::acos(std::max(-1.0f, std::min(1.0f, (tr0 - 1) / 2)));
                plausible = step <= maxSpeed * dt + 0.02f && ang0 <= maxAngRate * dt + 0.02f;
            }
            if (bestCount >= minInliers && plausible) {
                r.ok = true;
                // Points: q = R p + t  =>  current camera in previous frame:
                // Rc = R^T, tc = -R^T t. Displacement in current axes = -t.
                float Rc[9], tc[3];
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) Rc[i * 3 + j] = R[j * 3 + i];
                for (int i = 0; i < 3; ++i) tc[i] = -(Rc[i * 3] * t[0] + Rc[i * 3 + 1] * t[1] + Rc[i * 3 + 2] * t[2]);
                float Rn[9], tn[3];
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j)
                        Rn[i * 3 + j] = Rw_[i * 3] * Rc[j] + Rw_[i * 3 + 1] * Rc[3 + j] + Rw_[i * 3 + 2] * Rc[6 + j];
                    tn[i] = Rw_[i * 3] * tc[0] + Rw_[i * 3 + 1] * tc[1] + Rw_[i * 3 + 2] * tc[2] + tw_[i];
                }
                for (int i = 0; i < 9; ++i) Rw_[i] = Rn[i];
                for (int i = 0; i < 3; ++i) tw_[i] = tn[i];
                const float a = velSmooth;
                for (int i = 0; i < 3; ++i) vel_[i] = (1 - a) * vel_[i] + a * (-t[i] / static_cast<float>(dt));
                const float tr = std::max(-1.0f, std::min(3.0f, R[0] + R[4] + R[8]));
                const float ang = std::acos(std::max(-1.0f, std::min(1.0f, (tr - 1) / 2)));
                angRate_ = (1 - a) * angRate_ + a * ang / static_cast<float>(dt);
            }
        }
        if (!r.ok) {
            // No estimate: let the velocity decay rather than freeze.
            for (float& v : vel_) v *= 0.8f;
            angRate_ *= 0.8f;
        }
        tracks_.swap(next);
    } else {
        tracks_.clear();
    }

    // Top up features, and fill depth for new ones.
    const size_t before = tracks_.size();
    if (static_cast<int>(tracks_.size()) < maxPoints * 3 / 4) detect(cur_);
    for (size_t i = before; i < tracks_.size(); ++i)
        tracks_[i].hasP = backProject(tracks_[i].x, tracks_[i].y, depth, ctx, tracks_[i].P);

    prev_.swap(cur_);
    prevT_ = tSec;
    havePrev_ = true;
    for (int i = 0; i < 3; ++i) r.vel[i] = vel_[i];
    r.speed = std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1] + vel_[2] * vel_[2]);
    r.angRate = angRate_;
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

void VoThread::start(const VoCamera& cam, const DepthToColor& reg) {
    stop();
    vo_.setCamera(cam);
    vo_.reset();
    reg_ = reg;
    quit_ = false;
    resT_ = -1;
    running_ = true;
    thread_ = std::thread(&VoThread::loop, this);
}

void VoThread::stop() {
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    running_ = false;
}

void VoThread::reset() {
    std::lock_guard<std::mutex> lk(m_);
    resetReq_ = true;
    resT_ = -1;
}

void VoThread::submit(const uint8_t* rgba, const uint16_t* depth, float depthUnits, double tSec,
                      const RecImu* imu) {
    std::unique_lock<std::mutex> lk(m_);
    if (!running_ || busy_ || pending_) return;
    rgba_.assign(rgba, rgba + static_cast<size_t>(reg_.cw) * reg_.ch * 4);
    depth_.assign(depth, depth + static_cast<size_t>(reg_.dw) * reg_.dh);
    units_ = depthUnits;
    t_ = tSec;
    imu_ = imu ? *imu : RecImu{};
    pending_ = true;
    lk.unlock();
    cv_.notify_one();
}

bool VoThread::latest(VoResult& r, float R[9], float t[3], double& tSec) const {
    std::lock_guard<std::mutex> lk(m_);
    if (resT_ < 0) return false;
    r = res_;
    for (int i = 0; i < 9; ++i) R[i] = R_[i];
    for (int i = 0; i < 3; ++i) t[i] = T_[i];
    tSec = resT_;
    return true;
}

void VoThread::loop() {
    std::vector<uint8_t> rgba;
    std::vector<uint16_t> depth;
    std::vector<float> half;
    std::vector<uint8_t> gray;
    for (;;) {
        double t;
        float units;
        RecImu imu;
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return quit_ || pending_; });
            if (quit_) break;
            rgba.swap(rgba_);
            depth.swap(depth_);
            t = t_;
            units = units_;
            imu = imu_;
            pending_ = false;
            busy_ = true;
            if (resetReq_) { vo_.reset(); resetReq_ = false; }
        }
        const auto t0 = std::chrono::steady_clock::now();
        registerDepthHalf(depth.data(), units, reg_, 2, half);
        rgbaToHalfGray(rgba.data(), reg_.cw, reg_.ch, gray);
        struct Ctx { const std::vector<float>* m; int w, h; } ctx{&half, reg_.cw, reg_.ch};
        VoResult r = vo_.processGray(gray.data(), t,
                                 [](void* c, float u, float v) {
                                     const Ctx* x = static_cast<const Ctx*>(c);
                                     return depthHalfAt(*x->m, x->w, x->h, u, v);
                                 }, &ctx);
        r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (rec_ && rec_->active()) rec_->push(t, gray.data(), depth.data(), r, vo_.poseT(), imu);
        std::lock_guard<std::mutex> lk(m_);
        res_ = r;
        for (int i = 0; i < 9; ++i) R_[i] = vo_.poseR()[i];
        for (int i = 0; i < 3; ++i) T_[i] = vo_.poseT()[i];
        resT_ = t;
        busy_ = false;
    }
}
