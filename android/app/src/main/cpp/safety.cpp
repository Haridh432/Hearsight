#include "safety.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

constexpr float kBin = 0.10f;          // corridor bin length (m)
constexpr float kRadToDeg = 57.2957795f;

inline float dot(const float* a, const float* b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

// Smallest-eigenvalue eigenvector of a symmetric 3x3 matrix (Jacobi).
void smallestEigenvector(double A[3][3], float out[3]) {
    double V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int sweep = 0; sweep < 30; ++sweep) {
        int p = 0, q = 1;
        double maxOff = std::fabs(A[0][1]);
        if (std::fabs(A[0][2]) > maxOff) { maxOff = std::fabs(A[0][2]); p = 0; q = 2; }
        if (std::fabs(A[1][2]) > maxOff) { maxOff = std::fabs(A[1][2]); p = 1; q = 2; }
        if (maxOff < 1e-12) break;
        double theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
        double c = 1 / std::sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < 3; ++k) {
            double akp = A[k][p], akq = A[k][q];
            A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
            double apk = A[p][k], aqk = A[q][k];
            A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; ++k) {
            double vkp = V[k][p], vkq = V[k][q];
            V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
        }
    }
    int m = 0;
    if (A[1][1] < A[m][m]) m = 1;
    if (A[2][2] < A[m][m]) m = 2;
    for (int k = 0; k < 3; ++k) out[k] = static_cast<float>(V[k][m]);
}

}  // namespace


void SafetyLayer::setGravity(float gx, float gy, float gz) {
    float n = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (n < 1e-3f) return;
    gx_ = gx / n; gy_ = gy / n; gz_ = gz / n;
    haveGravity_ = true;
}

void SafetyLayer::reset() {
    floor_ = Plane{};
    floorTime_ = -1e9;
    for (int& h : history_) h = 0;
    histIdx_ = 0;
    held_ = SafetyResult{};
    heldUntil_ = -1e9;
    memoDist_ = -1;
    memoTime_ = -1e9;
    memoLevel_ = 0;
    prevNTime_ = -1e9;
    lastFarDropTime_ = -1e9;
    gravityFitFailSince_ = -1;
    floorSince_ = -1;
    nearHist_.clear();
    closing_ = 0;
    lastObs_ = lastHead_ = -1;
    lastObsTime_ = lastHeadTime_ = -1e9;
    for (int& d : dropHist_) d = 0;
    dropIdx_ = 0;
    heightSamples_.clear();
    learnedHeight_ = -1;
}

float SafetyLayer::minPointsAt(float along) const {
    // Sample density falls ~1/z^2; scale the evidence needed accordingly.
    const float a = std::max(along, 0.5f);
    return std::max(3.0f, 24.0f / (a * a) * (16.0f / (sampleStep * sampleStep)));
}

bool SafetyLayer::fitFloor(const std::vector<P>& pts, Plane& out, bool walkableOnly,
                           bool useGravity) {
    // Prior "up": previous floor if recent, else -gravity, else camera -y.
    float up[3];
    float tolDeg = maxFloorTiltDeg;
    if (haveGravity_ && useGravity) {
        // Gravity at this frame's capture time: the floor is level, so its
        // normal is "up" within the IMU / mount error, whatever the head does.
        up[0] = -gx_; up[1] = -gy_; up[2] = -gz_; tolDeg = gravityTiltDeg;
    } else if (floor_.valid) {
        // Tracked floor: head motion changes the tilt by a few degrees per
        // frame at most; a larger jump would be another surface.
        up[0] = floor_.nx; up[1] = floor_.ny; up[2] = floor_.nz; tolDeg = floorTrackTiltDeg;
    } else {
        up[0] = 0; up[1] = -1; up[2] = 0;
    }
    const float cosTol = std::cos(tolDeg / kRadToDeg);

    // Candidates: all points within 0.4-4 m. Walls fail the normal check and
    // ceilings fail the "camera above plane" check below.
    std::vector<const P*> cand;
    cand.reserve(pts.size());
    for (const P& p : pts) {
        if (p.z < 0.4f || p.z > 4.0f) continue;
        // With segmentation, fit only to pixels labelled floor / ground.
        if (walkableOnly && p.role != kSemWalkable) continue;
        cand.push_back(&p);
    }
    if (cand.size() < 50) return false;

    // Scoring subset.
    std::vector<const P*> sub;
    const size_t stride = std::max<size_t>(1, cand.size() / 1500);
    for (size_t i = 0; i < cand.size(); i += stride) sub.push_back(cand[i]);

    std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);
    int bestCount = 0;
    float bestScore = 0;
    float best[4] = {0, 0, 0, 0};
    for (int it = 0; it < ransacIters; ++it) {
        const P* a = cand[pick(rng_)];
        const P* b = cand[pick(rng_)];
        const P* c = cand[pick(rng_)];
        float e1[3] = {b->x - a->x, b->y - a->y, b->z - a->z};
        float e2[3] = {c->x - a->x, c->y - a->y, c->z - a->z};
        float n[3] = {e1[1]*e2[2] - e1[2]*e2[1], e1[2]*e2[0] - e1[0]*e2[2], e1[0]*e2[1] - e1[1]*e2[0]};
        float len = std::sqrt(dot(n, n));
        if (len < 1e-6f) continue;
        for (float& v : n) v /= len;
        if (dot(n, up) < 0) for (float& v : n) v = -v;
        if (dot(n, up) < cosTol) continue;
        const float d = -(n[0]*a->x + n[1]*a->y + n[2]*a->z);
        // Camera must be above the floor at a plausible height; once known,
        // close to the learned eye height and continuous with the tracked floor.
        if (d < eyeHeightMin || d > eyeHeightMax) continue;
        if (learnedHeight_ > 0 && std::fabs(d - learnedHeight_) > heightTolerance) continue;
        if (floor_.valid && std::fabs(d - floor_.d) > floorTrackTolerance) continue;
        // Inliers weighted by nearness: the floor is the surface at the
        // user's feet (beats a lower level beyond a step, and tabletops).
        int count = 0;
        float score = 0;
        for (const P* p : sub) {
            if (std::fabs(n[0]*p->x + n[1]*p->y + n[2]*p->z + d) < floorInlier) {
                ++count;
                score += 1.0f / std::max(0.5f, p->z);
            }
        }
        if (score > bestScore) {
            bestScore = score;
            bestCount = count;
            best[0] = n[0]; best[1] = n[1]; best[2] = n[2]; best[3] = d;
        }
    }
    if (bestCount < std::max<int>(60, static_cast<int>(sub.size() * 0.10))) return false;

    // Least-squares refinement on all inliers (PCA normal).
    double cx = 0, cy = 0, cz = 0; int n = 0;
    for (const P* p : cand) {
        if (std::fabs(best[0]*p->x + best[1]*p->y + best[2]*p->z + best[3]) < floorInlier) {
            cx += p->x; cy += p->y; cz += p->z; ++n;
        }
    }
    if (n < 30) return false;
    cx /= n; cy /= n; cz /= n;
    double C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (const P* p : cand) {
        if (std::fabs(best[0]*p->x + best[1]*p->y + best[2]*p->z + best[3]) < floorInlier) {
            double v[3] = {p->x - cx, p->y - cy, p->z - cz};
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) C[i][j] += v[i] * v[j];
        }
    }
    float nr[3];
    smallestEigenvector(C, nr);
    if (dot(nr, best) < 0) for (float& v : nr) v = -v;
    // Keep the refined normal within the same tilt limit as the search.
    if (dot(nr, up) < cosTol) { nr[0] = best[0]; nr[1] = best[1]; nr[2] = best[2]; }
    float dr = -static_cast<float>(nr[0]*cx + nr[1]*cy + nr[2]*cz);
    // The refinement can tilt onto a plane mixing two levels (near upper floor
    // + lower floor beyond a step); it must keep the learned eye height and
    // continuity like the hypotheses do, else keep the hypothesis plane.
    if ((learnedHeight_ > 0 && std::fabs(dr - learnedHeight_) > heightTolerance) ||
        (floor_.valid && std::fabs(dr - floor_.d) > floorTrackTolerance)) {
        nr[0] = best[0]; nr[1] = best[1]; nr[2] = best[2]; dr = best[3];
    }

    out.nx = nr[0]; out.ny = nr[1]; out.nz = nr[2]; out.d = dr; out.valid = true;
    return true;
}

SafetyResult SafetyLayer::process(const uint16_t* depth, const DepthIntrinsics& in,
                                  float units, double tSec, const SemanticInput* sem) {
    const auto t0 = std::chrono::steady_clock::now();
    SafetyResult r;
    const bool useSem = sem && sem->roles && sem->gridW > 0 && sem->gridH > 0 && sem->colorW > 0;
    r.semanticUsed = useSem;
    int walkable = 0;

    // 1. Sample + back-project.
    pts_.clear();
    for (int v = 0; v < in.height; v += sampleStep) {
        const uint16_t* row = depth + static_cast<size_t>(v) * in.width;
        for (int u = 0; u < in.width; u += sampleStep) {
            if (!row[u]) continue;
            const float z = row[u] * units;
            if (z < minRange || z > maxRange) continue;
            const float x = (u - in.ppx) * z / in.fx, y = (v - in.ppy) * z / in.fy;
            uint8_t role = kSemOther;
            if (useSem) {
                // Depth point -> colour camera -> segmentation cell.
                const float* R = sem->rot;
                const float cx = R[0]*x + R[3]*y + R[6]*z + sem->trans[0];
                const float cy = R[1]*x + R[4]*y + R[7]*z + sem->trans[1];
                const float cz = R[2]*x + R[5]*y + R[8]*z + sem->trans[2];
                if (cz > 0.05f) {
                    const float cu = sem->fx * cx / cz + sem->ppx;
                    const float cv = sem->fy * cy / cz + sem->ppy;
                    if (cu >= 0 && cv >= 0 && cu < sem->colorW && cv < sem->colorH) {
                        const int gx = static_cast<int>(cu * sem->gridW / sem->colorW);
                        const int gy = static_cast<int>(cv * sem->gridH / sem->colorH);
                        role = sem->roles[gy * sem->gridW + gx];
                    }
                }
                if (role == kSemWalkable) ++walkable;
            }
            pts_.push_back({x, y, z, u, v, role});
        }
    }
    r.points = static_cast<int>(pts_.size());

    // 2. Floor plane (with short memory, smoothed).
    Plane fit;
    // Walkable-only fit when the segmentation sees enough floor, else all points.
    const bool walkableOnly = useSem && walkable >= minWalkableForFit;
    // Gravity-pinned fit first; if the IMU / mount error is larger than the
    // tight tolerance and nothing fits, fall back to the tracked fit.
    bool fitted = fitFloor(pts_, fit, walkableOnly) || (walkableOnly && fitFloor(pts_, fit, false));
    if (haveGravity_) {
        if (fitted) gravityFitFailSince_ = -1;
        else if (gravityFitFailSince_ < 0) gravityFitFailSince_ = tSec;
        if (!fitted && tSec - gravityFitFailSince_ >= gravityFallbackSec)
            fitted = fitFloor(pts_, fit, false, false);
    }
    if (fitted) {
        if (floor_.valid) {
            // Light smoothing only: heavy smoothing lags head nods (~2 deg)
            // and makes the far floor look like a drop.
            const float a = 0.85f;
            floor_.nx = (1 - a) * floor_.nx + a * fit.nx;
            floor_.ny = (1 - a) * floor_.ny + a * fit.ny;
            floor_.nz = (1 - a) * floor_.nz + a * fit.nz;
            const float len = std::sqrt(floor_.nx*floor_.nx + floor_.ny*floor_.ny + floor_.nz*floor_.nz);
            floor_.nx /= len; floor_.ny /= len; floor_.nz /= len;
            floor_.d = (1 - a) * floor_.d + a * fit.d;
        } else {
            floor_ = fit;
        }
        floorTime_ = tSec;
        // Learn the camera's height above the floor from the first ~30
        // floor fits (median), then use it to reject false floors.
        if (learnedHeight_ < 0) {
            heightSamples_.push_back(floor_.d);
            if (heightSamples_.size() >= 30) {
                std::nth_element(heightSamples_.begin(), heightSamples_.begin() + 15, heightSamples_.end());
                learnedHeight_ = heightSamples_[15];
            }
        }
    } else if (tSec - floorTime_ > floorMemorySec) {
        floor_.valid = false;
        // Long without a floor: the learned height may be wrong (or the
        // mount moved). Re-learn from the next stable fits.
        if (tSec - floorTime_ > relearnSec && learnedHeight_ > 0) {
            learnedHeight_ = -1;
            heightSamples_.clear();
        }
    }

    int raw = 0;
    SafetyReason rawReason = SafetyReason::None;
    float rawDist = -1;
    // Real hazards outrank "can't see the floor" at the same level, then
    // the nearer one wins.
    auto weak = [](SafetyReason w) {
        return w == SafetyReason::UnknownFloor || w == SafetyReason::NoFloor;
    };
    auto consider = [&](int level, SafetyReason why, float dist) {
        bool better = level > raw;
        if (!better && level == raw && level > 0) {
            if (weak(rawReason) != weak(why)) better = !weak(why);
            else better = dist >= 0 && (rawDist < 0 || dist < rawDist);
        }
        if (better) { raw = level; rawReason = why; rawDist = dist; }
    };

    if (!floor_.valid) {
        floorSince_ = -1;
        // No floor: fall back to anything close straight ahead.
        int close = 0;
        float nearest = 1e9f;
        for (const P& p : pts_) {
            if (p.z < stopDist && std::fabs(p.x) < corridorHalfWidth) {
                ++close;
                nearest = std::min(nearest, p.z);
            }
        }
        if (close >= 20) consider(2, SafetyReason::Obstacle, nearest);
        else if (memoDist_ > 0 && tSec - memoTime_ < outOfViewHoldSec) consider(memoLevel_, memoReason_, memoDist_);
        else consider(1, SafetyReason::NoFloor, -1);
    } else {
        r.floorFound = true;
        if (floorSince_ < 0) floorSince_ = tSec;
        // Stable = this run of floor fits is self-consistent (whatever the
        // camera height): found for floorStableSec and steady within tol.
        if (tSec - floorSince_ < 1e-6) floorRunD_ = floor_.d;
        const bool steady = std::fabs(floor_.d - floorRunD_) <= floorStableTol;
        floorRunD_ = 0.9f * floorRunD_ + 0.1f * floor_.d;
        if (!steady) floorSince_ = tSec;   // restart the run
        // ...and, once the eye height is learned, close to it: a run of
        // wrong fits (seen at 1.16 m vs 1.5 m) filled the map with false
        // drops. A wrong learned height is re-learned after relearnSec.
        const bool nearEye = learnedHeight_ <= 0 || std::fabs(floor_.d - learnedHeight_) <= floorStableEyeTol;
        r.floorStable = steady && nearEye && tSec - floorSince_ >= floorStableSec;
        const float n[3] = {floor_.nx, floor_.ny, floor_.nz};
        r.floorNormal[0] = n[0]; r.floorNormal[1] = n[1]; r.floorNormal[2] = n[2];
        r.gravityUsed = haveGravity_;
        r.cameraHeight = floor_.d;
        // Pitch: angle between camera forward (z) and the floor plane.
        r.cameraPitchDeg = std::asin(std::max(-1.0f, std::min(1.0f, -n[2]))) * kRadToDeg;

        // 3-4. Corridor axes on the floor: forward f, lateral l.
        float f[3] = {0, 0, 1};
        const float fz = dot(f, n);
        for (int i = 0; i < 3; ++i) f[i] -= fz * n[i];
        float fl = std::sqrt(dot(f, f));
        if (fl < 1e-3f) { f[0] = 0; f[1] = 1; f[2] = 0; fl = 1; }  // looking straight down
        for (float& v : f) v /= fl;
        const float l[3] = {n[1]*f[2] - n[2]*f[1], n[2]*f[0] - n[0]*f[2], n[0]*f[1] - n[1]*f[0]};

        const int bins = static_cast<int>((corridorFar - corridorNear) / kBin + 0.5f);
        const float headTop = learnedHeight_ > 0 ? std::min(headMaxH, learnedHeight_ + headAboveEye) : headMaxH;
        std::vector<int> body(bins, 0), head(bins, 0), floorC(bins, 0), below(bins, 0), any(bins, 0);
        // Nearest point in each bin per category (exact distance reporting).
        std::vector<float> bodyMin(bins, 1e9f), headMin(bins, 1e9f), belowMin(bins, 1e9f);
        std::vector<float> floorMax(bins, -1.0f);   // farthest floor point per bin
        std::vector<int> stairs(bins, 0), low(bins, 0);
        std::vector<float> stairsMin(bins, 1e9f), lowMin(bins, 1e9f);
        // Trip hazards: local floor height per bin + candidate low points.
        std::vector<float> floorSum(bins, 0);
        struct TripCand { int b; float h, along; };
        std::vector<TripCand> tripCand;
        struct StripPt { int b; float h, along; uint8_t role; };
        std::vector<StripPt> strip;   // centre-strip points near / below the floor

        for (const P& p : pts_) {
            const float pv[3] = {p.x, p.y, p.z};
            const float along = dot(pv, f);
            if (along < corridorNear || along >= corridorFar) continue;
            const float lat = dot(pv, l);
            if (std::fabs(lat) > corridorHalfWidth) continue;
            const float h = dot(pv, n) + floor_.d;
            const int b = static_cast<int>((along - corridorNear) / kBin);
            any[b]++;
            if (h > bodyMinH && h <= bodyMaxH) { body[b]++; bodyMin[b] = std::min(bodyMin[b], along); }
            if (p.role == kSemStairs && h < 1.0f) { stairs[b]++; stairsMin[b] = std::min(stairsMin[b], along); }
            if (p.role == kSemObstacle && h > lowObstacleMinH && h <= bodyMinH) {
                low[b]++; lowMin[b] = std::min(lowMin[b], along);
            }
            else if (h > bodyMaxH && h <= headTop) { head[b]++; headMin[b] = std::min(headMin[b], along); }
            if (std::fabs(lat) < 0.30f) {
                if (along < tripRange && h > 0.0f && h <= bodyMinH + 0.03f) tripCand.push_back({b, h, along});
                if (std::fabs(h) < 0.05f) {
                    floorC[b]++; floorMax[b] = std::max(floorMax[b], along);
                    floorSum[b] += h;
                }
                // Below-floor candidates; judged against the local floor below.
                if (h < 0.3f) strip.push_back({b, h, along, p.role});
            }
        }

        // Trip hazards relative to the local floor of the same bin (or its
        // neighbours); needs enough points so depth noise doesn't trigger it.
        {
            std::vector<int> trip(bins, 0);
            std::vector<float> tripMinA(bins, 1e9f);
            auto localFloor = [&](int b, float& f) {
                int n = 0; float sum = 0;
                for (int k = std::max(0, b - 1); k <= std::min(bins - 1, b + 1); ++k) { n += floorC[k]; sum += floorSum[k]; }
                if (n < 5) return false;
                f = sum / n;
                return true;
            };
            for (const TripCand& c : tripCand) {
                float f;
                if (!localFloor(c.b, f)) continue;
                const float hl = c.h - f;
                if (hl >= tripMinH && hl <= bodyMinH) { trip[c.b]++; tripMinA[c.b] = std::min(tripMinA[c.b], c.along); }
            }
            for (int b = 0; b < bins; ++b) {
                const float a = corridorNear + b * kBin;
                if (a > tripRange) break;
                if (trip[b] >= std::max(static_cast<float>(tripMinPoints), 0.5f * minPointsAt(a))) { r.tripDist = tripMinA[b]; break; }
            }
        }

        // 5. Nearest body / head-height obstacle.
        for (int b = 0; b < bins; ++b) {
            const float a = corridorNear + b * kBin;
            if (r.obstacleDist < 0 && body[b] >= minPointsAt(a)) r.obstacleDist = bodyMin[b];
            if (r.headDist < 0 && head[b] >= minPointsAt(a)) r.headDist = headMin[b];
            // Stairs need a solid patch (not scattered mislabels): enough
            // points and a large share of the bin.
            if (r.stairsDist < 0 && stairs[b] >= 2 * minPointsAt(a) &&
                stairs[b] >= stairsMinShare * any[b]) r.stairsDist = stairsMin[b];
            if (r.lowObstacleDist < 0 && low[b] >= minPointsAt(a)) r.lowObstacleDist = lowMin[b];
        }

        // Local floor tracked along the centre strip: start at the fitted plane
        // (h = 0) and follow points within floorTrackBand bin by bin, so a slightly
        // mis-fitted floor (far floor 0.1-0.3 m "low") is still floor; a real step
        // jumps out of the band and stays below the tracked height.
        {
            std::vector<float> sum(bins, 0);
            std::vector<int> cnt(bins, 0);
            std::vector<float> ref(bins, 0);
            // Per-bin points in the band around the running reference.
            std::vector<std::vector<float>> hs(bins);
            for (const StripPt& q : strip) hs[q.b].push_back(q.h);
            // The track starts at the first bin where the floor is actually seen near the
            // fitted plane; bins before it have no measured floor (a glossy floor near the
            // feet gives no depth) and cannot show a drop - "unknown floor" covers them.
            // Phone: no floor depth 0.5-1.75 m, first points at -0.14 m -> false drop-off.
            float run = 0;
            bool tracking = false;
            std::vector<char> tracked(bins, 0);
            for (int b = 0; b < bins; ++b) {
                float s = 0; int c = 0;
                const float band = tracking ? floorTrackBand : floorTrackStartBand;
                for (float h : hs[b]) if (std::fabs(h - run) < band) { s += h; ++c; }
                if (c >= 4) { run = s / c; tracking = true; }
                ref[b] = run;
                tracked[b] = tracking;
                floorC[b] = std::max(floorC[b], c);   // tracked floor counts as floor here
            }
            for (const StripPt& q : strip) {
                if (!tracked[q.b]) continue;   // no floor seen yet nearer than this point
                const float below_local = ref[q.b] - q.h;
                if (below_local <= dropDepth) continue;
                // Mirror image of a wall / pillar / ceiling in a glossy floor.
                if (useSem && below_local > reflectMinDepth && q.role == kSemOther) continue;
                below[q.b]++; belowMin[q.b] = std::min(belowMin[q.b], q.along);
            }
        }

        // 6. Drop-off: floor falling away along the centre strip.
        for (int b = 0; b < bins; ++b) {
            const float a = corridorNear + b * kBin;
            if (a < 0.5f) continue;
            auto isBelow = [&](int k) {
                return k < bins && below[k] >= std::max(3.0f, minPointsAt(corridorNear + k * kBin) * 0.5f) &&
                       below[k] > floorC[k];
            };
            // A real step / stair continues downward: require 2 consecutive bins.
            if (isBelow(b) && isBelow(b + 1)) {
                // The edge occludes the lower floor right behind it (a
                // shadow of ~drop * distance / height), so report the edge:
                // the farthest upper-floor point before the drop.
                r.dropDist = belowMin[b];
                for (int k = b; k >= 0; --k) {
                    if (floorMax[k] > 0) { r.dropDist = std::min(r.dropDist, floorMax[k]); break; }
                }
                break;
            }
        }

        // Unknown floor: the floor at this distance should be in view but no
        // depth came back there (glass, black/shiny floor, hole).
        for (int b = 0; b < bins; ++b) {
            const float a = corridorNear + b * kBin;
            if (a < 0.5f || a > unknownCautionDist) continue;
            const float q[3] = {-floor_.d * n[0] + a * f[0], -floor_.d * n[1] + a * f[1],
                                -floor_.d * n[2] + a * f[2]};
            if (q[2] <= 0.1f) continue;
            const float u = in.fx * q[0] / q[2] + in.ppx;
            const float v = in.fy * q[1] / q[2] + in.ppy;
            const bool inView = u >= 0 && u < in.width && v >= 0 && v < in.height;
            if (inView && any[b] == 0) { r.unknownDist = a; break; }
        }

        // Head turn rate from the floor normal; ignore drop evidence during
        // fast nods.
        if (tSec > prevNTime_ && tSec - prevNTime_ < 0.2) {
            const float c = std::max(-1.0f, std::min(1.0f, dot(n, prevN_)));
            r.turnRateDegS = std::acos(c) * kRadToDeg / static_cast<float>(tSec - prevNTime_);
        }
        prevN_[0] = n[0]; prevN_[1] = n[1]; prevN_[2] = n[2];
        prevNTime_ = tSec;
        // Without gravity only: with it, the fit's tilt is pinned to the IMU
        // and floor-normal jumps near a step are not head motion.
        if (!haveGravity_ && r.turnRateDegS > maxTurnRateDegS) r.dropDist = -1;

        // Drop-off guards: trust a drop only when the floor fit sits at the
        // learned eye height, and when it persists over several frames.
        if (r.dropDist >= 0 && learnedHeight_ > 0 &&
            std::fabs(floor_.d - learnedHeight_) > heightTolerance) {
            r.dropDist = -1;
        }
        // Not before the eye height is learned: a wrong first fit (phone: camH 1.71 m
        // instead of 1.16 m right after START) makes the whole floor look like a drop.
        if (r.dropDist >= 0 && learnedHeight_ <= 0) r.dropDist = -1;
        if (r.dropDist >= nearDropDist) lastFarDropTime_ = tSec;
        else if (r.dropDist >= 0 && tSec - lastFarDropTime_ > nearDropSec) r.dropDist = -1;
        dropHist_[dropIdx_] = r.dropDist >= 0 ? 1 : 0;
        dropIdx_ = (dropIdx_ + 1) % 6;
        int dropSeen = 0;
        for (int d : dropHist_) dropSeen += d;
        if (dropSeen < dropConfirm) r.dropDist = -1;

        // Short persistence: an obstacle at the point-count threshold is
        // missed on some frames; keep the last detection for persistSec.
        if (r.obstacleDist >= 0) { lastObs_ = r.obstacleDist; lastObsTime_ = tSec; }
        else if (tSec - lastObsTime_ < persistSec) r.obstacleDist = lastObs_;
        if (r.headDist >= 0) { lastHead_ = r.headDist; lastHeadTime_ = tSec; }
        else if (tSec - lastHeadTime_ < persistSec) r.headDist = lastHead_;

        // Closing speed toward the nearest body / head obstacle: slope of a
        // line fitted to the distances of the last speedWindowSec.
        float nearest = -1;
        if (r.obstacleDist >= 0) nearest = r.obstacleDist;
        if (r.headDist >= 0 && (nearest < 0 || r.headDist < nearest)) nearest = r.headDist;
        if (nearest >= 0) {
            // A jump to a different (farther / much nearer) object restarts the fit.
            if (!nearHist_.empty() && std::fabs(nearHist_.back().second - nearest) > 0.5f)
                nearHist_.clear();
            nearHist_.push_back({tSec, nearest});
        }
        while (!nearHist_.empty() && tSec - nearHist_.front().first > speedWindowSec)
            nearHist_.erase(nearHist_.begin());
        float v = 0;
        if (nearHist_.size() >= 5 && nearHist_.back().first - nearHist_.front().first > 0.3) {
            double st = 0, sd = 0, stt = 0, std_ = 0;
            const double t0 = nearHist_.front().first;
            for (const auto& h : nearHist_) {
                const double t = h.first - t0;
                st += t; sd += h.second; stt += t * t; std_ += t * h.second;
            }
            const double k = static_cast<double>(nearHist_.size());
            const double den = k * stt - st * st;
            if (den > 1e-9) v = static_cast<float>(-(k * std_ - st * sd) / den);
        }
        closing_ = std::max(0.0f, std::min(3.0f, v));
        r.closingSpeed = closing_;
        // Walking speed from odometry along the corridor direction.
        float approach = closing_;
        if (haveEgo_) {
            r.walkSpeed = std::max(0.0f, egoV_[0] * f[0] + egoV_[1] * f[1] + egoV_[2] * f[2]);
            approach = std::max(approach, std::min(3.0f, r.walkSpeed));
        }
        const float stopD = std::max(stopDist, stopMargin + approach * reactionSec);
        const float cautionD = std::max(cautionDist, stopD + cautionExtra);
        r.stopDistUsed = stopD;

        // 7a. Raw decision.
        auto level = [&](float dist, float stop, float caution) {
            return dist < 0 ? 0 : dist < stop ? 2 : dist < caution ? 1 : 0;
        };
        consider(level(r.obstacleDist, stopD, cautionD), SafetyReason::Obstacle, r.obstacleDist);
        consider(level(r.headDist, stopD, cautionD), SafetyReason::HeadHeight, r.headDist);

        // Dynamic map (Phase 4): remembered obstacle / drop-off in the
        // walking direction, including out of view. CAUTION only.
        r.mapObstacle = mapObstacle_;
        r.mapHazard = mapHazard_;
        if (mapObstacle_ >= 0 && mapObstacle_ < cautionD) consider(1, SafetyReason::Obstacle, mapObstacle_);
        if (mapHazard_ >= 0 && mapHazard_ < dropCautionDist) consider(1, SafetyReason::DropOff, mapHazard_);

        // Out-of-view memory: remember a near obstacle; if it is no longer
        // detected because its place on the floor left the view (head
        // raised), keep the STOP for outOfViewHoldSec.
        const int nearLevel = nearest < 0 ? 0 : nearest < stopD ? 2 : nearest < cautionD ? 1 : 0;
        const bool memoFresh = memoDist_ > 0 && tSec - memoTime_ < outOfViewHoldSec;
        // Never lower a fresh remembered alert (while the head comes up the
        // obstacle is briefly seen farther away).
        if (nearLevel > 0 && !(memoFresh && nearLevel < memoLevel_)) {
            memoLevel_ = nearLevel;
            memoDist_ = nearest;
            memoTime_ = tSec;
            memoReason_ = (r.headDist >= 0 && r.headDist <= nearest) ? SafetyReason::HeadHeight
                                                                     : SafetyReason::Obstacle;
        } else if (memoDist_ > 0 && tSec - memoTime_ < outOfViewHoldSec) {
            // Forget it only when that stretch of floor is actually seen
            // again and empty: enough floor points, no body points. Being
            // inside the image is not enough (the bottom edge has patchy depth).
            const int b = static_cast<int>((memoDist_ - corridorNear) / kBin);
            bool seenClear = false;
            if (b >= 0 && b < bins) {
                const float need = minPointsAt(memoDist_);
                seenClear = floorC[b] >= need && body[b] < need * 0.5f;
            }
            if (seenClear) memoDist_ = -1;
            else consider(memoLevel_, memoReason_, memoDist_);
        }
        // A drop with stairs labelled near it is reported as stairs.
        const bool dropIsStairs = r.dropDist >= 0 && r.stairsDist >= 0 &&
                                  std::fabs(r.stairsDist - r.dropDist) < 0.6f;
        consider(level(r.dropDist, dropStopDist, dropCautionDist),
                 dropIsStairs ? SafetyReason::Stairs : SafetyReason::DropOff, r.dropDist);
        // Stairs seen by the segmentation (up or down): CAUTION in range.
        if (r.stairsDist >= 0 && r.stairsDist < stairsCautionDist)
            consider(1, SafetyReason::Stairs, r.stairsDist);
        // Trip hazard (3-10 cm above the local floor): CAUTION.
        if (r.tripDist >= 0) consider(1, SafetyReason::TripHazard, r.tripDist);
        // Low objects (4-10 cm) the height bands miss, labelled as obstacles.
        consider(level(r.lowObstacleDist, stopDist, cautionDist), SafetyReason::Obstacle, r.lowObstacleDist);
        if (r.unknownDist >= 0) consider(1, SafetyReason::UnknownFloor, r.unknownDist);
    }

    // 7b. Confirmation: a level counts when seen in 2 of the last 3 frames.
    history_[histIdx_] = raw;
    histIdx_ = (histIdx_ + 1) % 3;
    auto seen = [&](int lvl) { int c = 0; for (int h : history_) if (h >= lvl) ++c; return c >= 2; };
    const int confirmed = seen(2) ? 2 : seen(1) ? 1 : 0;

    r.state = static_cast<SafetyState>(confirmed);
    if (confirmed > 0) {
        r.reason = raw >= confirmed ? rawReason : held_.reason;
        r.distance = raw >= confirmed ? rawDist : held_.distance;
    }

    // 7c. Hold: keep a warning for holdSec so it doesn't flicker.
    if (confirmed > 0 && confirmed >= static_cast<int>(held_.state)) {
        held_ = r;
        heldUntil_ = tSec + holdSec;
    } else if (tSec < heldUntil_ && static_cast<int>(held_.state) > confirmed) {
        r.state = held_.state;
        r.reason = held_.reason;
        r.distance = held_.distance;
    } else if (tSec >= heldUntil_) {
        held_ = SafetyResult{};
    }

    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}
