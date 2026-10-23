#include "tracker.h"

#include <algorithm>
#include <cmath>

namespace {
// Process noise (acceleration spectral density) and measurement noise.
constexpr float kAccelNoise = 1.5f;   // (m/s^2)^2 -- walking people turn/stop
constexpr float kMeasNoiseBase = 0.02f;  // m^2 at 1 m; grows with distance^2 (stereo)
}  // namespace

void ObjectTracker::Kf::predict(float dt, float q) {
    p += v * dt;
    // P = F P F^T + Q  for F = [[1, dt], [0, 1]], white-acceleration Q.
    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    const float n00 = P00 + dt * (2 * P01 + dt * P11) + q * dt4 / 4;
    const float n01 = P01 + dt * P11 + q * dt3 / 2;
    const float n11 = P11 + q * dt2;
    P00 = n00; P01 = n01; P11 = n11;
}

void ObjectTracker::Kf::correct(float meas, float r) {
    if (!init) {
        p = meas; v = 0; P00 = r; P01 = 0; P11 = 4.0f; init = true;
        return;
    }
    const float s = P00 + r;
    const float k0 = P00 / s, k1 = P01 / s;
    const float y = meas - p;
    p += k0 * y;
    v += k1 * y;
    const float n00 = (1 - k0) * P00;
    const float n01 = (1 - k0) * P01;
    const float n11 = P11 - k1 * P01;
    P00 = n00; P01 = n01; P11 = n11;
}

float ObjectTracker::iou(float ax, float ay, float aw, float ah,
                         float bx, float by, float bw, float bh) {
    const float x1 = std::max(ax, bx), y1 = std::max(ay, by);
    const float x2 = std::min(ax + aw, bx + bw), y2 = std::min(ay + ah, by + bh);
    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float uni = aw * ah + bw * bh - inter;
    return uni > 0 ? inter / uni : 0.0f;
}

bool ObjectTracker::isMover(int cocoClass) {
    // person, bicycle, car, motorcycle, bus, train, truck, birds..animals (14-23)
    return cocoClass == 0 || cocoClass == 1 || cocoClass == 2 || cocoClass == 3 ||
           cocoClass == 5 || cocoClass == 6 || cocoClass == 7 ||
           (cocoClass >= 14 && cocoClass <= 23);
}

void ObjectTracker::setEgo(bool valid, const float R[9], const float t[3], const float velCam[3]) {
    ego_ = valid;
    if (!valid) return;
    for (int i = 0; i < 9; ++i) egoR_[i] = R[i];
    for (int i = 0; i < 3; ++i) egoT_[i] = t[i];
    // Camera velocity into world axes.
    for (int i = 0; i < 3; ++i)
        egoV_[i] = R[i * 3] * velCam[0] + R[i * 3 + 1] * velCam[1] + R[i * 3 + 2] * velCam[2];
}

void ObjectTracker::update(const std::vector<Detection>& dets, double tSec,
                           std::vector<TrackInfo>& info) {
    info.assign(dets.size(), TrackInfo{});

    // Predicted box positions for association.
    std::vector<float> px(tracks_.size()), py(tracks_.size());
    for (size_t t = 0; t < tracks_.size(); ++t) {
        const float dt = static_cast<float>(tSec - tracks_[t].lastUpdate);
        px[t] = tracks_[t].bx + tracks_[t].bvx * dt;
        py[t] = tracks_[t].by + tracks_[t].bvy * dt;
    }

    std::vector<int> detTrack(dets.size(), -1);
    std::vector<bool> trackUsed(tracks_.size(), false);

    // Greedy IoU matching in descending IoU order, for one confidence pass.
    auto matchPass = [&](bool high) {
        struct Pair { float iou; size_t d, t; };
        std::vector<Pair> pairs;
        for (size_t d = 0; d < dets.size(); ++d) {
            if (detTrack[d] >= 0) continue;
            if ((dets[d].confidence >= highThreshold) != high) continue;
            for (size_t t = 0; t < tracks_.size(); ++t) {
                if (trackUsed[t] || tracks_[t].classId != dets[d].classId) continue;
                const float v = iou(dets[d].x, dets[d].y, dets[d].w, dets[d].h,
                                    px[t], py[t], tracks_[t].bw, tracks_[t].bh);
                if (v >= matchIou) pairs.push_back({v, d, t});
            }
        }
        std::sort(pairs.begin(), pairs.end(),
                  [](const Pair& a, const Pair& b) { return a.iou > b.iou; });
        for (const Pair& p : pairs) {
            if (detTrack[p.d] >= 0 || trackUsed[p.t]) continue;
            detTrack[p.d] = static_cast<int>(p.t);
            trackUsed[p.t] = true;
        }
    };
    matchPass(true);
    matchPass(false);

    // Unmatched confident detections start new tracks.
    for (size_t d = 0; d < dets.size(); ++d) {
        if (detTrack[d] >= 0 || dets[d].confidence < highThreshold) continue;
        Track tr{};
        tr.id = nextId_++;
        tr.classId = dets[d].classId;
        tr.bx = dets[d].x; tr.by = dets[d].y; tr.bw = dets[d].w; tr.bh = dets[d].h;
        tr.lastSeen = tr.lastUpdate = tSec;
        tracks_.push_back(tr);
        trackUsed.push_back(true);
        detTrack[d] = static_cast<int>(tracks_.size() - 1);
    }

    // Update matched tracks with the measured 3D position.
    for (size_t d = 0; d < dets.size(); ++d) {
        if (detTrack[d] < 0) continue;
        Track& tr = tracks_[detTrack[d]];
        const Detection& det = dets[d];
        const float dt = std::max(1e-3f, static_cast<float>(tSec - tr.lastUpdate));

        // Box motion (for association), smoothed.
        if (tr.hits > 0) {
            const float a = 0.5f;
            tr.bvx = (1 - a) * tr.bvx + a * (det.x - tr.bx) / dt;
            tr.bvy = (1 - a) * tr.bvy + a * (det.y - tr.by) / dt;
        }
        tr.bx = det.x; tr.by = det.y; tr.bw = det.w; tr.bh = det.h;

        // Frame of the filters follows ego availability; restart on a switch.
        if (tr.world != ego_) {
            tr.kx = Kf{}; tr.ky = Kf{}; tr.kz = Kf{};
            tr.world = ego_;
        }
        if (tr.kx.init) {
            tr.kx.predict(dt, kAccelNoise);
            tr.ky.predict(dt, kAccelNoise);
            tr.kz.predict(dt, kAccelNoise);
        }
        if (det.distanceM > 0.0f) {
            const float Z = det.distanceM;
            const float u = det.x + det.w * 0.5f, v = det.y + det.h * 0.5f;
            const float X = (u - cx_) * Z / fx_;
            const float Y = (v - cy_) * Z / fy_;
            const float r = kMeasNoiseBase * std::max(1.0f, Z * Z);
            if (tr.world) {
                const float* R = egoR_;
                float w[3];
                for (int i = 0; i < 3; ++i) w[i] = R[i * 3] * X + R[i * 3 + 1] * Y + R[i * 3 + 2] * Z + egoT_[i];
                tr.kx.correct(w[0], r);
                tr.ky.correct(w[1], r);
                tr.kz.correct(w[2], r);
            } else {
                tr.kx.correct(X, r);
                tr.kz.correct(Z, r);
            }
        }
        tr.hits++;
        tr.lastSeen = tr.lastUpdate = tSec;
    }

    // Drop tracks not seen recently.
    std::vector<int> remap(tracks_.size(), -1);
    {
        std::vector<Track> kept;
        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (tSec - tracks_[t].lastSeen <= maxLostSec) {
                remap[t] = static_cast<int>(kept.size());
                kept.push_back(tracks_[t]);
            }
        }
        tracks_.swap(kept);
    }

    // Per-detection results + alert.
    alert_ = MotionAlert::None;
    alertTtc_ = -1;
    alertTrack_ = -1;
    for (size_t d = 0; d < dets.size(); ++d) {
        if (detTrack[d] < 0 || remap[detTrack[d]] < 0) continue;
        const Track& tr = tracks_[remap[detTrack[d]]];
        TrackInfo& ti = info[d];
        ti.id = tr.id;
        ti.confirmed = tr.hits >= confirmHits;
        if (!tr.kx.init) continue;

        float closing, range, relVx;
        bool movesItself;
        if (tr.world) {
            // World frame: object velocity is its own motion; relative to the
            // user subtract the user's velocity. Report in camera axes.
            const float* R = egoR_;
            const float rw[3] = {tr.kx.p - egoT_[0], tr.ky.p - egoT_[1], tr.kz.p - egoT_[2]};
            const float vo[3] = {tr.kx.v, tr.ky.v, tr.kz.v};
            const float vr[3] = {vo[0] - egoV_[0], vo[1] - egoV_[1], vo[2] - egoV_[2]};
            float rc[3], vc[3];   // R^T * (...)
            for (int i = 0; i < 3; ++i) {
                rc[i] = R[i] * rw[0] + R[3 + i] * rw[1] + R[6 + i] * rw[2];
                vc[i] = R[i] * vo[0] + R[3 + i] * vo[1] + R[6 + i] * vo[2];
            }
            ti.x = rc[0]; ti.z = rc[2];
            ti.vx = vc[0]; ti.vz = vc[2];
            ti.speed = std::sqrt(vo[0] * vo[0] + vo[1] * vo[1] + vo[2] * vo[2]);
            range = std::sqrt(rw[0] * rw[0] + rw[1] * rw[1] + rw[2] * rw[2]);
            closing = range > 0 ? -(rw[0] * vr[0] + rw[1] * vr[1] + rw[2] * vr[2]) / range : 0.0f;
            float vrc0 = R[0] * vr[0] + R[3] * vr[1] + R[6] * vr[2];
            relVx = vrc0;
            movesItself = ti.speed > minOwnSpeed;
        } else {
            ti.x = tr.kx.p; ti.z = tr.kz.p;
            ti.vx = tr.kx.v; ti.vz = tr.kz.v;
            ti.speed = std::sqrt(ti.vx * ti.vx + ti.vz * ti.vz);
            // Closing speed toward the user (the camera at the origin).
            range = std::sqrt(ti.x * ti.x + ti.z * ti.z);
            closing = range > 0 ? -(ti.x * ti.vx + ti.z * ti.vz) / range : 0.0f;
            relVx = ti.vx;
            // Without ego-motion a still chair "approaches" at the user's own
            // walking speed: only classes that move on their own count.
            movesItself = isMover(tr.classId);
        }
        if (ti.confirmed && closing > minClosingSpeed && movesItself) {
            ti.approaching = true;
            ti.ttc = range / closing;
            ti.lateralAtImpact = ti.x + relVx * ti.ttc;
            const bool inPath = std::fabs(ti.lateralAtImpact) <= corridorHalfWidth ||
                                std::fabs(ti.x) <= corridorHalfWidth;
            if (inPath) {
                MotionAlert a = ti.ttc < stopTtc ? MotionAlert::Stop
                              : ti.ttc < cautionTtc ? MotionAlert::Caution
                              : MotionAlert::None;
                if (a > alert_ || (a == alert_ && a != MotionAlert::None && ti.ttc < alertTtc_)) {
                    alert_ = a;
                    alertTtc_ = ti.ttc;
                    alertTrack_ = ti.id;
                }
            }
        }
    }
}
