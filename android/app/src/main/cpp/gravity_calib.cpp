#include "gravity_calib.h"
#include "rigid.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float kRadToDeg = 57.2957795f;

float angleDeg(const float a[3], const float b[3]) {
    const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return std::acos(std::max(-1.0f, std::min(1.0f, d))) * kRadToDeg;
}

}  // namespace

void GravityCalib::reset() {
    pairs_.clear();
    lastAdd_ = -1e9;
    sinceSolve_ = 0;
    ok_ = false;
    spreadDeg_ = residualDeg_ = inlierFrac_ = 0;
}

void GravityCalib::add(const float phoneUp[3], const float camUp[3], double tSec) {
    if (tSec - lastAdd_ < minIntervalSec) return;
    lastAdd_ = tSec;
    Pair p;
    for (int i = 0; i < 3; ++i) { p.p[i] = phoneUp[i]; p.c[i] = camUp[i]; }
    pairs_.push_back(p);
    if (pairs_.size() > maxSamples) pairs_.erase(pairs_.begin());
    if (++sinceSolve_ >= 10 && pairs_.size() >= minSamples) {
        sinceSolve_ = 0;
        solve();
    }
}

void GravityCalib::solve() {
    // Orientation spread: largest angle between any sample and the mean.
    float mean[3] = {0, 0, 0};
    for (const Pair& p : pairs_) for (int i = 0; i < 3; ++i) mean[i] += p.p[i];
    const float ml = std::sqrt(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
    if (ml < 1e-6f) return;
    for (float& v : mean) v /= ml;
    spreadDeg_ = 0;
    for (const Pair& p : pairs_) spreadDeg_ = std::max(spreadDeg_, angleDeg(p.p, mean));

    // Horn: B = sum c * p^T ; N (4x4) from B; R from N's top eigenvector.
    auto fit = [&](const std::vector<char>& use, float R[9]) {
        double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        for (size_t k = 0; k < pairs_.size(); ++k) {
            if (!use[k]) continue;
            const Pair& p = pairs_[k];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) S[i][j] += static_cast<double>(p.p[i]) * p.c[j];
        }
        hornRotation(S, R);
    };
    auto errorDeg = [&](const float R[9], const Pair& p) {
        float c[3];
        for (int i = 0; i < 3; ++i) c[i] = R[i * 3] * p.p[0] + R[i * 3 + 1] * p.p[1] + R[i * 3 + 2] * p.p[2];
        return angleDeg(c, p.c);
    };
    std::vector<char> use(pairs_.size(), 1);
    float R[9];
    fit(use, R);
    // One outlier pass: a few frames with a wrong floor fit (or a step's bounce) dominate the
    // RMS (recorded walk: median 3.0 deg, RMS 6.2 deg). Drop errors above max(3 deg, 2.5 x
    // median) and refit, but only if most pairs stay - a phone that is not rigid has large
    // errors everywhere and must still be rejected.
    {
        std::vector<float> e(pairs_.size());
        for (size_t k = 0; k < pairs_.size(); ++k) e[k] = errorDeg(R, pairs_[k]);
        std::vector<float> sorted = e;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        const float limit = std::max(3.0f, 2.5f * sorted[sorted.size() / 2]);
        size_t kept = 0;
        for (size_t k = 0; k < pairs_.size(); ++k) kept += (use[k] = e[k] <= limit);
        inlierFrac_ = static_cast<float>(kept) / pairs_.size();
        if (kept >= minSamples && inlierFrac_ >= minInlierFrac) fit(use, R);
        else std::fill(use.begin(), use.end(), 1);
    }

    double se = 0;
    size_t n = 0;
    for (size_t k = 0; k < pairs_.size(); ++k) {
        if (!use[k]) continue;
        const float e = errorDeg(R, pairs_[k]);
        se += e * e;
        ++n;
    }
    residualDeg_ = static_cast<float>(std::sqrt(se / std::max<size_t>(1, n)));
    for (int i = 0; i < 9; ++i) R_[i] = R[i];
    ok_ = spreadDeg_ >= minSpreadDeg && residualDeg_ <= maxResidualDeg && inlierFrac_ >= minInlierFrac;
}

bool GravityCalib::toCamera(const float phone[3], float cam[3]) const {
    if (!ok_) return false;
    for (int i = 0; i < 3; ++i)
        cam[i] = R_[i * 3] * phone[0] + R_[i * 3 + 1] * phone[1] + R_[i * 3 + 2] * phone[2];
    return true;
}
