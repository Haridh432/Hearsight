#ifndef HSV2_GRAVITY_CALIB_H
#define HSV2_GRAVITY_CALIB_H

#include <cstddef>
#include <vector>

/*
 * Online phone -> depth-camera rotation from gravity.
 *
 * The phone IMU gives "up" in phone axes; the safety layer's floor fit gives
 * "up" (the floor normal) in depth-camera axes. If the phone is rigidly
 * mounted with the camera, one rotation R maps the first onto the second for
 * every head pose. R is fitted to many pairs (Horn's quaternion method) and
 * accepted only when
 *   - the pairs span enough head orientations (else R is undetermined about
 *     the up axis), and
 *   - the residual is small (else the phone is not rigid with the camera,
 *     e.g. in a pocket, and its gravity must not be used).
 */
class GravityCalib {
public:
    // [phoneUp]: unit "up" in phone axes; [camUp]: unit floor normal (up) in
    // camera axes, both at the same frame time.
    void add(const float phoneUp[3], const float camUp[3], double tSec);

    // Rotates a phone-axes vector into camera axes. False if not calibrated.
    bool toCamera(const float phone[3], float cam[3]) const;

    bool ok() const { return ok_; }
    int samples() const { return static_cast<int>(pairs_.size()); }
    float spreadDeg() const { return spreadDeg_; }
    float residualDeg() const { return residualDeg_; }
    float inlierFrac() const { return inlierFrac_; }
    void reset();

    // Tunables.
    double minIntervalSec{0.2};   // sample at most 5 Hz
    size_t maxSamples{300};       // ~60 s window
    size_t minSamples{30};
    float minSpreadDeg{8.0f};     // orientation spread needed
    float maxResidualDeg{3.0f};   // RMS angle error to accept (after one outlier pass)
    float minInlierFrac{0.6f};    // share of pairs that must survive the outlier pass

private:
    struct Pair { float p[3], c[3]; };
    void solve();

    std::vector<Pair> pairs_;
    double lastAdd_{-1e9};
    int sinceSolve_{0};
    float R_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};   // row-major, cam = R * phone
    bool ok_{false};
    float spreadDeg_{0}, residualDeg_{0}, inlierFrac_{0};
};

#endif  // HSV2_GRAVITY_CALIB_H
