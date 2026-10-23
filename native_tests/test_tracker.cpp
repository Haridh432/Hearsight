// Host test: synthetic people at 30 fps with stereo-like depth noise.
#include "tracker.h"
#include <cmath>
#include <cstdio>
#include <random>
const float fx = 600, cx = 320;
Detection makeDet(float X, float Z, std::mt19937& rng) {
    std::normal_distribution<float> n(0, 0.01f * Z * Z);   // depth noise grows with Z^2
    Detection d; d.classId = 0; d.confidence = 0.8f;
    float u = cx + X * fx / Z; float h = 1.7f * fx / Z, w = 0.5f * fx / Z;
    d.x = u - w / 2; d.y = 240 - h / 2; d.w = w; d.h = h; d.distanceM = Z + n(rng);
    return d;
}
void run(const char* name, float X0, float Z0, float vx, float vz, float secs) {
    ObjectTracker tr; tr.setIntrinsics(fx, cx); std::mt19937 rng(1);
    std::vector<TrackInfo> info; float firstStop = -1;
    for (int f = 0; f <= secs * 30; ++f) {
        float t = f / 30.0f, X = X0 + vx * t, Z = Z0 + vz * t;
        std::vector<Detection> d{makeDet(X, Z, rng)}; tr.update(d, t, info);
        if (tr.alert() == MotionAlert::Stop && firstStop < 0) firstStop = t;
        if (f % 30 == 0 || f == (int)(secs * 30)) {
            const TrackInfo& i = info[0];
            float trueTtc = vz < 0 ? Z / -vz : -1;
            printf("%-9s t=%.1fs Z=%.2f | id %d est v=(%.2f,%.2f) m/s ttc %.2f (true %.2f) alert %d\n", name, t, Z,
                   i.id, i.vx, i.vz, i.ttc, trueTtc, (int)tr.alert());
        }
    }
    printf("%-9s first STOP at t=%.2fs\n\n", name, firstStop);
}
// The user walks forward at [vu] m/s (camera z), the object moves at
// (vx, vz) in the world. Detections are made in the camera frame. With
// [ego] the tracker gets the user's pose/velocity (as from odometry).
// expect: 0 = no alert at all, 1 = STOP with correct TTC, 2 = no STOP (CAUTION allowed)
int runEgo(const char* name, float X0, float Z0, float vx, float vz, float vu, bool ego, float secs, int expect) {
    ObjectTracker tr; tr.setIntrinsics(fx, cx, fx, 240); std::mt19937 rng(2);
    std::vector<TrackInfo> info; float firstStop = -1, ttcAt2 = -1; int alertFrames = 0;
    for (int f = 0; f <= secs * 30; ++f) {
        float t = f / 30.0f, uz = vu * t;
        float Xw = X0 + vx * t, Zw = Z0 + vz * t;
        float Zc = Zw - uz;   // camera at (0,0,uz), looking along +z
        if (Zc < 0.6f) break;
        const float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, T[3] = {0, 0, uz}, V[3] = {0, 0, vu};
        tr.setEgo(ego, R, T, V);
        std::vector<Detection> d{makeDet(Xw, Zc, rng)}; tr.update(d, t, info);
        if (tr.alert() != MotionAlert::None) ++alertFrames;
        if (tr.alert() == MotionAlert::Stop && firstStop < 0) firstStop = t;
        if (ttcAt2 < 0 && Zc < 2.0f && info[0].ttc > 0) ttcAt2 = info[0].ttc;
    }
    const float closing = vu - vz;
    const bool pass = expect == 1 ? (firstStop > 0 && ttcAt2 > 0 && std::fabs(ttcAt2 - 2.0f / closing) < 0.25f)
                    : expect == 2 ? firstStop < 0 : alertFrames == 0;
    printf("%-44s: alert frames %3d, first STOP %.2fs, TTC at 2 m %.2f s (true %.2f)  %s\n", name, alertFrames,
           firstStop, ttcAt2, closing > 0 ? 2.0f / closing : -1.0f, pass ? "OK" : "FAIL");
    return pass ? 0 : 1;
}

int main() {
    int fail = 0;
    fail += runEgo("user walks 1.2 m/s to a standing person, ego", 0.1f, 6.0f, 0, 0, 1.2f, true, 4.5f, 0);
    fail += runEgo("same, no ego (old behaviour: false alert)", 0.1f, 6.0f, 0, 0, 1.2f, false, 4.5f, 1);
    fail += runEgo("user 1.0 m/s, person walks at user 1.3 m/s, ego", 0.1f, 8.0f, 0, -1.3f, 1.0f, true, 3.5f, 1);
    fail += runEgo("user 1.0 m/s, person crossing 1.3 m/s, ego (no STOP)", -3.0f, 5.0f, 1.3f, 0, 1.0f, true, 3.0f, 2);
    printf("ego tracker failures: %d\n\n", fail);
    run("approach", 0.1f, 6.0f, 0.0f, -1.3f, 3.5f);   // walking at the user, 1.3 m/s
    run("crossing", -3.0f, 3.0f, 1.3f, 0.0f, 4.0f);   // crossing left->right 3 m ahead
    run("standing", 0.3f, 2.5f, 0.0f, 0.0f, 3.0f);    // standing still
    return fail;
}
