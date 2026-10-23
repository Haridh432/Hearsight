#ifndef HSV2_TRACKER_H
#define HSV2_TRACKER_H

#include <vector>

#include "yolo_detector.h"

/*
 * ByteTrack-style multi-object tracker with a constant-velocity Kalman
 * filter on each object's 3D position (camera frame: X right, Z forward,
 * metres). Gives every detection a stable track ID, a velocity and a
 * time-to-collision (TTC) estimate.
 *
 * Association: confident detections (>= highThreshold) are matched to the
 * predicted track boxes first by IoU; the remaining low-confidence ones are
 * then matched to the leftover tracks (recovers occluded / blurred
 * objects); unmatched confident detections start new tracks.
 */
struct TrackInfo {
    int id{-1};
    bool confirmed{false};
    float x{0}, z{0};        // metres, camera frame
    float vx{0}, vz{0};      // m/s
    float speed{0};          // |v|
    bool approaching{false};
    float ttc{-1};           // seconds; -1 if not closing
    float lateralAtImpact{0};  // predicted X when it reaches the user (m)
};

enum class MotionAlert { None = 0, Caution = 1, Stop = 2 };

class ObjectTracker {
public:
    // Camera intrinsics of the colour stream (pixels).
    void setIntrinsics(float fx, float cx, float fy = 0, float cy = 0) {
        fx_ = fx; cx_ = cx;
        if (fy > 0) { fy_ = fy; cy_ = cy; }
    }

    // User's own motion from visual odometry for this frame: camera pose in
    // the world (p_w = R p_cam + t, R row-major) and camera velocity in
    // camera axes. With it, objects are tracked in the world frame, so a still
    // object has ~zero velocity while the user walks toward it. Call every
    // frame before update(); pass valid=false when odometry has no estimate.
    void setEgo(bool valid, const float R[9], const float t[3], const float velCam[3]);

    // Updates tracks with this frame's detections (distances already set)
    // at time [tSec]; fills [info] (one entry per detection, same order).
    void update(const std::vector<Detection>& dets, double tSec,
                std::vector<TrackInfo>& info);

    // Most severe alert from confirmed tracks in the last update.
    MotionAlert alert() const { return alert_; }
    float alertTtc() const { return alertTtc_; }
    int alertTrack() const { return alertTrack_; }

    // Classes that move on their own (person, vehicles, animals).
    static bool isMover(int cocoClass);

    void reset() { tracks_.clear(); nextId_ = 1; alert_ = MotionAlert::None; }

    // Tunables.
    float highThreshold{0.5f};
    float matchIou{0.3f};
    int confirmHits{3};
    double maxLostSec{1.0};
    float stopTtc{2.0f};
    float cautionTtc{4.0f};
    float corridorHalfWidth{0.6f};  // metres either side of the user
    float minClosingSpeed{0.25f};   // m/s; slower is treated as not approaching
    float minOwnSpeed{0.3f};        // with ego-motion: the object itself must move

private:
    // 1D constant-velocity Kalman filter (position, velocity).
    struct Kf {
        float p{0}, v{0};
        float P00{1}, P01{0}, P11{1};
        bool init{false};
        void predict(float dt, float q);
        void correct(float meas, float r);
    };
    struct Track {
        int id;
        int classId;
        float bx, by, bw, bh;    // last box (pixels)
        float bvx{0}, bvy{0};    // box centre velocity (px/s), smoothed
        Kf kx, ky, kz;
        bool world{false};       // filters hold world-frame positions
        int hits{0};
        double lastSeen{0};
        double lastUpdate{0};
    };

    static float iou(float ax, float ay, float aw, float ah,
                     float bx, float by, float bw, float bh);

    std::vector<Track> tracks_;
    int nextId_{1};
    float fx_{600.0f}, cx_{320.0f}, fy_{600.0f}, cy_{240.0f};
    bool ego_{false};
    float egoR_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1}, egoT_[3]{0, 0, 0}, egoV_[3]{0, 0, 0};
    MotionAlert alert_{MotionAlert::None};
    float alertTtc_{-1};
    int alertTrack_{-1};
};

#endif  // HSV2_TRACKER_H
