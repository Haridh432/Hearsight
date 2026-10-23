#ifndef HSV2_SAFETY_H
#define HSV2_SAFETY_H

#include <cstdint>
#include <random>
#include <vector>

/*
 * Layer 1 reactive safety: works from the live depth frame only (plus an
 * optional gravity vector), independent of the map, tracking and the model.
 *
 * Depth camera frame: x right, y down, z forward (metres).
 *  1. Sample the depth image on a grid and back-project the points.
 *  2. Fit the floor plane with RANSAC (normal near "up": gravity if given,
 *     otherwise the camera's -y axis).
 *  3. Height of every point above the floor.
 *  4. Walking corridor on the floor straight ahead of the camera.
 *  5. Obstacles in the corridor at body height and head height.
 *  6. Drop-offs (floor falling away) and unknown floor ahead.
 *  7. STOP / CAUTION with frame confirmation and a short hold.
 */
enum class SafetyState { Clear = 0, Caution = 1, Stop = 2 };
enum class SafetyReason { None = 0, Obstacle, HeadHeight, DropOff, UnknownFloor, NoFloor, Stairs, TripHazard };

// Per-cell semantic role from the scene segmentation (optional input).
enum SemRole : uint8_t { kSemOther = 0, kSemWalkable, kSemStairs, kSemObstacle };

// Semantic label grid in colour-image coordinates plus the depth->colour
// projection used to look up the role of each depth point.
struct SemanticInput {
    const uint8_t* roles{nullptr};   // SemRole per cell, row-major
    int gridW{0}, gridH{0};
    float rot[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};  // depth->colour, column-major (rs2_extrinsics)
    float trans[3]{0, 0, 0};
    float fx{0}, fy{0}, ppx{0}, ppy{0};       // colour intrinsics
    int colorW{0}, colorH{0};
};

struct SafetyResult {
    SafetyState state{SafetyState::Clear};
    SafetyReason reason{SafetyReason::None};
    float distance{-1};        // metres along the corridor to the hazard

    // Diagnostics.
    bool floorFound{false};
    float floorNormal[3]{0, 0, 0};   // "up" in depth camera axes (when floorFound)
    bool gravityUsed{false};
    // Floor found continuously for floorStableSec and at the learned eye
    // height: safe to accumulate into a map (a wrong fit makes the real
    // floor look like a drop).
    bool floorStable{false};
    float cameraHeight{0};     // metres above the floor
    float cameraPitchDeg{0};   // + = looking down
    float obstacleDist{-1};
    float headDist{-1};
    float dropDist{-1};
    float unknownDist{-1};
    float stairsDist{-1};
    float tripDist{-1};        // 3-10 cm object (cable, threshold, shoe) ahead
    float closingSpeed{0};     // m/s toward the nearest obstacle (smoothed)
    float walkSpeed{-1};       // user's speed along the floor ahead (odometry), -1 if none
    float mapObstacle{-1};     // remembered obstacle ahead (map), -1 if none
    float mapHazard{-1};
    float stopDistUsed{0};     // effective STOP distance this frame
    float turnRateDegS{0};     // floor-normal rotation rate (head turn)
    float lowObstacleDist{-1};
    bool semanticUsed{false};
    int points{0};
    double ms{0};
};

struct DepthIntrinsics {
    float fx, fy, ppx, ppy;
    int width, height;
};

class SafetyLayer {
public:
    // [depth] Z16, [units] metres per unit. [tSec] frame time.
    // [sem] optional scene segmentation (nullptr = geometry only).
    SafetyResult process(const uint16_t* depth, const DepthIntrinsics& in,
                         float units, double tSec, const SemanticInput* sem = nullptr);

    // Gravity "down" direction in the depth camera frame (unit vector).
    // Optional: improves the floor fit when an IMU is available.
    void setGravity(float gx, float gy, float gz);
    void clearGravity() { haveGravity_ = false; }

    // Remembered obstacle / hazard ahead from the dynamic map (Phase 4),
    // metres along the walking direction, -1 = none. Includes what is out of
    // view; used for CAUTION only (odometry drift must not cause a STOP).
    void setMapAhead(float obstacle, float hazard) { mapObstacle_ = obstacle; mapHazard_ = hazard; }

    // User's own velocity (camera axes, m/s) from visual odometry, or
    // valid=false. Its component along the walking direction sets the
    // speed-dependent STOP distance (with the obstacle-distance slope).
    void setEgoVelocity(bool valid, const float velCam[3]) {
        haveEgo_ = valid;
        if (valid) { egoV_[0] = velCam[0]; egoV_[1] = velCam[1]; egoV_[2] = velCam[2]; }
    }

    void reset();

    // ---- Tunables (metres / seconds) ----
    int sampleStep{4};
    float minRange{0.3f}, maxRange{4.5f};
    float corridorHalfWidth{0.45f};
    float corridorNear{0.3f}, corridorFar{3.0f};
    float bodyMinH{0.10f}, bodyMaxH{1.40f}, headMaxH{2.00f};
    // Head band top follows the user: learned eye height + this (the top of
    // the head is ~15 cm above the eyes); a door lintel at ~2.0 m is no
    // obstacle for a 1.6 m eye height.
    float headAboveEye{0.30f};
    float stopDist{1.2f}, cautionDist{2.5f};
    float dropDepth{0.12f};               // below floor = drop
    // Drop-offs are measured against the LOCAL floor, tracked bin by bin along
    // the corridor (a floor fit a few degrees off makes the far floor look
    // 0.1-0.3 m low; a real step is a sudden jump between bins).
    float floorTrackBand{0.08f};
    // The track starts at the first bin with floor points within this of the fitted
    // plane: wide enough for a briefly stale fit while nodding (gravity test 1c, 1.5 deg
    // bias: +-5 cm never started and lost the step at 1.47 m) and for a glossy floor's
    // first points at -0.14 m.
    float floorTrackStartBand{0.15f};
    // Glossy floors mirror walls / pillars / ceiling: the depth camera sees them
    // far below the floor, and segmentation labels them wall etc. With
    // segmentation, a point more than reflectMinDepth below the local floor
    // labelled neither floor, stairs nor obstacle is a reflection, not a drop.
    // (A real lower floor or stairs keep counting; without segmentation every
    // point below the floor counts.)
    float reflectMinDepth{0.5f};
    float dropStopDist{1.5f}, dropCautionDist{2.5f};
    float unknownCautionDist{1.2f};
    float floorInlier{0.04f};
    float maxFloorTiltDeg{35.0f};         // allowed normal deviation from "up"
    // With gravity, the floor normal must stay within this of -gravity on
    // every frame (instead of following the previous fit, which lags nods).
    float gravityTiltDeg{2.0f};
    // Fall back to the tracked (non-gravity) fit only after the tight fit has
    // failed this long (a real IMU / mount bias, not a brief miss at a step).
    double gravityFallbackSec{0.5};
    int ransacIters{120};
    // Floor continuity: a tracked floor may change height by at most this
    // much per frame, so a lower level ahead is a drop, not a new floor.
    float floorTrackTolerance{0.04f};
    float floorTrackTiltDeg{6.0f};
    // Learned camera height above the floor (eye height) and tolerance. On
    // flat ground the height only changes by head bob (a few cm); a head
    // pitch rotates the plane but keeps the height, while a wrong fit that
    // mixes two levels changes it.
    float heightTolerance{0.05f};
    // Plausible eye height of a head-mounted camera on a standing person:
    // rejects desks / tables (~0.75 m below the eye) as the floor.
    float eyeHeightMin{1.1f}, eyeHeightMax{2.1f};
    // No floor for this long -> forget the learned eye height and re-learn.
    double relearnSec{3.0};
    // Semantics: stairs ahead raise CAUTION within this range; objects
    // labelled as obstacles lower than bodyMinH count from lowObstacleMinH.
    float stairsCautionDist{3.0f};
    float stairsMinShare{0.30f};   // of the corridor points in a 10 cm bin
    float lowObstacleMinH{0.04f};
    int minWalkableForFit{150};
    double holdSec{0.5};
    double floorStableSec{0.5};
    float floorStableTol{0.03f};
    float floorStableEyeTol{0.08f};   // map learns only within this of the learned eye height
    // A STOP obstacle that leaves the camera view (head raised) is held this
    // long, unless the place where it was is visible again and empty.
    double outOfViewHoldSec{2.0};
    // Speed-dependent stopping distance: stop = max(stopDist, margin +
    // closing speed * reaction time); caution starts cautionExtra further.
    float stopMargin{0.5f}, reactionSec{0.8f}, cautionExtra{1.3f};
    // Drop-offs: the floor fit must match the learned eye height (a wrong
    // fit makes the real floor look lower), and a drop must be seen in
    // dropConfirm of the last 6 frames.
    int dropConfirm{4};
    // An obstacle missed for a frame or two (point count at the threshold)
    // is kept this long; closing speed is fitted over speedWindowSec.
    double persistSec{0.3};
    // Trip hazards: objects tripMinH..bodyMinH above the LOCAL floor (the
    // floor points of the same 10 cm bin, so floor-fit tilt cancels) in the
    // centre strip within tripDist. CAUTION only.
    float tripMinH{0.03f};
    float tripRange{2.0f};
    int tripMinPoints{6};
    // Drop-off evidence is ignored while the head turns faster than this
    // (the floor fit lags a fast nod and the floor ahead looks lower).
    float maxTurnRateDegS{1e9f};   // disabled: fit jumps near a step read as fast turns and hid real drops
    // A drop-off nearer than nearDropDist must have been seen farther away
    // within nearDropSec (a real step is seen while approaching; a sudden
    // near "drop" is a fit error).
    float nearDropDist{0.8f};
    double nearDropSec{1.5};
    double speedWindowSec{1.0};
    double floorMemorySec{1.0};

private:
    struct P { float x, y, z; int u, v; uint8_t role; };
    struct Plane { float nx, ny, nz, d; bool valid{false}; };  // n·p + d = 0, n = up

    bool fitFloor(const std::vector<P>& pts, Plane& out, bool walkableOnly, bool useGravity = true);
    float minPointsAt(float along) const;

    std::vector<P> pts_;
    // Camera-height learning over the first stable frames.
    std::vector<float> heightSamples_;
    float learnedHeight_{-1};
    std::mt19937 rng_{1234};
    Plane floor_;
    double floorTime_{-1e9};
    bool haveGravity_{false};
    bool haveEgo_{false};
    float mapObstacle_{-1}, mapHazard_{-1};
    float egoV_[3]{0, 0, 0};
    float gx_{0}, gy_{1}, gz_{0};

    // Out-of-view memory of the last near obstacle.
    float memoDist_{-1};
    double memoTime_{-1e9};
    SafetyReason memoReason_{SafetyReason::None};
    int memoLevel_{0};
    // Previous floor normal, for the head turn rate.
    float prevN_[3]{0, 0, 0};
    double prevNTime_{-1e9};
    double lastFarDropTime_{-1e9};
    double gravityFitFailSince_{-1};   // first frame the gravity-pinned fit failed
    double floorSince_{-1};            // start of the current run of floor-found frames
    float floorRunD_{0};               // smoothed camera height over the run
    // Recent nearest-obstacle distances (time, metres) for the closing
    // speed (line fit over speedWindowSec, robust to missed frames).
    std::vector<std::pair<double, float>> nearHist_;
    float closing_{0};
    // Short obstacle persistence: last body / head detection.
    float lastObs_{-1}, lastHead_{-1};
    double lastObsTime_{-1e9}, lastHeadTime_{-1e9};
    // Drop-off confirmation history.
    int dropHist_[6]{0, 0, 0, 0, 0, 0};
    int dropIdx_{0};

    // Confirmation / hold.
    int history_[3]{0, 0, 0};
    int histIdx_{0};
    SafetyResult held_;
    double heldUntil_{-1e9};
};

#endif  // HSV2_SAFETY_H
