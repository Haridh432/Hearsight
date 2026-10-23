#ifndef HSV2_LOCAL_MAP_H
#define HSV2_LOCAL_MAP_H

#include <cstddef>
#include <cstdint>
#include <vector>

/*
 * Phase 4: dynamic local map around the user (top-down, gravity-aligned).
 *
 * A rolling grid (default 160 x 160 cells of 10 cm = 16 x 16 m) centred on
 * the user, in the odometry world frame projected onto the floor plane.
 *
 * Layers per cell:
 *  1. Occupancy: log-odds of an obstacle at body/head height (hits from
 *     depth points 0.10-2.0 m above the floor; floor points are evidence of
 *     free space).
 *  2. Hazards: drop-off (floor falling away) and unknown-floor evidence.
 *  3. Semantics: latest confident navigation class (wall / door / stairs /
 *     floor ...) from the segmentation.
 *  4. Movers: people / vehicles from the tracker, kept as a list with
 *     velocity; their pixels are NOT written into layers 1-3.
 *
 * "Dynamic": evidence decays toward unknown when not re-observed
 * (half-life decayHalfLifeSec), and a cell seen as free floor clears an
 * obstacle that has moved away.
 */
struct MapPose {
    float R[9];   // camera -> world rotation (row-major)
    float t[3];   // camera position in the world
};

struct MapInput {
    // Depth image (Z16) and intrinsics (depth camera; the floor plane and
    // pose below must be in the same camera frame).
    const uint16_t* depth{nullptr};
    int width{0}, height{0};
    float fx{0}, fy{0}, cx{0}, cy{0};
    float units{0.001f};
    int step{4};
    // Floor plane in camera axes: n . p + d = 0, n = up (unit), d = height.
    float floorN[3]{0, -1, 0};
    float floorD{0};
    MapPose pose{};
    double tSec{0};
    // Optional per-pixel exclusion (movers): skip depth pixel (u, v) if
    // exclude(ctx, u, v) returns true.
    bool (*exclude)(void* ctx, int u, int v){nullptr};
    void* excludeCtx{nullptr};
    // Optional semantic class per depth pixel (NavClass, 255 = unknown).
    uint8_t (*semantic)(void* ctx, int u, int v){nullptr};
    void* semanticCtx{nullptr};
};

struct MapMover {
    int id;
    float x, y;     // map plane coordinates (m)
    float vx, vy;   // m/s
    int classId;
};

class LocalMap {
public:
    explicit LocalMap(int cells = 160, float cellM = 0.10f);

    void reset();
    // Integrates one frame. Returns false (nothing done) without a floor/pose.
    // update=false: only follows the user's pose and refreshes the live point
    // cloud (for the 3D view) without changing the map; in.floorD <= 0 means
    // no floor this frame (heights then come from the map's up axis and the
    // last camera height).
    bool integrate(const MapInput& in, bool update = true);
    bool initialized() const { return haveUp_; }
    void setMovers(const std::vector<MapMover>& m) { movers_ = m; }
    // Walking direction for the "ahead" queries (map plane, unit). When the
    // user walks (odometry) this is better than where the head points.
    void setWalkDirection(bool valid, float x, float y) { walkValid_ = valid; wx_ = x; wy_ = y; }

    // --- Queries (map-plane metres) ---
    // User position and heading (unit, camera forward on the floor plane).
    void user(float& x, float& y, float& hx, float& hy) const { x = ux_; y = uy_; hx = hx_; hy = hy_; }
    // Nearest occupied cell (P > minProb) inside a corridor of [halfWidth]
    // along the user's heading, up to [maxDist]; -1 if none. Unlike the live
    // depth check this includes what is currently out of view.
    float obstacleAhead(float halfWidth, float maxDist, float minProb = 0.7f, float minDist = 0.2f) const;
    float hazardAhead(float halfWidth, float maxDist, float minDist = 0.2f) const;
    // Projects world point [w] to raw plane coordinates (no drift correction).
    void toPlane(const float w[3], float& x, float& y) const;
    // World point / direction -> map coordinates (with drift correction).
    void toMapPoint(const float w[3], float& x, float& y) const;
    void toMapDir(const float v[3], float& x, float& y) const;

    // Cell access (for drawing): row-major cells x cells, centred on centre().
    int cells() const { return n_; }
    float cellSize() const { return cell_; }
    void origin(float& x0, float& y0) const { x0 = ox_ * cell_; y0 = oy_ * cell_; }
    float occupancy(int i, int j) const;           // probability 0..1 (0.5 unknown)
    bool known(int i, int j) const;
    uint8_t hazard(int i, int j) const;            // bit 0 drop-off
    uint8_t semantic(int i, int j) const;          // NavClass or 255
    const uint8_t* semHist(int i, int j) const { return grid_[static_cast<size_t>(j) * n_ + i].semHist; }
    float logOdds(int i, int j) const { return grid_[static_cast<size_t>(j) * n_ + i].lo; }
    const std::vector<MapMover>& movers() const { return movers_; }
    // Height layer: highest obstacle point seen in the cell (m above floor).
    float obstacleHeight(int i, int j) const { return grid_[static_cast<size_t>(j) * n_ + i].maxH; }
    float obstacleBottom(int i, int j) const { return grid_[static_cast<size_t>(j) * n_ + i].minH; }

    // Trajectory: user poses (map plane) every ~0.3 m or 15 deg, last 200.
    struct TrailPose { float x, y, hx, hy; };
    const std::vector<TrailPose>& trail() const { return trail_; }
    // Latest frame's points (map plane x, y, height above floor, NavClass or
    // 255), subsampled; for the 3D view.
    struct CloudPt { float x, y, h; uint8_t sem; };
    const std::vector<CloudPt>& cloud() const { return cloud_; }

    // Draws a top-down view (north = heading up) into an RGBA8 image region.
    void draw(uint8_t* rgba, int stride, int x0, int y0, int size) const;

    // Drift correction (map alignment): each frame's obstacle points are
    // matched against the occupancy map over small shifts / turns, and the
    // best correction is accumulated and applied to the odometry pose.
    bool alignEnabled{true};
    float alignStep{0.05f}, alignRange{0.10f};   // metres
    float alignYawStepDeg{0.5f}, alignYawRangeDeg{2.0f};
    size_t alignMaxPoints{300};
    void correction(float& dx, float& dy, float& yawDeg) const;

    // Tunables.
    float bodyMinH{0.10f}, headMaxH{2.0f};
    float floorTol{0.05f}, dropBelow{0.12f};
    // Glossy-floor reflections: a point more than reflectBelow under the floor
    // that segmentation labels wall / ceiling / other (not floor, stairs or
    // obstacle) is a mirror image, not a drop (see SafetyLayer::reflectMinDepth).
    float reflectBelow{0.5f};
    // Drop-off evidence only within dropRange, with a threshold growing with
    // distance (a 1-2 deg floor-tilt error puts far floor "below" the plane).
    float dropRange{3.0f}, dropPerMetre{0.02f};
    float maxRange{4.5f};
    float hitLogOdds{0.85f}, freeLogOdds{-0.4f}, maxLogOdds{4.0f};
    double decayHalfLifeSec{30.0};
    // Ray clearing: every rayStride-th depth ray marks the cells it passes
    // through (at a height inside their obstacle range) as evidence that the
    // obstacle is gone; >= rayClearVotes votes in a frame without hits clear.
    bool rayClear{true};
    int rayStride{4};
    int rayClearVotes{3};
    float heightTrack{0.3f};   // cell height follows observations at this rate

private:
    struct Cell {
        float lo{0};          // occupancy log-odds (0 = unknown)
        float drop{0};        // drop-off evidence
        uint8_t sem{255};
        uint8_t semVotes{0};
        // Class tally from obstacle points (bins: object, wall, door, stairs,
        // person, vehicle, vegetation, other), slowly fading.
        uint8_t semHist[8]{0, 0, 0, 0, 0, 0, 0, 0};
        float seen{-1e9f};    // last observation time (s)
        float maxH{0};        // obstacle top (m above floor): 2nd-highest point, tracked
        float minH{0};        // obstacle bottom: > ~0.3 m = overhang with free space under it
    };
    int index(int ci, int cj) const;   // world cell -> storage index, -1 outside
    void recentre(int cx, int cy);
    void decay(double tSec);

    int n_;
    float cell_;
    std::vector<Cell> grid_;
    int ox_{0}, oy_{0};          // world cell index of grid (0,0)
    bool haveUp_{false};
    float up_[3]{0, 1, 0}, e1_[3]{1, 0, 0}, e2_[3]{0, 0, 1};
    float ux_{0}, uy_{0}, hx_{0}, hy_{1};
    double lastDecay_{-1};
    // Accumulated plane correction: p_corr = Rc * p_raw + tc.
    float corrC_{1}, corrS_{0}, corrTx_{0}, corrTy_{0};
    float alignScore(const std::vector<float>& pts, float c, float s, float tx, float ty) const;
    std::vector<MapMover> movers_;
    std::vector<TrailPose> trail_;
    std::vector<CloudPt> cloud_;
    float lastCamH_{1.5f};
    bool walkValid_{false};
    float wx_{0}, wy_{1};
    void aheadDir(float& x, float& y) const { if (walkValid_) { x = wx_; y = wy_; } else { x = hx_; y = hy_; } }
};

#endif  // HSV2_LOCAL_MAP_H
