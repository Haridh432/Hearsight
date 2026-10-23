#ifndef HSV2_VO_H
#define HSV2_VO_H

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

/*
 * RGB-D visual odometry (Phase 3): the user's own motion from the colour
 * camera + depth, CPU only.
 *
 *  1. Grey image at half resolution, 3-level pyramid.
 *  2. Shi-Tomasi corners on a grid; pyramidal Lucas-Kanade tracking with a
 *     forward-backward check.
 *  3. Each tracked point gets a 3D position from depth (previous and current
 *     frame); a RANSAC rigid fit gives the camera motion. Points on moving
 *     objects (people) disagree with the dominant motion and are rejected.
 *
 * Camera frame: x right, y down, z forward (metres). The first frame defines
 * the world frame.
 */
struct VoCamera {
    float fx, fy, cx, cy;   // full-resolution colour intrinsics (pixels)
    int width, height;      // full resolution
};

// Depth (metres) at full-resolution colour pixel (u, v); <= 0 if unknown.
using VoDepthFn = float (*)(void* ctx, float u, float v);

struct VoResult {
    bool ok{false};          // motion estimated this frame
    int tracked{0};          // points tracked from the previous frame
    int withDepth{0};        // of which have depth in both frames
    int inliers{0};          // agreeing with the camera motion
    float vel[3]{0, 0, 0};   // camera velocity, current camera axes (m/s), smoothed
    float speed{0};          // |vel|
    float angRate{0};        // rotation rate (rad/s), smoothed
    double ms{0};
};

// Depth registered to the colour image at half resolution (z-buffer):
// every [step]-th depth pixel is back-projected, moved into the colour camera
// ([rotColMajor] / [trans] = depth->colour extrinsics, librealsense layout)
// and projected. out[(v/2)*(cw/2) + u/2] = nearest depth (m), 0 = none.
struct DepthToColor {
    float dfx, dfy, dcx, dcy; int dw, dh;   // depth intrinsics
    float cfx, cfy, ccx, ccy; int cw, ch;   // colour intrinsics
    float rot[9];                           // column-major
    float trans[3];
};
void registerDepthHalf(const uint16_t* depth, float units, const DepthToColor& c, int step,
                       std::vector<float>& out);
// Half-resolution grey (2x2 average of luma) of an RGBA8 image.
void rgbaToHalfGray(const uint8_t* rgba, int width, int height, std::vector<uint8_t>& out);

// Lookup with a 3x3 nearest-valid search; (u, v) full-res colour pixels.
float depthHalfAt(const std::vector<float>& map, int cw, int ch, float u, float v);

class VisualOdometry {
public:
    void setCamera(const VoCamera& cam);
    // [rgba]: full-resolution RGBA8, row stride width*4.
    VoResult process(const uint8_t* rgba, double tSec, VoDepthFn depth, void* ctx);
    // [gray]: half-resolution grey (see rgbaToHalfGray); what process() uses.
    VoResult processGray(const uint8_t* gray, double tSec, VoDepthFn depth, void* ctx);
    void reset();

    // Current camera pose in the world frame: p_world = R * p_cam + t.
    const float* poseR() const { return Rw_; }
    const float* poseT() const { return tw_; }

    // Tunables.
    int maxPoints{250};
    int gridCols{16}, gridRows{12};
    // Corner strength threshold: relative to the frame's strongest corner,
    // with an absolute floor (intensity 0..1, per-pixel gradients).
    float minEigenRel{0.01f};
    float minEigenAbs{2e-6f};
    float fbMaxErr{1.0f};        // forward-backward check (half-res pixels)
    int ransacIters{80};
    int minInliers{12};
    float velSmooth{0.5f};       // EMA weight of the new measurement
    // A frame step faster than this is a bad match, not motion (hall recording: a 0.9 m
    // jump in one frame from 13 inliers at start-up).
    float maxSpeed{3.0f};        // m/s
    float maxAngRate{6.0f};      // rad/s

private:
    struct Level { int w, h; std::vector<float> img; };
    struct Track { float x, y; float P[3]; bool hasP; };

    void buildPyramid(const uint8_t* gray, std::vector<Level>& pyr) const;
    bool trackPoint(const std::vector<Level>& from, const std::vector<Level>& to,
                    float x, float y, float& ox, float& oy) const;
    void detect(const std::vector<Level>& pyr);
    bool backProject(float hx, float hy, VoDepthFn depth, void* ctx, float P[3]) const;

    VoCamera cam_{};
    std::vector<Level> prev_, cur_;
    std::vector<Track> tracks_;
    bool havePrev_{false};
    double prevT_{0};
    float Rw_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    float tw_[3]{0, 0, 0};
    float vel_[3]{0, 0, 0};
    float angRate_{0};
    std::mt19937 rng_{42};
};

// Runs VisualOdometry on its own thread (latest frame only), so the camera
// loop is not slowed down. The loop submits each frame's colour + depth and
// reads the newest result (at most a frame or two old).
class Recorder;

// Phone IMU sample handed along with a frame (for recordings).
struct RecImu {
    bool valid{false}; float up[3]{0, 0, 0}; float gyro[3]{0, 0, 0};   // phone IMU
    double camTsMs{-1};   // depth frame timestamp (camera clock, ms) for the D455 samples; -1 = none
};

class VoThread {
public:
    // Optional: frames processed by the worker are also written here.
    void setRecorder(Recorder* rec) { rec_ = rec; }
    ~VoThread() { stop(); }
    void start(const VoCamera& cam, const DepthToColor& reg);
    void stop();
    bool running() const { return running_; }
    // Copies the frame; dropped if the worker is still busy.
    void submit(const uint8_t* rgba, const uint16_t* depth, float depthUnits, double tSec,
                const RecImu* imu = nullptr);
    // Newest result and the pose at that frame. False if none yet.
    bool latest(VoResult& r, float R[9], float t[3], double& tSec) const;
    void reset();

private:
    void loop();
    Recorder* rec_{nullptr};
    VisualOdometry vo_;
    DepthToColor reg_{};
    std::thread thread_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool running_{false}, quit_{false}, pending_{false}, busy_{false}, resetReq_{false};
    std::vector<uint8_t> rgba_;
    std::vector<uint16_t> depth_;
    float units_{0.001f};
    double t_{0};
    RecImu imu_{};
    VoResult res_;
    float R_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1}, T_[3]{0, 0, 0};
    double resT_{-1};
};

#endif  // HSV2_VO_H
