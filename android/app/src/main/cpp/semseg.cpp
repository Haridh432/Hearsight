#include "semseg.h"

#include <android/log.h>

#include <chrono>
#include <cmath>
#include <cstring>

#define LOG_TAG "HSV2Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {

const char* kNames[kNavCount] = {
    "obstacle", "floor", "wall", "door", "stairs", "ceiling", "person", "vehicle",
    "road", "sidewalk", "terrain", "water", "sky", "vegetation", "building"};

const uint8_t kColors[kNavCount][3] = {
    {255, 140, 0},    // obstacle  orange
    {0, 200, 80},     // floor     green
    {120, 120, 180},  // wall      slate
    {255, 220, 0},    // door      yellow
    {255, 0, 0},      // stairs    red
    {200, 200, 200},  // ceiling   light grey
    {160, 80, 255},   // person    purple
    {0, 90, 255},     // vehicle   blue
    {90, 90, 90},     // road      dark grey
    {0, 200, 200},    // sidewalk  cyan
    {140, 170, 50},   // terrain   olive
    {0, 0, 160},      // water     navy
    {135, 206, 235},  // sky       sky blue
    {30, 120, 60},    // vegetation dark green
    {110, 60, 200},   // building  violet
};

// ADE20K class index (0-based, 150 classes) -> NavClass. Unlisted = obstacle.
struct AdeMap {
    uint8_t lut[256];
    AdeMap() {
        memset(lut, kNavObstacle, sizeof(lut));
        auto set = [&](std::initializer_list<int> ids, NavClass c) {
            for (int i : ids) lut[i] = c;
        };
        set({3, 28, 54}, kNavFloor);                                // floor, rug, runway
        set({0, 8, 18, 22, 27, 32, 42, 63, 100, 144}, kNavWall);   // wall, window, curtain, painting, mirror, fence, column, blind, poster, bulletin board
        set({14, 58}, kNavDoor);                                    // door, screen door
        set({53, 59, 96, 121}, kNavStairs);                         // stairs, stairway, escalator, step
        set({5}, kNavCeiling);
        set({12}, kNavPerson);
        set({20, 76, 80, 83, 90, 102, 103, 116, 127}, kNavVehicle); // car, boat, bus, truck, airplane, van, ship, minibike, bicycle
        set({6, 91}, kNavRoad);                                     // road, dirt track
        set({11, 52}, kNavSidewalk);                                // sidewalk, path
        set({9, 13, 29, 46, 94}, kNavTerrain);                      // grass, earth, field, sand, land
        set({21, 26, 60, 109, 113, 128}, kNavWater);                // water, sea, river, pool, waterfall, lake
        set({2}, kNavSky);
        set({4, 17, 66, 72}, kNavVegetation);                       // tree, plant, flower, palm
        set({1, 25, 48, 79, 84}, kNavBuilding);                     // building, house, skyscraper, hovel, tower
    }
};
const AdeMap kAde;

}  // namespace

const char* navClassName(int c) { return c >= 0 && c < kNavCount ? kNames[c] : "?"; }
const uint8_t* navClassColor(int c) { return kColors[c >= 0 && c < kNavCount ? c : 0]; }

void drawSegmentation(uint8_t* rgba, int width, int height, int stride, const SegMap& map,
                      int alpha) {
    if (map.width <= 0 || map.height <= 0) return;
    std::vector<int> colIdx(width);
    for (int x = 0; x < width; ++x) colIdx[x] = (x * map.width) / width;
    for (int y = 0; y < height; ++y) {
        const uint8_t* row = map.cls.data() + static_cast<size_t>((y * map.height) / height) * map.width;
        uint8_t* p = rgba + static_cast<size_t>(y) * stride * 4;
        for (int x = 0; x < width; ++x, p += 4) {
            const uint8_t* c = kColors[row[colIdx[x]]];
            p[0] = static_cast<uint8_t>((p[0] * (256 - alpha) + c[0] * alpha) >> 8);
            p[1] = static_cast<uint8_t>((p[1] * (256 - alpha) + c[1] * alpha) >> 8);
            p[2] = static_cast<uint8_t>((p[2] * (256 - alpha) + c[2] * alpha) >> 8);
        }
    }
}

bool SemanticSegmenter::init(const std::string& contextPath, const std::string& libDir,
                             std::string& err) {
    stop();
    if (!rt_.initialize(contextPath, libDir)) {
        err = rt_.getLastError();
        return false;
    }
    const QnnTensorInfo& in = rt_.inputInfo();
    if (in.dims.size() != 4 || in.dims[3] != 3 || in.bytes != in.dims[1] * in.dims[2] * 3) {
        err = "unexpected input tensor (want NHWC uint8 x3)";
        return false;
    }
    inH_ = static_cast<int>(in.dims[1]);
    inW_ = static_cast<int>(in.dims[2]);
    if (rt_.outputInfos().size() != 1) {
        err = "expected one output (class map)";
        return false;
    }
    const QnnTensorInfo& out = rt_.outputInfos()[0];
    if (out.dims.size() != 3 || out.bytes != out.dims[1] * out.dims[2] * 4) {
        err = "unexpected output tensor (want int32 [1,H,W])";
        return false;
    }
    outH_ = static_cast<int>(out.dims[1]);
    outW_ = static_cast<int>(out.dims[2]);
    // real = (q + offset) * scale  ->  q = real / scale - offset
    for (int v = 0; v < 256; ++v) {
        const int q = static_cast<int>(std::lround(v / in.scale)) - in.offset;
        quant_[v] = static_cast<uint8_t>(q < 0 ? 0 : q > 255 ? 255 : q);
    }
    input_.assign(in.bytes, 0);
    quit_ = false;
    worker_ = std::thread(&SemanticSegmenter::run, this);
    ready_ = true;
    LOGI("SemSeg ready: input %dx%d (scale %.3f offset %d), output %dx%d",
         inW_, inH_, in.scale, in.offset, outW_, outH_);
    return true;
}

void SemanticSegmenter::stop() {
    ready_ = false;
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void SemanticSegmenter::submit(const uint8_t* rgba, int w, int h, double tSec) {
    if (!ready_) return;
    std::unique_lock<std::mutex> lk(m_);
    if (busy_ || pending_) return;   // worker still on an older frame
    // Nearest-neighbour resize to the model input (640x480 -> 384x288).
    uint8_t* dst = input_.data();
    for (int y = 0; y < inH_; ++y) {
        const uint8_t* row = rgba + static_cast<size_t>((y * h) / inH_) * w * 4;
        for (int x = 0; x < inW_; ++x) {
            const uint8_t* p = row + ((x * w) / inW_) * 4;
            *dst++ = quant_[p[0]];
            *dst++ = quant_[p[1]];
            *dst++ = quant_[p[2]];
        }
    }
    inputTime_ = tSec;
    pending_ = true;
    lk.unlock();
    cv_.notify_one();
}

bool SemanticSegmenter::latest(SegMap& out) const {
    std::lock_guard<std::mutex> lk(m_);
    if (result_.seq == 0) return false;
    if (out.seq != result_.seq) out = result_;
    return true;
}

void SemanticSegmenter::run() {
    std::vector<std::vector<uint8_t>> outputs;
    std::vector<uint8_t> inputCopy;
    SegMap map;
    map.width = outW_;
    map.height = outH_;
    map.cls.resize(static_cast<size_t>(outW_) * outH_);
    int count = 0;
    double npuSum = 0;
    auto statsStart = std::chrono::steady_clock::now();

    for (;;) {
        double t;
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return quit_ || pending_; });
            if (quit_) break;
            inputCopy = input_;
            t = inputTime_;
            pending_ = false;
            busy_ = true;
        }
        const bool ok = rt_.execute(inputCopy.data(), inputCopy.size(), outputs);
        if (ok) {
            const int32_t* c = reinterpret_cast<const int32_t*>(outputs[0].data());
            for (size_t i = 0; i < map.cls.size(); ++i) {
                const int32_t a = c[i];
                map.cls[i] = (a >= 0 && a < 150) ? kAde.lut[a] : kNavObstacle;
            }
            map.frameTimeSec = t;
            npuSum += rt_.lastExecuteMs();
            ++count;
        } else {
            LOGE("SemSeg inference failed: %s", rt_.getLastError().c_str());
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            if (ok) {
                map.seq = result_.seq + 1;
                result_ = map;
            }
            busy_ = false;
        }
        const auto now = std::chrono::steady_clock::now();
        const double el = std::chrono::duration<double>(now - statsStart).count();
        if (el >= 1.0) {
            hz_ = count / el;
            npuMs_ = count ? npuSum / count : 0.0;
            count = 0;
            npuSum = 0;
            statsStart = now;
        }
    }
    rt_.shutdown();
}
