#ifndef HSV2_SEMSEG_H
#define HSV2_SEMSEG_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "qnn_runtime.h"

/*
 * Whole-scene semantic segmentation: DeepLabV3+ (MobileNetV2, ADE20K, 150
 * classes) on the HTP, run on its own worker thread at whatever rate the NPU
 * allows (~10-15 Hz) so the 30 fps detector loop is not blocked.
 *
 * The 150 ADE20K classes are merged into a few navigation classes.
 * Input: RGB uint8 NHWC (inW x inH). Output: int32 class map at 1/4 size.
 */
#include "nav_classes.h"

const char* navClassName(int c);
// RGB display colour of a navigation class.
const uint8_t* navClassColor(int c);

struct SegMap {
    int width{0}, height{0};
    std::vector<uint8_t> cls;   // NavClass per cell, row-major
    double frameTimeSec{0};     // timestamp of the input frame
    uint64_t seq{0};            // increments with every new result
};

// Blends the class colours over an RGBA8 image (any size; the map is scaled).
void drawSegmentation(uint8_t* rgba, int width, int height, int stride, const SegMap& map,
                      int alpha = 115 /* of 256 */);

class SemanticSegmenter {
public:
    ~SemanticSegmenter() { stop(); }

    bool init(const std::string& contextPath, const std::string& libDir, std::string& err);
    void stop();
    bool ready() const { return ready_; }

    // Offers an RGBA8 colour frame. Dropped if the worker is still busy
    // (latest-only), so this never blocks for inference.
    void submit(const uint8_t* rgba, int w, int h, double tSec);

    // Copies the newest result; false if none yet.
    bool latest(SegMap& out) const;

    // Stats over the last second.
    double hz() const { return hz_; }
    double npuMs() const { return npuMs_; }

private:
    void run();

    QnnRuntime rt_;
    int inW_{0}, inH_{0}, outW_{0}, outH_{0};
    uint8_t quant_[256];        // 0..255 pixel -> quantized input value

    std::thread worker_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool busy_{false}, pending_{false}, quit_{false};
    std::vector<uint8_t> input_;   // resized + quantized, NHWC
    double inputTime_{0};
    SegMap result_;
    std::atomic<bool> ready_{false};
    std::atomic<double> hz_{0}, npuMs_{0};
};

#endif  // HSV2_SEMSEG_H
