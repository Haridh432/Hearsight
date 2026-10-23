#ifndef YOLO_DETECTOR_H
#define YOLO_DETECTOR_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "qnn_runtime.h"

// One detection in camera-frame pixel coordinates.
struct Detection {
    int classId{-1};
    float confidence{0.0f};
    float x{0.0f};       // top-left
    float y{0.0f};
    float w{0.0f};
    float h{0.0f};
    float distanceM{0.0f};

    // Instance mask (segmentation models, persons only): a grid of
    // maskCellPx-sized camera-pixel cells starting at (maskX, maskY).
    bool hasMask{false};
    float maskX{0.0f}, maskY{0.0f}, maskCellPx{4.0f};
    int maskW{0}, maskH{0};
    std::vector<uint8_t> mask;   // maskW*maskH, 1 = object
};

/*
 * YOLOv8 post-processing for the split-output INT8 model:
 *   input  "images" uint8 NHWC [1,640,640,3], scale 1/255
 *   output "boxes"  uint8 [1,4,N]   (cx,cy,w,h in model pixels)
 *   output "scores" uint8 [1,80,N]  (sigmoid class scores)
 * Segmentation models additionally expose
 *   output "mask_coef" uint8 [1,32,N]          (per-anchor mask coefficients)
 *   output "protos"    uint8 [1,160,160,32]    (prototype masks, NHWC)
 * and get an instance mask for each person detection.
 * Frames are letterboxed into the top-left of the 640x640 input,
 * matching the calibration data.
 */
class YoloDetector {
public:
    // Looks up the "boxes"/"scores" outputs. Call after
    // QnnRuntime::initialize() succeeds.
    bool configure(const QnnRuntime& runtime, std::string& error);
    // Same, from tensor descriptions (also used by host-side tests).
    bool configure(const QnnTensorInfo& input,
                   const std::vector<QnnTensorInfo>& outputs,
                   std::string& error);

    // RGBA8 camera frame -> quantized NHWC input buffer.
    void preprocess(const uint8_t* rgba, int width, int height);

    // Runs the NPU and decodes detections (camera-frame coordinates).
    bool detect(QnnRuntime& runtime, std::vector<Detection>& out);
    // Decodes raw quantized outputs (already in outputs order) without
    // running the NPU; used by detect() and host-side tests.
    void decode(const std::vector<std::vector<uint8_t>>& outputs,
                std::vector<Detection>& out);
    // Sets the frame geometry normally established by preprocess().
    void setFrameSize(int width, int height);

    const std::vector<uint8_t>& inputBuffer() const { return input_; }

    bool isSegmentation() const { return coefIdx_ >= 0; }

    float confThreshold{0.35f};
    float iouThreshold{0.45f};
    size_t maxDetections{20};

private:
    int inputW_{640};
    int inputH_{640};
    float frameScale_{1.0f};   // model px per camera px
    int frameW_{0};
    int frameH_{0};

    int boxesIdx_{-1};
    int scoresIdx_{-1};
    int coefIdx_{-1};
    int protosIdx_{-1};
    QnnTensorInfo boxes_;
    QnnTensorInfo scores_;
    QnnTensorInfo coef_;
    QnnTensorInfo protos_;
    int protoH_{0}, protoW_{0}, numCoef_{0};

    // Decodes the instance mask of [d] from anchor [anchor].
    void decodeMask(Detection& d, int anchor,
                    const std::vector<std::vector<uint8_t>>& outputs) const;
    int numAnchors_{0};
    int numClasses_{0};

    std::vector<uint8_t> input_;
    std::vector<std::vector<uint8_t>> outputs_;
};

// Returns depth in metres at a colour-frame pixel, or 0 if unknown.
using DepthSampler = std::function<float(float x, float y)>;

// Median depth (metres) over the central part of the box; 0 if no data.
float boxDistance(const DepthSampler& depthAt, const Detection& det);

// Median depth (metres) over the mask pixels. [depthAt] maps a colour pixel
// to depth; returns 0 if fewer than ~10 mask pixels have depth.
float maskDistance(const DepthSampler& depthAt, const Detection& det);

// Draws instance masks (tinted) and 2px box outlines into an RGBA8 buffer.
void drawDetections(uint8_t* rgba, int width, int height, int stride,
                    const std::vector<Detection>& dets);

#endif  // YOLO_DETECTOR_H
