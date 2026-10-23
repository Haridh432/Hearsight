#include "yolo_detector.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

float iou(const Detection& a, const Detection& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.w, b.x + b.w);
    float y2 = std::min(a.y + a.h, b.y + b.h);
    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

}  // namespace


bool YoloDetector::configure(const QnnRuntime& runtime, std::string& error) {
    return configure(runtime.inputInfo(), runtime.outputInfos(), error);
}


bool YoloDetector::configure(const QnnTensorInfo& in,
                             const std::vector<QnnTensorInfo>& outs,
                             std::string& error) {
    if (in.dims.size() != 4 || in.dims[3] != 3) {
        error = "Expected NHWC [1,H,W,3] input";
        return false;
    }

    inputH_ = static_cast<int>(in.dims[1]);
    inputW_ = static_cast<int>(in.dims[2]);
    input_.assign(in.bytes, 0);

    boxesIdx_ = scoresIdx_ = -1;

    for (size_t i = 0; i < outs.size(); ++i) {
        if (outs[i].name == "boxes") boxesIdx_ = static_cast<int>(i);
        if (outs[i].name == "scores") scoresIdx_ = static_cast<int>(i);
    }

    coefIdx_ = protosIdx_ = -1;
    for (size_t i = 0; i < outs.size(); ++i) {
        if (outs[i].name == "mask_coef") coefIdx_ = static_cast<int>(i);
        if (outs[i].name == "protos") protosIdx_ = static_cast<int>(i);
    }

    if (boxesIdx_ < 0 || scoresIdx_ < 0) {
        error = "Model must expose 'boxes' and 'scores' outputs";
        return false;
    }

    if ((coefIdx_ >= 0) != (protosIdx_ >= 0)) {
        error = "Segmentation model needs both 'mask_coef' and 'protos'";
        return false;
    }
    if (coefIdx_ >= 0) {
        coef_ = outs[coefIdx_];
        protos_ = outs[protosIdx_];
        // protos are NHWC [1, H, W, C] after QNN conversion.
        if (coef_.dims.size() != 3 || protos_.dims.size() != 4 ||
            protos_.dims[3] != coef_.dims[1]) {
            error = "Unexpected mask_coef/protos shapes";
            return false;
        }
        numCoef_ = static_cast<int>(coef_.dims[1]);
        protoH_ = static_cast<int>(protos_.dims[1]);
        protoW_ = static_cast<int>(protos_.dims[2]);
    }

    boxes_ = outs[boxesIdx_];
    scores_ = outs[scoresIdx_];

    if (boxes_.dims.size() != 3 || scores_.dims.size() != 3 ||
        boxes_.dims[1] != 4 || boxes_.dims[2] != scores_.dims[2]) {
        error = "Unexpected boxes/scores shapes";
        return false;
    }

    numClasses_ = static_cast<int>(scores_.dims[1]);
    numAnchors_ = static_cast<int>(scores_.dims[2]);
    return true;
}


void YoloDetector::setFrameSize(int width, int height) {
    frameW_ = width;
    frameH_ = height;
    frameScale_ = std::min(
        static_cast<float>(inputW_) / width,
        static_cast<float>(inputH_) / height);
}


void YoloDetector::preprocess(const uint8_t* rgba, int width, int height) {
    setFrameSize(width, height);

    int scaledW = std::min(inputW_, static_cast<int>(width * frameScale_));
    int scaledH = std::min(inputH_, static_cast<int>(height * frameScale_));

    // Letterbox grey (114), image in the top-left corner.
    std::memset(input_.data(), 114, input_.size());

    // Input scale is 1/255 with offset 0, so RGB bytes are already
    // the quantized values. Nearest-neighbour resize (a no-op copy
    // for the 640x480 stream).
    for (int y = 0; y < scaledH; ++y) {
        int sy = std::min(height - 1, static_cast<int>(y / frameScale_));
        const uint8_t* src = rgba + static_cast<size_t>(sy) * width * 4;
        uint8_t* dst = input_.data() + static_cast<size_t>(y) * inputW_ * 3;

        for (int x = 0; x < scaledW; ++x) {
            int sx = std::min(width - 1, static_cast<int>(x / frameScale_));
            const uint8_t* p = src + sx * 4;
            dst[x * 3 + 0] = p[0];
            dst[x * 3 + 1] = p[1];
            dst[x * 3 + 2] = p[2];
        }
    }
}


bool YoloDetector::detect(QnnRuntime& runtime, std::vector<Detection>& out) {
    out.clear();

    if (!runtime.execute(input_.data(), input_.size(), outputs_)) {
        return false;
    }

    decode(outputs_, out);
    return true;
}


void YoloDetector::decode(const std::vector<std::vector<uint8_t>>& outputs,
                          std::vector<Detection>& out) {
    out.clear();
    const uint8_t* boxes = outputs[boxesIdx_].data();
    const uint8_t* scores = outputs[scoresIdx_].data();

    // Compare in the quantized domain: q >= conf/scale - offset.
    int qThresh = static_cast<int>(
        std::ceil(confThreshold / scores_.scale - scores_.offset));
    qThresh = std::max(qThresh, 1);

    auto deq = [](uint8_t q, const QnnTensorInfo& t) {
        return (static_cast<int>(q) + t.offset) * t.scale;
    };

    std::vector<Detection> candidates;
    std::vector<int> candidateAnchor;

    for (int a = 0; a < numAnchors_; ++a) {
        int bestClass = -1;
        int bestQ = qThresh - 1;

        for (int c = 0; c < numClasses_; ++c) {
            int q = scores[static_cast<size_t>(c) * numAnchors_ + a];
            if (q > bestQ) {
                bestQ = q;
                bestClass = c;
            }
        }

        if (bestClass < 0) continue;

        float cx = deq(boxes[0 * numAnchors_ + a], boxes_);
        float cy = deq(boxes[1 * numAnchors_ + a], boxes_);
        float bw = deq(boxes[2 * numAnchors_ + a], boxes_);
        float bh = deq(boxes[3 * numAnchors_ + a], boxes_);

        // Model pixels -> camera pixels (top-left letterbox).
        Detection d;
        d.classId = bestClass;
        d.confidence = deq(static_cast<uint8_t>(bestQ), scores_);
        d.x = (cx - bw * 0.5f) / frameScale_;
        d.y = (cy - bh * 0.5f) / frameScale_;
        d.w = bw / frameScale_;
        d.h = bh / frameScale_;

        // Clip to the frame.
        float x2 = std::min(static_cast<float>(frameW_), d.x + d.w);
        float y2 = std::min(static_cast<float>(frameH_), d.y + d.h);
        d.x = std::max(0.0f, d.x);
        d.y = std::max(0.0f, d.y);
        d.w = x2 - d.x;
        d.h = y2 - d.y;

        if (d.w > 1.0f && d.h > 1.0f) {
            candidates.push_back(d);
            candidateAnchor.push_back(a);
        }
    }

    // Sort by confidence, keeping each candidate's anchor index.
    std::vector<size_t> order(candidates.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t l, size_t r) {
        return candidates[l].confidence > candidates[r].confidence;
    });
    {
        std::vector<Detection> c2; std::vector<int> a2;
        c2.reserve(order.size()); a2.reserve(order.size());
        for (size_t i : order) { c2.push_back(candidates[i]); a2.push_back(candidateAnchor[i]); }
        candidates.swap(c2); candidateAnchor.swap(a2);
    }

    // Per-class greedy NMS.
    std::vector<bool> removed(candidates.size(), false);

    for (size_t i = 0; i < candidates.size() && out.size() < maxDetections; ++i) {
        if (removed[i]) continue;
        out.push_back(candidates[i]);
        // Instance masks for every detected object.
        if (isSegmentation()) {
            decodeMask(out.back(), candidateAnchor[i], outputs);
        }

        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (!removed[j] &&
                candidates[j].classId == candidates[i].classId &&
                iou(candidates[i], candidates[j]) > iouThreshold) {
                removed[j] = true;
            }
        }
    }

}


void YoloDetector::decodeMask(Detection& d, int anchor,
                              const std::vector<std::vector<uint8_t>>& outputs) const {
    const uint8_t* coefQ = outputs[coefIdx_].data();
    const uint8_t* protoQ = outputs[protosIdx_].data();

    // Dequantized coefficients for this anchor.
    std::vector<float> coef(numCoef_);
    for (int k = 0; k < numCoef_; ++k) {
        coef[k] = (static_cast<int>(coefQ[static_cast<size_t>(k) * numAnchors_ + anchor]) +
                   coef_.offset) * coef_.scale;
    }
    // logit = sum_k coef_k * protoScale * (q_k + protoOffset)
    //       = protoScale * (sum_k coef_k*q_k + protoOffset*sum_k coef_k)
    float coefSum = 0.0f;
    for (float c : coef) coefSum += c;
    const float bias = protos_.offset * coefSum;

    // Proto grid cell size in model pixels (640 / 160 = 4).
    const float cellModel = static_cast<float>(inputW_) / protoW_;
    const float cellCam = cellModel / frameScale_;

    // Box in proto cells.
    int x0 = std::max(0, static_cast<int>(d.x * frameScale_ / cellModel));
    int y0 = std::max(0, static_cast<int>(d.y * frameScale_ / cellModel));
    int x1 = std::min(protoW_, static_cast<int>(std::ceil((d.x + d.w) * frameScale_ / cellModel)));
    int y1 = std::min(protoH_, static_cast<int>(std::ceil((d.y + d.h) * frameScale_ / cellModel)));
    if (x1 <= x0 || y1 <= y0) return;

    d.maskX = x0 * cellCam;
    d.maskY = y0 * cellCam;
    d.maskCellPx = cellCam;
    d.maskW = x1 - x0;
    d.maskH = y1 - y0;
    d.mask.assign(static_cast<size_t>(d.maskW) * d.maskH, 0);

    int on = 0;
    for (int y = y0; y < y1; ++y) {
        const uint8_t* row = protoQ + (static_cast<size_t>(y) * protoW_) * numCoef_;
        for (int x = x0; x < x1; ++x) {
            const uint8_t* p = row + static_cast<size_t>(x) * numCoef_;
            float acc = bias;
            for (int k = 0; k < numCoef_; ++k) acc += coef[k] * p[k];
            // sigmoid(scale*acc) > 0.5  <=>  acc > 0 (scale > 0)
            if (acc > 0.0f) {
                d.mask[static_cast<size_t>(y - y0) * d.maskW + (x - x0)] = 1;
                ++on;
            }
        }
    }
    d.hasMask = on > 0;
}


float maskDistance(const DepthSampler& depthAt, const Detection& det) {
    if (!det.hasMask) return 0.0f;

    // Sample at most ~100 mask cells, evenly spaced.
    int total = 0;
    for (uint8_t m : det.mask) total += m;
    if (total == 0) return 0.0f;
    const int step = std::max(1, total / 100);

    std::vector<float> samples;
    samples.reserve(128);
    int seen = 0;
    for (int y = 0; y < det.maskH; ++y) {
        for (int x = 0; x < det.maskW; ++x) {
            if (!det.mask[static_cast<size_t>(y) * det.maskW + x]) continue;
            if (seen++ % step) continue;
            float cx = det.maskX + (x + 0.5f) * det.maskCellPx;
            float cy = det.maskY + (y + 0.5f) * det.maskCellPx;
            float z = depthAt(cx, cy);
            if (z > 0.0f) samples.push_back(z);
        }
    }
    if (samples.size() < 10) return 0.0f;

    auto mid = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), mid, samples.end());
    return *mid;
}


float boxDistance(const DepthSampler& depthAt, const Detection& det) {
    // Sample the central 50% of the box on a 7x7 grid.
    std::vector<float> samples;
    samples.reserve(49);

    for (int iy = 0; iy < 7; ++iy) {
        for (int ix = 0; ix < 7; ++ix) {
            float x = det.x + det.w * (0.25f + 0.5f * ix / 6.0f);
            float y = det.y + det.h * (0.25f + 0.5f * iy / 6.0f);

            float z = depthAt(x, y);
            if (z > 0.0f) samples.push_back(z);
        }
    }

    if (samples.empty()) return 0.0f;

    auto mid = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), mid, samples.end());
    return *mid;
}


void drawDetections(uint8_t* rgba, int width, int height, int stride,
                    const std::vector<Detection>& dets) {
    const int thickness = 2;

    // Instance masks: tint each object's pixels (45%), one colour per class;
    // people stay magenta.
    static const uint8_t kPalette[8][3] = {
        {255, 0, 255}, {0, 200, 255}, {255, 160, 0}, {80, 255, 80},
        {255, 60, 60}, {120, 120, 255}, {255, 255, 0}, {0, 255, 180}};
    for (const Detection& d : dets) {
        if (!d.hasMask) continue;
        const uint8_t* c = kPalette[d.classId == 0 ? 0 : 1 + (d.classId % 7)];
        for (int my = 0; my < d.maskH; ++my) {
            int py0 = static_cast<int>(d.maskY + my * d.maskCellPx);
            int py1 = std::min(height, static_cast<int>(d.maskY + (my + 1) * d.maskCellPx));
            for (int mx = 0; mx < d.maskW; ++mx) {
                if (!d.mask[static_cast<size_t>(my) * d.maskW + mx]) continue;
                int px0 = static_cast<int>(d.maskX + mx * d.maskCellPx);
                int px1 = std::min(width, static_cast<int>(d.maskX + (mx + 1) * d.maskCellPx));
                for (int y = std::max(0, py0); y < py1; ++y) {
                    uint8_t* row = rgba + static_cast<size_t>(y) * stride * 4;
                    for (int x = std::max(0, px0); x < px1; ++x) {
                        uint8_t* p = row + x * 4;
                        p[0] = static_cast<uint8_t>((p[0] * 55 + c[0] * 45) / 100);
                        p[1] = static_cast<uint8_t>((p[1] * 55 + c[1] * 45) / 100);
                        p[2] = static_cast<uint8_t>((p[2] * 55 + c[2] * 45) / 100);
                    }
                }
            }
        }
    }

    for (const Detection& d : dets) {
        int x0 = std::max(0, static_cast<int>(d.x));
        int y0 = std::max(0, static_cast<int>(d.y));
        int x1 = std::min(width - 1, static_cast<int>(d.x + d.w));
        int y1 = std::min(height - 1, static_cast<int>(d.y + d.h));

        auto put = [&](int x, int y) {
            uint8_t* p = rgba + (static_cast<size_t>(y) * stride + x) * 4;
            p[0] = 0; p[1] = 255; p[2] = 0; p[3] = 255;
        };

        for (int t = 0; t < thickness; ++t) {
            for (int x = x0; x <= x1; ++x) {
                if (y0 + t <= y1) put(x, y0 + t);
                if (y1 - t >= y0) put(x, y1 - t);
            }
            for (int y = y0; y <= y1; ++y) {
                if (x0 + t <= x1) put(x0 + t, y);
                if (x1 - t >= x0) put(x1 - t, y);
            }
        }
    }
}
