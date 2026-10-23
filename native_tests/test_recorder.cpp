// Recorder format v3: segmentation written with each frame and read back; v2-style
// frames (no fresh segmentation) read as w = h = 0.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "recorder.h"

static int fails = 0;
static void check(const char* name, bool ok) { printf("  %-48s: %s\n", name, ok ? "OK" : "FAIL"); if (!ok) ++fails; }

int main() {
    printf("recorder\n");
    const std::string path = std::string(getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp") + "/hsv2_rec_test.hsr";
    RecHeader h{};
    h.grayW = 8; h.grayH = 6; h.depW = 4; h.depH = 3; h.depthUnits = 0.001f;
    h.reg.dw = 8; h.reg.dh = 6;
    std::vector<uint8_t> gray(48, 7);
    std::vector<uint16_t> depthFull(48, 1234);
    VoResult r{}; float pose[3] = {1, 2, 3}; RecImu imu{};
    {
        Recorder rec;
        // Fake D455 IMU: accel + gyro every 5 ms from 0 to 2000 ms (camera clock).
        rec.setImuSource([](double after, double upTo, std::vector<RecImuSample>& out) {
            for (int k = 0; k <= 400; ++k) {
                const double ts = 5.0 * k;
                if (ts <= after || ts > upTo) continue;
                out.push_back({ts, 1, {0.0f, -9.8f, 0.0f}});
                out.push_back({ts, 2, {0.01f, 0.0f, 0.0f}});
            }
        });
        check("start", rec.start(path, h));
        imu.camTsMs = 1000.0;
        rec.push(10.00, gray.data(), depthFull.data(), r, pose, imu);       // no segmentation yet
        std::vector<uint8_t> cls(12);
        for (int i = 0; i < 12; ++i) cls[i] = static_cast<uint8_t>(i);
        rec.setSegmentation(cls.data(), 4, 3, 10.02);
        imu.camTsMs = 1033.0;
        rec.push(10.05, gray.data(), depthFull.data(), r, pose, imu);       // fresh (0.03 s)
        imu.camTsMs = -1;                                                    // no IMU
        rec.push(11.50, gray.data(), depthFull.data(), r, pose, imu);       // stale (1.48 s)
        std::vector<uint8_t> rgba(8 * 6 * 4, 200);
        rec.pushColor(10.25, rgba.data(), 8, 6);                            // colour snapshot
        rec.stop();
        check("3 frames written", rec.frames() == 3);
    }
    RecReader rd;
    check("open v4", rd.open(path.c_str()) && rd.version == 4);
    RecFrame fr; std::vector<uint8_t> g, cls; std::vector<uint16_t> d; RecSeg sg{};
    RecD455 dh{}; std::vector<RecImuSample> im;
    check("frame 1: no segmentation", rd.next(fr, g, d, &sg, &cls, &dh, &im) && sg.w == 0 && cls.empty());
    // First frame: the 50 ms before it (955..1000 -> 10 accel + 10 gyro).
    check("frame 1: D455 samples 950 < ts <= 1000", dh.frameTsMs == 1000.0 && im.size() == 20 &&
          im.front().ts > 950.0 && im.back().ts <= 1000.0);
    bool f2 = rd.next(fr, g, d, &sg, &cls, &dh, &im);
    // Second frame: everything after the first frame up to 1033 (1005..1030 -> 6 + 6), no gap.
    check("frame 2: D455 samples continue (1000, 1033]", f2 && im.size() == 12 && im.front().ts == 1005.0 &&
          im.back().ts == 1030.0 && im[0].type == 1 && im[0].v[1] == -9.8f);
    check("frame 2: 4x3 segmentation, age 0.03 s", f2 && sg.w == 4 && sg.h == 3 && cls.size() == 12 && cls[11] == 11 &&
          sg.age > 0.02f && sg.age < 0.04f);
    check("frame 2: gray / depth intact", g.size() == 48 && g[5] == 7 && d.size() == 12 && d[3] == 1234);
    check("frame 3: stale -> none, no IMU", rd.next(fr, g, d, &sg, &cls, &dh, &im) && sg.w == 0 && im.empty() && dh.n == 0);
    check("end of file", !rd.next(fr, g, d, &sg, &cls));
    {
        const std::string snap = path + ".frames/0000000010250.ppm";
        FILE* f = std::fopen(snap.c_str(), "rb");
        char hdr[32] = {0};
        const bool ok = f && std::fgets(hdr, sizeof hdr, f) && std::string(hdr) == "P6\n";
        if (f) std::fclose(f);
        check("colour snapshot saved (half size, PPM)", ok);
        std::remove(snap.c_str());
        std::remove((path + ".frames").c_str());
    }
    std::remove(path.c_str());
    printf("recorder failures: %d\n", fails);
    return fails ? 1 : 0;
}
