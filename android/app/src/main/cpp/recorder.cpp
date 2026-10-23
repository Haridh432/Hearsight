#include "recorder.h"

#include <cstddef>
#include <cstring>
#include <sys/stat.h>

bool Recorder::start(const std::string& path, const RecHeader& h) {
    stop();
    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) return false;
    // Large stdio buffer: one write per ~frame.
    std::setvbuf(file_, nullptr, _IOFBF, 1 << 20);
    h_ = h;
    std::memcpy(h_.magic, "HSVREC4", 8);
    lastImuTs_ = -1;
    std::fwrite(&h_, sizeof(h_), 1, file_);
    path_ = path;
    ::mkdir((path_ + ".frames").c_str(), 0775);
    frames_ = 0;
    dropped_ = 0;
    bytes_ = sizeof(h_);
    quit_ = false;
    thread_ = std::thread(&Recorder::loop, this);
    active_ = true;
    return true;
}

void Recorder::stop() {
    if (!active_ && !thread_.joinable()) return;
    active_ = false;
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

void Recorder::push(double t, const uint8_t* gray, const uint16_t* depthFull, const VoResult& r,
                    const float pose[3], const RecImu& imu) {
    if (!active_) return;
    Item it;
    it.f.t = t;
    it.f.voOk = r.ok ? 1 : 0;
    for (int i = 0; i < 3; ++i) { it.f.vel[i] = r.vel[i]; it.f.pose[i] = pose[i]; }
    it.f.angRate = r.angRate;
    it.f.inliers = r.inliers;
    it.f.tracked = r.tracked;
    it.f.withDepth = r.withDepth;
    it.f.imuValid = imu.valid ? 1 : 0;
    for (int i = 0; i < 3; ++i) { it.f.phoneUp[i] = imu.up[i]; it.f.gyro[i] = imu.gyro[i]; }
    it.gray.assign(gray, gray + static_cast<size_t>(h_.grayW) * h_.grayH);
    if (imuSource_ && imu.camTsMs >= 0) {
        const double after = lastImuTs_ >= 0 && lastImuTs_ < imu.camTsMs ? lastImuTs_ : imu.camTsMs - 50.0;
        imuSource_(after, imu.camTsMs, it.imu);
        lastImuTs_ = imu.camTsMs;
        it.d455 = {imu.camTsMs, static_cast<uint32_t>(it.imu.size())};
    }
    {
        std::lock_guard<std::mutex> lk(segM_);
        if (segW_ > 0 && segT_ >= 0 && t - segT_ < 1.0 && t - segT_ > -1.0) {
            it.seg = {static_cast<uint16_t>(segW_), static_cast<uint16_t>(segH_), static_cast<float>(t - segT_)};
            it.cls = segCls_;
        }
    }
    it.depth.resize(static_cast<size_t>(h_.depW) * h_.depH);
    const int dw = h_.reg.dw;
    for (int y = 0; y < h_.depH; ++y)
        for (int x = 0; x < h_.depW; ++x)
            it.depth[static_cast<size_t>(y) * h_.depW + x] = depthFull[static_cast<size_t>(2 * y) * dw + 2 * x];
    {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() >= kMaxQueue) { dropped_++; return; }
        q_.push_back(std::move(it));
    }
    cv_.notify_one();
}

void Recorder::pushColor(double t, const uint8_t* rgba, int w, int h) {
    if (!active_) return;
    ColorItem c{t, w / 2, h / 2, {}};
    c.rgb.resize(static_cast<size_t>(c.w) * c.h * 3);
    for (int y = 0; y < c.h; ++y)
        for (int x = 0; x < c.w; ++x) {
            const uint8_t* p = rgba + (static_cast<size_t>(2 * y) * w + 2 * x) * 4;
            uint8_t* o = &c.rgb[(static_cast<size_t>(y) * c.w + x) * 3];
            o[0] = p[0]; o[1] = p[1]; o[2] = p[2];
        }
    {
        std::lock_guard<std::mutex> lk(m_);
        if (colorQ_.size() >= 4) { dropped_++; return; }
        colorQ_.push_back(std::move(c));
    }
    cv_.notify_one();
}

void Recorder::loop() {
    for (;;) {
        Item it;
        bool haveItem = false;
        ColorItem col;
        bool haveCol = false;
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return quit_ || !q_.empty() || !colorQ_.empty(); });
            if (!colorQ_.empty()) { col = std::move(colorQ_.front()); colorQ_.pop_front(); haveCol = true; }
            if (!q_.empty()) { it = std::move(q_.front()); q_.pop_front(); haveItem = true; }
            if (!haveItem && !haveCol) break;   // quit and drained
        }
        if (haveCol) {
            char name[64];
            snprintf(name, sizeof name, "/%013.0f.ppm", col.t * 1000.0);
            if (std::FILE* f = std::fopen((path_ + ".frames" + name).c_str(), "wb")) {
                std::fprintf(f, "P6\n%d %d\n255\n", col.w, col.h);
                std::fwrite(col.rgb.data(), 1, col.rgb.size(), f);
                std::fclose(f);
                bytes_ += col.rgb.size();
            }
        }
        if (!haveItem) continue;
        std::fwrite(&it.f, sizeof(it.f), 1, file_);
        std::fwrite(it.gray.data(), 1, it.gray.size(), file_);
        std::fwrite(it.depth.data(), 2, it.depth.size(), file_);
        std::fwrite(&it.seg, sizeof(it.seg), 1, file_);
        if (!it.cls.empty()) std::fwrite(it.cls.data(), 1, it.cls.size(), file_);
        std::fwrite(&it.d455, sizeof(it.d455), 1, file_);
        if (!it.imu.empty()) std::fwrite(it.imu.data(), sizeof(RecImuSample), it.imu.size(), file_);
        bytes_ += sizeof(it.f) + it.gray.size() + 2 * it.depth.size() + sizeof(it.seg) + it.cls.size() +
                  sizeof(it.d455) + sizeof(RecImuSample) * it.imu.size();
        frames_++;
    }
}

void Recorder::setSegmentation(const uint8_t* cls, int w, int h, double frameTimeSec) {
    std::lock_guard<std::mutex> lk(segM_);
    segCls_.assign(cls, cls + static_cast<size_t>(w) * h);
    segW_ = w;
    segH_ = h;
    segT_ = frameTimeSec;
}

bool RecReader::open(const char* path) {
    f = std::fopen(path, "rb");
    if (!f) return false;
    if (std::fread(&h, sizeof(h), 1, f) != 1) return false;
    if (std::memcmp(h.magic, "HSVREC4", 7) == 0) version = 4;
    else if (std::memcmp(h.magic, "HSVREC3", 7) == 0) version = 3;
    else if (std::memcmp(h.magic, "HSVREC2", 7) == 0) version = 2;
    else if (std::memcmp(h.magic, "HSVREC1", 7) == 0) version = 1;
    return version > 0;
}

bool RecReader::next(RecFrame& fr, std::vector<uint8_t>& gray, std::vector<uint16_t>& depth,
                     RecSeg* seg, std::vector<uint8_t>* cls, RecD455* d455,
                     std::vector<RecImuSample>* imu) {
    // Version 1 frames end before the IMU fields.
    const size_t frameSize = version == 1 ? offsetof(RecFrame, imuValid) : sizeof(RecFrame);
    fr = RecFrame{};
    if (std::fread(&fr, frameSize, 1, f) != 1) return false;
    gray.resize(static_cast<size_t>(h.grayW) * h.grayH);
    depth.resize(static_cast<size_t>(h.depW) * h.depH);
    if (std::fread(gray.data(), 1, gray.size(), f) != gray.size()) return false;
    if (std::fread(depth.data(), 2, depth.size(), f) != depth.size()) return false;
    RecSeg sg{0, 0, 0};
    std::vector<uint8_t> tmp;
    if (version >= 3) {
        if (std::fread(&sg, sizeof(sg), 1, f) != 1) return false;
        const size_t n = static_cast<size_t>(sg.w) * sg.h;
        std::vector<uint8_t>& dst = cls ? *cls : tmp;
        dst.resize(n);
        if (n && std::fread(dst.data(), 1, n, f) != n) return false;
    } else if (cls) {
        cls->clear();
    }
    if (seg) *seg = sg;
    RecD455 dh{-1, 0};
    std::vector<RecImuSample> itmp;
    std::vector<RecImuSample>& idst = imu ? *imu : itmp;
    idst.clear();
    if (version >= 4) {
        if (std::fread(&dh, sizeof(dh), 1, f) != 1) return false;
        idst.resize(dh.n);
        if (dh.n && std::fread(idst.data(), sizeof(RecImuSample), dh.n, f) != dh.n) return false;
    }
    if (d455) *d455 = dh;
    return true;
}
