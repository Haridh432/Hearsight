#ifndef HSV2_IMU_H
#define HSV2_IMU_H

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>

/*
 * D455 IMU (BMI055) streaming on the motion-module sensor, independent of
 * the colour/depth rs2::pipeline. Gyro and accel run on their own callback
 * (librealsense thread) and feed a complementary filter for pitch/roll.
 *
 * Axes are the D455 IMU frame (librealsense): x right, y down, z forward.
 * At rest and level, accel reads about (0, -9.81, 0).
 *   pitch > 0 : camera tilted up (looking above the horizon)
 *   roll  > 0 : camera rolled clockwise (as seen from behind)
 */
struct ImuSnapshot {
    bool running{false};
    float gyro[3]{};          // rad/s
    float accel[3]{};         // m/s^2
    float pitchDeg{0.0f};     // complementary filter
    float rollDeg{0.0f};
    float accelPitchDeg{0.0f};  // accel-only angles (for verification)
    float accelRollDeg{0.0f};
    float gyroHz{0.0f};       // measured sample rates
    float accelHz{0.0f};
    int gyroProfileHz{0};     // configured rates
    int accelProfileHz{0};
    unsigned long long gyroCount{0};
    unsigned long long accelCount{0};
    double accelAgeSec{-1};   // age of the last accel sample, s (-1 = none yet)
};

class ImuStreamer {
public:
    // Opens and starts gyro + accel on the device's motion sensor with requested target rates.
    // preferredGyroHz / preferredAccelHz: <= 0 for hardware default (highest native rates: 400 Hz / 200 Hz).
    bool start(const rs2::device& device, std::string& error, int preferredGyroHz = -1, int preferredAccelHz = -1);
    void stop();

    ImuSnapshot snapshot();

    // Direct access for 3D mapping and safety:
    // camUp: unit vector pointing UP in camera coordinates (matches accel direction)
    // gyro: angular velocity in rad/s in camera coordinates
    // Returns true if running and fresh data is available.
    bool getMotion(float camUp[3], float gyro[3], double maxAgeSec = 0.1);

    // IMU -> depth-camera rotation from the device's extrinsics (factory / CAD values).
    // Call after start() with the pipeline's depth profile. Until then: identity.
    void setDepthProfile(const rs2::stream_profile& depth);

    // Up (unit, depth-camera axes) and gyro (rad/s, depth axes) at a video frame's
    // timestamp (ms, same camera clock): gravity interpolated between the accel samples
    // around it, gyro averaged over the frame interval. [matchMs] = distance to the
    // nearest accel sample. False when no sample lies within maxGapMs (e.g. the video and
    // IMU clocks differ) - the caller then uses getMotion().
    bool motionAt(double frameTsMs, float up[3], float gyro[3], double& matchMs,
                  double maxGapMs = 40.0, double* accelDevMax = nullptr);
    // [accelDevMax]: largest | |accel| - 9.81 | (m/s^2) of the raw samples in the frame interval
    // (step bounce); used to pick steady frames for the floor alignment.

    // Raw samples with after < ts <= upTo (hardware ms; last ~2 s kept), in IMU axes:
    // type 1 accel (m/s^2), 2 gyro (rad/s). For recordings and VIO. Any thread.
    struct RawSample { double ts; int type; float v[3]; };
    void rawSamples(double after, double upTo, std::vector<RawSample>& out);

private:
    double lastAccelSteady_{-1};   // steady clock (s) of the last accel sample
    void onFrame(const rs2::frame& frame);

    std::mutex mutex_;
    rs2::sensor sensor_;
    bool started_{false};

    ImuSnapshot state_;
    bool filterInit_{false};
    double lastGyroTs_{0.0};     // ms, hardware timestamps
    bool haveAccel_{false};
    float gravity_[3]{};         // low-passed accel
    // Last ~1 s of samples (hardware ms timestamps): low-passed accel, raw gyro.
    struct Sample { double ts; float v[3]; };
    std::deque<Sample> accHist_, gyrHist_;
    std::deque<RawSample> raw_;   // raw accel + gyro, time order, ~2 s
    rs2::stream_profile accelProfile_;
    float R_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};   // row-major, depth = R * imu
    void rotate(const float in[3], float out[3]) const;

    // Rate measurement window.
    double rateWindowStart_{0.0};
    unsigned long long rateGyro0_{0};
    unsigned long long rateAccel0_{0};
};

#endif  // HSV2_IMU_H
