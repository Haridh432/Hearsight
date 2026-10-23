class DetectionResult {
  final String objectName;
  final double confidence;
  final double distanceMeters;
  final String directionClock;
  final List<double> boundingBox; // [x, y, width, height]
  final bool masked;          // instance mask available (persons)
  final int trackId;          // -1 until the track is confirmed
  final double speed;         // m/s
  final bool approaching;     // closing in on the user
  final double ttc;           // time-to-collision (s), -1 if not closing

  DetectionResult({
    required this.objectName,
    required this.confidence,
    required this.distanceMeters,
    required this.directionClock,
    required this.boundingBox,
    this.masked = false,
    this.trackId = -1,
    this.speed = 0.0,
    this.approaching = false,
    this.ttc = -1.0,
  });
}

/// D455 IMU state from native telemetry. Angles in degrees
/// (pitch > 0: camera tilted up, roll > 0: rolled clockwise from behind).
class ImuData {
  final bool running;
  final String source;      // "phone" or "d455"
  final double pitchDeg;
  final double rollDeg;
  final List<double> gyro;  // rad/s, IMU frame (x right, y down, z forward)
  final List<double> accel; // m/s^2
  final double gyroHz;
  final double accelHz;
  final double ageMs;       // age of the last accel sample (-1 = none received)
  final bool d455Failed;    // D455 IMU gave no data after a restart: phone sensors used

  const ImuData({
    this.running = false,
    this.source = '',
    this.pitchDeg = 0.0,
    this.rollDeg = 0.0,
    this.gyro = const [0.0, 0.0, 0.0],
    this.accel = const [0.0, 0.0, 0.0],
    this.gyroHz = 0.0,
    this.accelHz = 0.0,
    this.ageMs = -1.0,
    this.d455Failed = false,
  });

  factory ImuData.fromJson(Map<String, dynamic> j) {
    List<double> vec(dynamic v) =>
        (v as List<dynamic>?)?.map((e) => (e as num).toDouble()).toList() ?? const [0.0, 0.0, 0.0];
    return ImuData(
      running: j['running'] as bool? ?? false,
      source: j['source'] as String? ?? '',
      pitchDeg: (j['pitch'] as num?)?.toDouble() ?? 0.0,
      rollDeg: (j['roll'] as num?)?.toDouble() ?? 0.0,
      gyro: vec(j['gyro']),
      accel: vec(j['accel']),
      gyroHz: (j['gyroHz'] as num?)?.toDouble() ?? 0.0,
      accelHz: (j['accelHz'] as num?)?.toDouble() ?? 0.0,
      ageMs: (j['ageMs'] as num?)?.toDouble() ?? -1.0,
      d455Failed: j['d455Failed'] as bool? ?? false,
    );
  }
}

class TelemetryData {
  final double fps;
  final double p50LatencyMs;
  final double p95LatencyMs;
  final double p99LatencyMs;
  final double cpuUsage;
  final double gpuUsage;
  final double npuUsage;
  final double thermalState;
  final double batteryLevel;
  final int droppedFrames;
  final double usbTransportLatencyMs;
  final double centerDistanceMeters;
  final String alert;          // "none" | "caution" | "stop"
  // obstacle | head | dropoff | unknown | nofloor | moving
  final String alertReason;
  final double alertDist;      // metres to the hazard (-1 if n/a)
  final double alertTtc;       // seconds (moving objects)
  // Planner (Phase 4b): path ahead or "blocked".
  final bool planOk;
  final bool planBlocked;
  final double planTurn;       // degrees, + = left
  final double planLength;     // metres
  final String guideCode;      // Phase 5 guidance: straight, bear_left, turn_right_soon, door, stop, ...
  final String guideText;      // e.g. "Turn left in 2 m"
  final int guideSeq;          // increments when the instruction changes
  final String aheadSrc;       // what set the 'object ahead' distance: obstacle / head / low / trip / det:<class>
  final ImuData imu;

  TelemetryData({
    this.fps = 0.0,
    this.p50LatencyMs = 0.0,
    this.p95LatencyMs = 0.0,
    this.p99LatencyMs = 0.0,
    this.cpuUsage = 0.0,
    this.gpuUsage = 0.0,
    this.npuUsage = 0.0,
    this.thermalState = 0.0,
    this.batteryLevel = 100.0,
    this.droppedFrames = 0,
    this.usbTransportLatencyMs = 0.0,
    this.centerDistanceMeters = 0.0,
    this.alert = 'none',
    this.alertReason = 'none',
    this.alertDist = -1.0,
    this.alertTtc = -1.0,
    this.planOk = false,
    this.planBlocked = false,
    this.planTurn = 0.0,
    this.planLength = 0.0,
    this.guideCode = 'none',
    this.guideText = '',
    this.guideSeq = 0,
    this.aheadSrc = '',
    this.imu = const ImuData(),
  });
}

enum PipelineStage {
  stage1,
  stage2,
  stage3,
  stage4,
}

class StageConfig {
  final PipelineStage currentStage;
  final bool isStage3Enabled; // Feature flag for stage 3

  StageConfig({
    required this.currentStage,
    this.isStage3Enabled = false,
  });
}
