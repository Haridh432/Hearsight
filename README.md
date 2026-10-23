# HearSight + Sago Unified Navigation & Voice Assistant (hsv2-lite)

**A single unified assistive navigation and voice assistant application for blind and low-vision users.**

This repository is a unified merger of two open-source platforms:
1. **HearSight** ([rnaveen1236/Hearsight](https://github.com/rnaveen1236/Hearsight)): An Intel RealSense D455 depth camera on a head mount feeds an Android phone; the phone finds the floor, obstacles, drop-offs, stairs, and people, computes RGB + depth distance and direction, builds a local 3D map, plans a safe path, and provides deterministic navigation guidance.
2. **Sago** ([TringyTrongy/argus](https://github.com/TringyTrongy/argus)): Offline voice navigation, voice assistant, and speech recognition pipeline featuring on-device Sherpa ONNX, Moonshine/Parakeet ASR, Silero VAD, AudioRoute management, and real-time voice command processing.

Everything runs **on the phone completely offline**:
- **Computer Vision & Perception**: Intel RealSense D455 RGB and depth streaming, YOLOv8s object detection, DeepLab ADE20K semantic segmentation on the Snapdragon NPU (Qualcomm QNN), geometry and D* Lite planning in C++, Flutter UI.
- **Voice Assistant & Speech Recognition**: On-device Moonshine / Parakeet ASR, Silero Voice Activity Detection (VAD), earcon feedback, and speech command processing (`com.hsv2.hsv2_lite.voice`) via sherpa-onnx.

> Status: Unified single repository under active development and walk-testing.
> Not a certified mobility aid — always use with a cane or guide.

---

## Contents

- [What it does](#what-it-does)
- [Hardware](#hardware)
- [How it works](#how-it-works)
- [Safety design](#safety-design)
- [Voice and vibration](#voice-and-vibration)
- [Project layout](#project-layout)
- [Building](#building)
- [Installing and running](#installing-and-running)
- [Testing](#testing)
- [Validation walks (Phase 7)](#validation-walks-phase-7)
- [Debug switches](#debug-switches)
- [Development phases](#development-phases)
- [Known limitations](#known-limitations)
- [Third-party code and licences](#third-party-code-and-licences)

---

## What it does

| Capability | Details |
|---|---|
| **Obstacle warning** | Body-height and head-height obstacles, low objects and trip hazards (3–10 cm) in the walking corridor; any object directly ahead within **0.5 m** → **"STOP! … directly ahead."**, then a new way round it ("Stop, way is left") |
| **Drop-offs and stairs** | Floor falling away, steps and stairs ahead → STOP / CAUTION; drops are judged against the floor tracked step by step (glossy floors and mirror images do not count) |
| **Moving objects** | People and vehicles tracked with velocity and time-to-collision |
| **Scene understanding** | YOLOv8s (objects, person masks) + DeepLab (ADE20K: wall, floor, door, stairs, …) on the NPU |
| **Local 3D map** | 16 × 16 m map (10 cm cells) that remembers obstacles, drop-offs and classes; visual odometry, gravity from the D455 IMU (phone IMU as fallback) |
| **Path planning** | Incremental **D\* Lite**: safe path 2.5–5 m ahead, kept away from walls, re-planned as the map changes (people, new obstacles); doorways stay passable; the way round an obstacle keeps its side |
| **Guidance** | Live situation sentences, the same on screen and in the voice: "Chair 2 metres ahead, path goes right.", "Stopped, a table blocks, way on the left.", "Nothing within 3 metres.", "Door 3 metres ahead", "No clear way", "Wait, checking the way" |
| **Scene description** | Optional **Gemma 4 E2B** on the phone (LiteRT-LM GPU): "What's ahead?" on request, and ✨ Live comments on events (obstacle, door, stopped); never overrides the safety decision |
| **Voice** | English, Tamil, or both (each in its own voice); urgent messages interrupt; no chatter |
| **Vibration** | STOP pattern, left / right cues, camera-lost pattern; works in silent mode |
| **Sensor health** | Camera covered / unplugged / frozen / lagging → **"STOP! Camera …"**; 1 s start-up warm-up ("Wait, checking the way") instead of a first-frame STOP |
| **3D view** | Live 3D map with class colours and a top-down navigation map inset; path **green** when verified safe, **dashed amber** ("path (wait)") when planned but not yet safe, hidden with a red bar + X during a 1 m STOP |
| **Accessibility** | TalkBack labels, large START / STOP, spoken start / stop, screen kept on |

## Hardware

| Part | Used / tested |
|---|---|
| Depth camera | Intel RealSense **D455** (colour + depth + IMU over USB) |
| Phone | OnePlus **CPH2745** (Snapdragon **SM8850**, Hexagon **v81** NPU) — main target; CPH2729 (SM6650, v73) also supported |
| Android | 12+ (minSdk 31), tested on Android 16 |
| Mount | Head mount holding camera and phone; camera level (the D455 IMU gives gravity; a rigid phone is only needed for the phone-IMU fallback) |

The phone's single USB port is used by the D455 during walks, so development uses
**wireless debugging** (`adb connect <ip>:<port>`); USB is far faster for installs when free.

## How it works

```
 D455 colour + depth (30 fps) + IMU (gyro/accel ~400 Hz)   phone IMU (fallback)
          │                                                   │
          ▼                                                   ▼
 ┌─────────────────── native (C++, one stream thread) ───────────────────┐
 │ 1. Capture & sync        frames + D455 IMU (camera clock, ~1 ms)      │
 │ 2. Perception (NPU)      YOLOv8s-seg + DeepLab via QNN HTP            │
 │ 3. Safety layer          floor fit, walking corridor, obstacles,      │
 │                          head height, drop-off, stairs, trip hazards  │
 │ 4. Tracking              moving objects, velocity, time-to-collision  │
 │ 5. Visual odometry       pose from depth + colour; OpenVINS VIO       │
 │                          (D455 IMU, own thread) in shadow mode        │
 │ 6. Local map             occupancy, drop-offs, classes, ray clearing  │
 │ 7. Planner               cost map + Incremental D* Lite, safe path    │
 │ 8. Sensor health         blocked / frozen / lag / frame gaps          │
 │ 9. SAFETY VALIDATOR      one verdict: GO / CAUTION / HALT, mayMove    │
 │10. Guidance              words the verdict + path as one instruction  │
 │11. Render                camera / depth / segmentation / 3D view      │
 └──────────────────────────────┬────────────────────────────────────────┘
                                │ telemetry JSON (10 Hz)
                                ▼
 ┌──── Android (Kotlin) ────┐        ┌──── Flutter (Dart) UI ────┐
 │ VoiceGuide: TTS + haptics │ ─────▶ │ banners, 3D view, legend, │
 │ (Sailens output)          │        │ START/STOP, voice, tests  │
 │ ValidationLogger (CSV)    │        └───────────────────────────┘
 └──────────────────────────┘
```

Key numbers (CPH2745): ~30 fps end-to-end, ~35–50 ms camera-to-screen, map 2–4 ms/frame,
planner ~1–2 ms on average (incremental D\* Lite; PC replay 0.4–2.3 ms, occasional full
re-search ~10–15 ms) at 5 Hz, 3D view redrawn at 15 Hz.

**D455 IMU.** Gyro and accel run at ~400 Hz together with 30 fps video. Two fixes in the
bundled librealsense 2.50 made this work on Android: the motion sensor is switched on even
though it stalls the HID GET_REPORT request, and the D455's 38-byte reports (32-bit axes,
newer firmware) are decoded. Gravity is matched to each depth frame's capture time (~1 ms),
rotated into depth-camera axes and fine-aligned to the floor; it is used only when its
magnitude is plausible (8–11.6 m/s²). A watchdog restarts the stream once when the IMU stops
delivering, then falls back to the phone IMU (badge: "IMU d455 · 400 Hz", "NO DATA" or
"IMU phone (D455 failed)").

**VIO (in progress).** OpenVINS (MSCKF, mono + IMU) runs on its own thread in shadow mode:
its pose is reported (telemetry `vio`, log `VIO,…`) but the map and planner still use VO.
Off unless `debug.hsv2.vio 1`. Settings in `tools/vio/config` (same as the PC replay).

## Safety design

The **safety validator** (`safety_validator.cpp`) is the single authority on whether the
user may be told to move. Every frame it runs all checks independently:

| Check | Source |
|---|---|
| Sensor health | `sensor_health.cpp` — depth < 10 % valid 0.5 s (blocked), same frame 0.5 s (frozen), lag > 300 ms |
| Object directly ahead < 1 m | depth corridor (body / head / low / trip) + detected objects whose box overlaps the walking corridor (±0.45 m) at their distance; released only after 0.5 s with nothing within 1.2 m. The cause (`obstacle`, `head`, `low`, `trip`, `det:<class>`) is logged |
| Drop-off, stairs, head height, obstacle, floor | Layer 1 safety (`safety.cpp`) |
| Moving objects | tracker time-to-collision; an alert is held 0.5 s (no flicker), a higher level takes over at once |
| Path clearance | planned path not blocked, and its next 0.5–3 m keep ≥ 0.22 m (the planner's body half-width) from obstacles; the far part is re-planned as the user walks |

It returns one decision — **verdict** (GO / CAUTION / HALT), the check that decided it, all
failed checks, `pathVerified` and **`mayMove`**. The banner, vibration and voice all follow
the same decision, so they cannot disagree. Guidance only *words* the decision and, as a
second guard, turns any walking instruction into "Wait" without `mayMove`.

Tested by `native_tests/test_validator.cpp`: 30 rule checks plus 100 000 random frames
(with single-frame alert flicker) — **never** a walking instruction unless every check passes.

Camera loss outside a frame is covered too: no frames for 1 s → "STOP! Camera lost."; the
processing loop hanging → telemetry older than 0.7 s → "STOP! Camera stalled."; USB unplugged
while running → camera-lost vibration and a full red STOP screen.

## Voice and vibration

Voice and haptics use the **Sailens** output components (TTS with audio focus, strict-priority
interruption, expiry; accessibility-usage vibration), see
[third-party](#third-party-code-and-licences). HearSight's policy (`VoiceGuide.kt`):

- each instruction is spoken **once when it changes**; repeats wait for a cooldown;
- "don't move" messages (Stop, Stop way left/right, No clear way, Obstacle ahead, camera
  problem) are one group, spoken when it starts and **repeated every 5 s** while it lasts;
- more urgent messages interrupt speech in progress; speech older than 1.5 s is dropped;
- language: **English**, **தமிழ்**, or **தமிழ் + English** (Tamil voice, then English voice).
  Tamil needs a Tamil voice in the phone's TTS (Settings → Text-to-speech → Google →
  Install voice data → Tamil); without it English is spoken.
- the Tamil wording was reviewed and chosen by a Tamil speaker (e.g. "இடது / வலது பக்கம்",
  "நில்லுங்கள்! நேராக முன்னால் தடை உள்ளது.").

Vibration: STOP = three fast pulses; caution = one long; left = short pulses; right = long
pulse; camera lost = two bursts of three; start = two short; stop = one.

## Project layout

```
lib/                          Flutter UI
  main.dart
  providers/app_state.dart    state, telemetry parsing, commands, voice + test settings
  screens/home_screen.dart    camera / 3D view, banners, controls, test list
  models/models.dart          telemetry model
android/app/src/main/
  kotlin/com/hsv2/hsv2_lite/
    MainActivity.kt           USB / RealSense, JNI bridge, telemetry polling, commands
    PhoneImu.kt               phone gyro / accelerometer
    VoiceGuide.kt             voice + haptic policy, situation sentences (English / Tamil)
    SceneDescriber.kt         Gemma 4 E2B scene description (LiteRT-LM)
    ValidationLogger.kt       Phase 7 per-walk CSV recorder
  kotlin/com/sailens/         Sailens output components (Apache-2.0, see third_party/)
  cpp/
    native-lib.cpp            stream thread: wires every stage below, telemetry JSON
    qnn_runtime.*             QNN HTP runtime (context binaries)
    yolo_detector.*           YOLOv8s / YOLOv8s-seg on the NPU
    semseg.*                  DeepLab ADE20K segmentation on the NPU
    nav_classes.h             navigation classes (wall, floor, door, stairs, person, …)
    safety.*                  Layer 1 safety: floor, corridor, obstacles, drop-offs, stairs
    tracker.*                 moving objects, velocity, time-to-collision
    vo.*, rigid.*             visual odometry
    imu.*, imu_sync.*, gravity_calib.*   D455 IMU (samples, frame-time gravity), phone IMU sync,
                              gravity calibration (phone -> camera, D455 fine alignment)
    vio_thread.*, vio_calib.h OpenVINS VIO thread (shadow mode), camera/IMU calibration file
    local_map.*               dynamic local map
    nav_plan.*                cost map + incremental D* Lite planner
    sensor_health.*           camera trust checks
    safety_validator.*        the single safety authority
    guidance.*                instruction wording
    view3d.*                  CPU 3D view + navigation map inset
    recorder.*                walk recording (.hsr v4: grey, depth, segmentation, raw D455 IMU)
  assets/qnn/                 NPU models (context binaries, v73 and v81)
native_tests/                 PC tests and replay tools (no phone needed)
tools/validate_walks.py       Phase 7 walk scorer
tools/deeplab/                DeepLab ADE20K -> QNN conversion (scripts + steps)
tools/vio/                    OpenVINS: HearSight config, PC replay (replay_vio), Android build
tools/vlm/                    scene-description tests on recorded snapshots (PC)
docs/phase7-validation.md     validation protocol
third_party/sailens/          Sailens licence, notice and provenance
```

## Building

Requirements (versions used):

| Tool | Version / location |
|---|---|
| Flutter | `/home/naveen/flutter_new` (Dart 3.13; the older `flutter` on PATH is too old) |
| Android NDK | 28.2.13676358 |
| Qualcomm QNN / QAIRT SDK | 2.47.0.260601 at `/home/naveen/Downloads/qairt/2.47.0.260601` (`QNN_SDK_ROOT` in `android/app/src/main/cpp/CMakeLists.txt`) |
| librealsense | source in `android/app/src/main/cpp/librealsense` (built with the app) |
| JDK | 17 |

The NPU model files (`android/app/src/main/assets/qnn/*.bin`, ~80 MB) are stored in
**Git LFS** — install it before cloning, or the files arrive as small pointer files:

```bash
sudo apt install git-lfs && git lfs install
git clone https://github.com/rnaveen1236/Hearsight.git      # models download with the clone
git lfs pull                                                # if cloned before installing git-lfs
```

The QNN runtime libraries are not in git; copy them from the QNN SDK into
`android/app/src/main/jniLibs/arm64-v8a/`:

```
libQnnHtp.so  libQnnHtpPrepare.so  libQnnSystem.so
libQnnHtpV73Skel.so  libQnnHtpV73Stub.so  libQnnHtpV81Skel.so  libQnnHtpV81Stub.so
```

Build the release APK (arm64 only):

```bash
/home/naveen/flutter_new/bin/flutter build apk --release --target-platform android-arm64
```

Output: `build/app/outputs/flutter-apk/app-release.apk` (~118 MB, mostly the NPU runtime and
models). The build uses `-Werror=format`: a printf argument mismatch fails the build (it once
crashed the phone).

## Installing and running

```bash
adb install -r build/app/outputs/flutter-apk/app-release.apk          # USB: ~10 s
adb connect <phone-ip>:<port>                                          # wireless debugging
adb -s <phone-ip>:<port> install -r build/app/outputs/flutter-apk/app-release.apk   # Wi-Fi: ~10 min
```

Over Wi-Fi a fresh `adb disconnect` + `adb connect` before installing helps; verify an install
by comparing `md5sum` of the APK with the installed `base.apk`. The wireless-debugging port
changes every time it is re-enabled. OnePlus phones may show an "Install via USB" prompt.

On the phone: connect the D455, allow USB access, tap **START**. Controls (bottom panel):
view switch (camera / mix / segmentation / 3D), record walk, START / STOP; second row: voice
on / off, language (English → தமிழ் → தமிழ் + English), validation tests (✓).

## Testing

PC tests — synthetic scenes and simulated walks, no phone needed:

```bash
bash native_tests/run_tests.sh
```

| Test | Covers |
|---|---|
| test_safety, test_walk, test_walkstep, test_semantic, test_fixes | Layer 1 safety, walking scenarios, semantic false alarms |
| test_gravity, test_vo | gravity calibration, visual odometry |
| test_map, test_trip, test_plan | local map, trip hazards, planner (doorway, stability, D\* Lite incremental == full search, side kept at an obstacle) |
| test_guidance | 21 guidance checks (instructions, hysteresis, hold time) |
| test_health | 13 sensor-health checks |
| test_validator | 30 validator rules + 100 000 random frames (safety invariant) |
| test_tracker | moving-object tracker |
| test_recorder | recording format v4 (segmentation and D455 IMU samples written and read back) |

Offline replay of recorded walks (`.hsr`, made with the record button):
`native_tests/replay_map.cpp` (safety, map, planner, 3D view snapshots) and `replay_vo.cpp`.
Recordings hold half-res grey + depth, the phone's odometry and IMU, (v3) the phone's scene
segmentation per frame, which the replay feeds to the safety layer and the map exactly as on
the phone, and (v4) every raw D455 gyro / accel sample. Older recordings still replay.

VIO replay (OpenVINS on the PC, v4 recordings): build OpenVINS without ROS and the tool as
described in `tools/vio/CMakeLists.txt`, then

```bash
replay_vio walk.hsr outdir      # time to initialise, VIO vs VO path, ms per frame, vio_traj.csv
```

## Validation walks (Phase 7)

Measured walk tests with pass / fail rules — see [docs/phase7-validation.md](docs/phase7-validation.md).

1. Tap **✓**, pick a scenario (S1 wall … S9 chair), walk, tap the amber banner to finish.
   The app writes a CSV (10 rows / s + every spoken message) on the phone.
2. Score on the PC:

   ```bash
   python3 tools/validate_walks.py --pull --device <serial-or-ip:port> --out validation_runs
   ```

   → `validation_runs/validation_report.md`, PASS / FAIL per walk and per check.

Every walk is checked for the safety invariant (no walking advice without the validator's
permission) and ≥ 20 fps, plus its scenario's rules (e.g. S1: STOP at ≥ 0.7 m from the wall,
spoken within 0.6 s).

## Debug switches

Set with `adb shell setprop <name> <value>`, then restart the stream:

| Property | Effect |
|---|---|
| `debug.hsv2.timing 1` | per-frame timing / safety logs (off by default: they fill the log quota) |
| `debug.hsv2.imu 0` | disable the D455 IMU (on by default; phone IMU is used instead) |
| `debug.hsv2.gyro_hz` / `debug.hsv2.accel_hz` | D455 IMU rates (default: highest, ~400 Hz) |
| `debug.hsv2.vio 1` | OpenVINS VIO in shadow mode (needs the D455 IMU) |
| `debug.hsv2.map 0` | disable the local map |
| `debug.hsv2.vo 0` | disable visual odometry |
| `debug.hsv2.semseg` | segmentation on / off |
| `debug.hsv2.model` | choose the detection model |
| `debug.hsv2.fps` | camera frame rate |
| `debug.hsv2.imu_offset_ms` | camera → IMU time offset |
| `debug.hsv2.globaltime` | RealSense global timestamps |

Useful log tags: `HSV2Native` (`VALID`, `GUIDE`, `HEALTH`, `MAP`/`PLAN`, `D455IMU` and
`VIO` every 10 s), `HSV2Voice` (`SAY`), `HSV2Valid`, `HSV2Lite` (`SCENE`), `HSV2_IMU`, `HSV2_VIO`.

## Development phases

| Phase | Content | Branch |
|---|---|---|
| NPU | YOLOv8s INT8 on the Hexagon NPU, depth labels | `feature/npu-yolo-imu` |
| 1 | Reactive safety: floor, corridor, obstacles, drop-offs, stall recovery | `phase1-safety` |
| 2 | Perception: person masks, tracking, velocity, time-to-collision | `phase2-perception` |
| — | Semantic segmentation (DeepLab ADE20K) in safety | `phase-semseg` |
| 3 | Visual odometry + phone IMU (sync, gravity calibration) | `phase3-vo`, `phase-imu` |
| 4 | Dynamic local map, drift correction | `phase4-map` |
| 4b | 3D view, planner, navigation map inset | `phase4b-3d` |
| 5 | Walking guidance | `phase5-guidance` |
| — | Sensor health; Phase 6 voice + vibration (English / Tamil); accessibility | `sensor-health` |
| — | Safety validator (single authority) | `safety-validator` |
| 7 | Validation: test mode, walk recorder, scorer | `phase7-validation` |
| — | D\* Lite planner, glossy floors, start-up warm-up, situation sentences, Gemma scene description | `dstar-lite`, `fix-glossy-floor-wip`, `scene-gemma` |
| — | D455 IMU (HID fix, 38-byte decoder, watchdog), recordings v4, OpenVINS VIO (shadow mode) | `feature/d455-imu-direct` |
| 8 | Outdoor use | planned |

All phases are merged into `main`.

## Known limitations

- **Mount**: the camera tilts ~12–25°; this causes some false STOP / drop-off warnings. A
  rigid, level mount is needed before the full Phase 7 run.
- **D455 IMU**: this D455 has no factory IMU calibration (default intrinsics); the floor fine
  alignment corrects the mounting error once it locks. Some starts still stalled the camera;
  the watchdog then restarts once and falls back to the phone IMU.
- **VIO** runs in shadow mode only: not yet tuned on real walks (needs v4 recordings), not
  yet driving the map. Start-up (static initialisation) is the known risk.
- Distances are the app's own depth measurements; validation thresholds are not yet checked
  against tape-measured ground truth.
- Indoor-tuned; outdoor use (Phase 8) is not done.
- Wireless debugging drops often during walks; turn on "Stay awake" in Developer options or
  use USB for installs and logs.

## Third-party code and licences

- **Sailens Android** (Apache-2.0) — `com.sailens.output` (SpeechManager, Announcement,
  SpeechOwner, HapticPlayer, AccessibilityStatusProvider) and `com.sailens.core.log.LogService`,
  copied from https://github.com/wnbotoo/sailens-android; SpeechManager modified for an
  app-chosen language and multi-voice announcements. Licence, notice and provenance in
  `third_party/sailens/`. HearSight's cooldown policy follows Sailens' CooldownManager rules
  (re-implemented).
- **Intel RealSense SDK (librealsense)** — Apache-2.0; patched for the D455 IMU on Android
  (`src/hid/hid-device.cpp`, `src/usbhost/handle-usbhost.h`).
- **OpenVINS** (rpng/open_vins) — **GPL-3.0**, built into the app (VIO). Distributing the app
  therefore means publishing HearSight's source under GPL-3.0. Source tree outside the repo
  (`OPENVINS_SRC`, see `tools/vio`); `tools/vio/android/compat` maps `boost::filesystem` to
  `std::filesystem`.
- **OpenCV** (Android SDK, static) — Apache-2.0; Eigen — MPL-2.0; Boost headers — BSL-1.0.
- **Gemma 4 E2B** (Google, Gemma terms of use) via **LiteRT-LM** (Apache-2.0); the model file
  is copied to the phone separately, not stored in git.
- DeepLab conversion scripts: see [tools/deeplab/README.md](tools/deeplab/README.md).
- **Qualcomm QNN / QAIRT** runtime — Qualcomm licence; libraries not redistributed in git.
- **YOLOv8** (Ultralytics, AGPL-3.0) and **DeepLab** (ADE20K) model weights (stored in Git
  LFS) — check their licences before any distribution.
- **Sago Voice Assistant** (Apache-2.0 / MIT) — Integrated from https://github.com/TringyTrongy/argus
  providing offline speech recognition, Moonshine/Parakeet ASR, Silero VAD, AudioRoute management,
  and offline voice command processing (`com.hsv2.hsv2_lite.voice`).
- **sherpa-onnx** (Apache-2.0) — Next-gen Kaldi offline speech recognition engine for Android
  (`third_party/maven/com/k2fsa/sherpa/onnx/`).
