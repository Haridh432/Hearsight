# 04 — Tech stack

## Languages and frameworks

| Layer | Technology | Version |
|---|---|---|
| UI | **Flutter** / Dart, `provider` state management, `http` | Flutter 3.47.3, Dart SDK ^3.12.2, provider ^6.1.5, http ^1.6.0 |
| Android | **Kotlin**, kotlinx-coroutines | coroutines 1.9.0; minSdk 31, tested on Android 16 |
| Native | **C++17** via JNI, CMake | Android NDK 28.2.13676358 (clang), CMake 3.22 |
| Build | Gradle (Kotlin DSL), Flutter tool | `flutter build apk --release --target-platform android-arm64` |

## Native libraries

| Library | Use | Version | Licence |
|---|---|---|---|
| **Intel RealSense SDK (librealsense)** | D455 colour, depth, IMU over USB (built from source, rsusb backend) | 2.50.0, patched HID for the D455 IMU | Apache-2.0 |
| **Qualcomm QNN / QAIRT** (HTP backend) | Runs the AI models on the Hexagon NPU | 2.47.0 | Qualcomm licence (not in git) |
| **OpenVINS** (ov_core, ov_init static, ov_msckf) | Visual-inertial odometry (MSCKF) | upstream master, 2025-11 | **GPL-3.0** |
| **OpenCV** (Android SDK, static) | Image ops for OpenVINS (KLT, features) | 4.x Android SDK | Apache-2.0 |
| **Eigen** | Linear algebra (OpenVINS) | 3.4 | MPL-2.0 |
| **Boost** (headers only) | posix_time, chi-squared (OpenVINS); `boost::filesystem` mapped to `std::filesystem` | 1.7x headers | BSL-1.0 |
| HearSight native code | Safety, map, planner (D\* Lite), VO, tracker, validator, guidance, 3D view, recorder | — | project |

## Android libraries

| Library | Use | Licence |
|---|---|---|
| **LiteRT-LM** (`com.google.ai.edge.litertlm:litertlm-android:0.18.0`) | Runs Gemma on the phone GPU | Apache-2.0 |
| **Sailens Android** output components | SpeechManager, Announcement, HapticPlayer, accessibility status | Apache-2.0 (copied, see `third_party/sailens/`) |
| Android TextToSpeech | English and Tamil voices | Android |

## AI models

| Model | Task | Format / precision | Input → output | Size | Runs on |
|---|---|---|---|---|---|
| **YOLOv8s** (Ultralytics) | Object detection, 80 COCO classes | QNN context binary, INT8 (v81 + v73) | 640×640 RGB → 80 × 8400 scores + boxes | 12 MB | NPU |
| **YOLOv8s-seg** | Detection + person masks | QNN, INT8 | 640×640 → boxes, scores, 32 mask coefficients, 160×160×32 protos | 12 MB | NPU |
| **DeepLabV3+ MobileNetV2** (MMSegmentation, ADE20K) | Scene segmentation, 150 classes → navigation classes | QNN, INT8 | 288×384 RGB → 72×96 class map | 16 MB | NPU |
| **Gemma 4 E2B-it** (Google) | Scene description ("What's ahead?", ✨ Live) | `.litertlm` | colour frame + sensor facts → one sentence | 2.6 GB (on the phone, not in git) | GPU |

Model files for the NPU are stored in Git LFS (`android/app/src/main/assets/qnn/`). Conversion
steps: [tools/deeplab](../../tools/deeplab/README.md). Licences: YOLOv8 AGPL-3.0, DeepLab /
ADE20K own terms, Gemma terms of use — check before distribution.

## Algorithms (own implementation)

| Area | Method |
|---|---|
| Floor and safety | Gravity-aided floor plane fit, walking-corridor bins, drop-offs vs the locally tracked floor, reflection rejection with segmentation |
| Visual odometry | KLT feature tracking on half-res grey + depth back-projection, RANSAC rigid motion, plausibility gate (≤ 3 m/s, ≤ 6 rad/s) |
| Local map | 2.5D log-odds occupancy, 10 cm cells, 16 × 16 m, ray clearing, class histograms, recentring |
| Planning | Cost map (inflation, wall preference, unknown cost) + **incremental D\* Lite** with a virtual goal over 2.5–5 m candidates, goal memory, string pulling, Chaikin smoothing |
| Tracking | Moving-object velocity and time-to-collision with ego-motion compensation |
| Calibration | Horn's method for phone → camera and D455 → floor rotation from gravity pairs |

## Tools and tests

| Tool | Purpose |
|---|---|
| `native_tests/run_tests.sh` | 12 PC test groups (safety, walk, semantic, gravity, VO, map, trip, plan, recorder, guidance, health, validator, tracker) |
| `native_tests/replay_map`, `replay_vo` | Replay recorded walks through safety, map, planner, VO |
| `tools/vio/replay_vio` | OpenVINS replay of v4 recordings (PC build without ROS) |
| `tools/validate_walks.py` | Phase 7 walk scorer |
| `tools/deeplab/` | DeepLab → QNN conversion |
| `tools/vlm/describe_test.py` | Scene-description model tests (PC) |
| QAIRT `qnn-net-run`, `qnn-profile-viewer` | NPU benchmarks |
| `adb` (USB / wireless debugging) | Install, logs, recordings |

## Hardware stack

Intel RealSense D455 → USB-C → OnePlus CPH2745 (Snapdragon SM8850, Hexagon v81 NPU; GPU for
Gemma), head mount. Second supported phone: CPH2729 (SM6650, Hexagon v73).
