# 05 — Benchmark

All phone numbers are from the **OnePlus CPH2745 (Snapdragon SM8850, Hexagon v81)** unless noted.

## 1. AI models on the NPU (measured 22 Oct 2026)

| Model | Input | Size | Load | **NPU time / frame** | Call incl. transfer | Throughput* |
|---|---|---|---|---|---|---|
| YOLOv8s INT8 | 640×640 | 12 MB | 29 ms | **3.0 ms** | 3.9 ms | — |
| YOLOv8s-seg INT8 | 640×640 | 12 MB | 32 ms | **4.3 ms** | 5.4 ms | — |
| DeepLabV3+ INT8 (ADE20K) | 288×384 | 16 MB | 29 ms | **12.9 ms** | 14.3 ms | 61 inf/s |

\*Throughput as reported by `qnn-net-run` including writing every output to storage; the YOLO
runs wrote large outputs (≈ 0.7–1.5 MB each), so their figure (7–17 inf/s) measures storage,
not the NPU, and is left out.

**Method:** QAIRT 2.47 `qnn-net-run` on the phone, the app's own context binaries, 100 runs per
model with 8-bit test input, `--profiling_level basic`, default NPU power mode (as the app uses),
8 HVX threads; averages from `qnn-profile-viewer`.

**Reading:** YOLO + DeepLab together use about **16 ms of NPU time per frame**, inside the 33 ms
budget at 30 fps; DeepLab runs on its own worker thread. The app's own log showed 7–17 ms
inference per frame, consistent with these numbers plus pre- / post-processing.

## 2. On-phone language model

| Model | Runtime | Size | First answer | **Later answers** |
|---|---|---|---|---|
| Gemma 4 E2B-it | LiteRT-LM, GPU | 2.6 GB | 7.8 s (warm-up) | **1.03–1.27 s, avg ≈ 1.12 s** (9 answers) |

**Method:** the app's scene-description test (10 recorded snapshots, `scene_test.csv`, 22 Oct).
Live narration in the phone log the same day: 1.0–1.2 s per comment.

## 3. Pipeline (CPU)

| Stage | Number | Source |
|---|---|---|
| Whole pipeline | **~30 fps**, camera-to-screen **35–50 ms** | Phone (README key numbers) |
| Per-frame processing p99 | ~24 ms (33 ms budget) | Phone timing logs |
| Local map update | 2–4 ms / frame | Phone |
| **D\* Lite planner** | **avg 0.4–2.3 ms**, worst ~15 ms (full re-search), at 5 Hz | PC replay of 3 recorded walks |
| Planner vs old Dijkstra | same paths (hall walk 79 / 135 ok, both), avg 2.3 vs 3.0 ms and 0.4 vs 1.8 ms | PC replay |
| D455 IMU | gyro ~399 Hz, accel ~404 Hz; frame match 0.0–1.3 ms; newest sample 0–3 ms old | Phone log |
| 3D view | redrawn at 15 Hz | Phone |
| Telemetry to UI | 10 Hz | Phone |

## 4. Quality and safety checks

| Check | Result |
|---|---|
| PC test groups | 12 groups, 0 failures (`native_tests/run_tests.sh`) |
| Safety invariant | 100,000 random frames: no walking advice without the validator's permission |
| Glossy-floor fix on recordings | false drop-off STOP frames 6 → 0 (Oct 18 walk), drop sightings 34 → 24 (hall) |
| Start-up warm-up | first-frame false "STOP 0.33 m" (11 % depth) → "Wait, checking the way" |
| Side keeping | blocked-plan-in-between test: old planner moved goal 1.9 m and flipped side; new keeps both |
| D455 IMU decode | level camera P −0.0°, R −1.2°; \|a\| 9.72–9.78 m/s²; steady frames 0 → 56–163 per 10 s |

## 5. Not measured yet

| Item | Needed |
|---|---|
| YOLO accuracy (mAP) of the INT8 phone model | COCO validation set (download) |
| DeepLab accuracy (mIoU) of the INT8 model | `tools/deeplab/scripts/eval.py` on ADE20K images |
| v73 models (CPH2729 / SM6650) | That phone connected |
| OpenVINS VIO: time per frame, CPU load, drift vs VO | A v4 recorded walk + a phone run with `debug.hsv2.vio 1` |
| Battery and temperature over a 30 min walk | Sustained test (the September test files are empty) |
| Distances vs tape-measured ground truth | Phase 7 walks with floor marks |
