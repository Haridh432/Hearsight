# 01 — Overview

## The problem

A white cane finds what is on the ground within reach; a guide dog avoids obstacles but does
not explain the scene. Neither warns about head-height obstacles, drop-offs a step away, or a
person walking straight at you — and neither tells you *which way* is free. HearSight adds that
information as short spoken instructions, without replacing the cane.

## Who it is for

Blind and low-vision people walking indoors (corridors, halls, offices, homes). Outdoor use is
planned (Phase 8). The user wears a head mount with the camera and phone; no screen interaction
is needed while walking (large START / STOP, TalkBack labels, spoken start / stop).

## What it does

| Capability | Behaviour |
|---|---|
| **Stop in time** | Anything directly ahead within **0.5 m** → "STOP! Chair directly ahead." — then a way round it: "Stopped, a chair blocks, way on the left." |
| **Find the way** | Plans a safe path 2.5–5 m ahead (incremental D\* Lite), kept away from walls, passable through doorways, re-planned as people move; the way round an obstacle keeps its side |
| **Hazards** | Head-height obstacles, low objects, trip hazards (3–10 cm), drop-offs, steps and stairs; drops judged against the floor tracked step by step (glossy floors and mirror images do not trigger) |
| **Moving objects** | People and vehicles tracked with velocity and time-to-collision |
| **Situation sentences** | Object + distance + direction, the same on screen and in the voice: "Person 2 metres ahead, path goes left.", "Nothing within 3 metres.", "Door 3 metres ahead" |
| **Languages and feedback** | English, Tamil or both (each in its own voice); urgent messages interrupt; vibration patterns for STOP, left / right, camera lost; works in silent mode |
| **Scene description (optional)** | On-phone Gemma 4 E2B: "What's ahead?" on request and short ✨ Live comments on events; filtered so it can never contradict the safety decision |
| **Sensor health** | Camera covered / frozen / lagging / unplugged → "STOP! Camera …"; 1 s start-up warm-up ("Wait, checking the way") |
| **3D view** | Live 3D map with class colours and a top-down navigation inset for a sighted helper; path green when verified, dashed amber while waiting |

## Hardware

| Part | Used |
|---|---|
| Depth camera | Intel RealSense **D455**: colour 640×480 + depth 640×480 at 30 fps, IMU (gyro + accel) ~400 Hz, over USB-C |
| Phone | OnePlus **CPH2745** (Snapdragon **SM8850**, Hexagon **v81** NPU) — main target; CPH2729 (SM6650, v73) also supported |
| Android | 12+ (minSdk 31), tested on Android 16 |
| Mount | Head mount holding camera and phone, camera level |

## Design principles

1. **Safety first, one authority** — a single safety validator decides GO / CAUTION / STOP every
   frame; guidance, banner, voice and vibration only word that decision, so they never disagree.
2. **On-device** — every model runs on the phone (NPU / GPU); works offline; no images leave the phone.
3. **One clear instruction at a time** — situation sentences, no chatter, urgent messages first.
4. **Measured, not guessed** — synthetic tests, random stress tests, recorded walks replayed on the
   PC, and a scored field protocol (Phase 7).

## Status (22 Oct 2026)

| Area | State |
|---|---|
| Perception on the NPU, safety layer, 3D map, D\* Lite planner, guidance, voice (EN / TA), vibration | Done |
| Scene description (Gemma), sensor health, validation test mode, walk recorder + PC replay | Done |
| D455 IMU (gravity for floor / drop-off checks, ~1 ms frame sync, watchdog with phone fallback) | Done |
| Visual-inertial odometry (OpenVINS) | In progress — runs on the phone in shadow mode; tuning needs recorded walks |
| Phase 7 field validation (9 scenarios × 3 walks) | Pending — needs a rigid, level mount |
| Outdoor use (Phase 8), user pilot | Planned |

## Known limitations

- Camera mount tilt (~12–25°) causes some false STOP / drop-off warnings.
- The D455 has no factory IMU calibration; the floor fine alignment corrects it once it locks.
- Distances are the app's own depth measurements, not tape-measured ground truth.
- Indoor-tuned; outdoor is not done.
- OpenVINS is GPL-3.0: distributing the app means publishing its source under GPL-3.0.
