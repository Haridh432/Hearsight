# HearSight — Client Presentation (6 slides)

Palette suggestion: deep teal `0F3B40` (dominant), teal `0E7C7B`, amber `F2A541` (accent, matches the app's STOP / wait cues), white background on content slides.
Fonts: Cambria (titles) + Calibri (body).

---

## Slide 1 — Title (dark background)

**HearSight**
*A head-worn navigation assistant for blind and low-vision people*

- Depth camera + smartphone, everything runs on the phone — no cloud, no internet
- Warns about obstacles, drop-offs and moving people, and guides around them by voice
- English and Tamil

Visual: photo of the head mount (Intel RealSense D455 + phone), or a phone screenshot with the 3D view and green path.

> Speaker notes: HearSight turns a depth camera and a phone into a walking assistant. It sees the space in front of the user, decides what is safe, and speaks one clear instruction at a time. All processing is on the phone, so it works without a network and no images leave the device.

---

## Slide 2 — What it does

Six capability cards (icon in a teal circle + bold header + one line):

| Capability | One line |
|---|---|
| **Stops in time** | Anything directly ahead within 0.5 m → "STOP! Chair directly ahead." — then a new way round it |
| **Finds the way** | Plans a safe path 2.5–5 m ahead, away from walls, through doorways, re-planned as people move |
| **Sees hazards** | Drop-offs, steps and stairs, head-height obstacles, low objects and trip hazards |
| **Tracks people** | Moving people and vehicles with speed and time-to-collision |
| **Speaks clearly** | Live situation sentences: "Chair 2 metres ahead, path goes right." — English / Tamil, with vibration cues |
| **Describes the scene** | Optional on-phone AI (Gemma): "What's ahead?" and short live comments |

> Speaker notes: The core promise is simple — stop the user before contact, then show the way around. Every instruction says what is there, how far, and which way to go. The scene description is an extra: it adds context but never overrides the safety decision.

---

## Slide 3 — How it works (on-device pipeline)

Process flow (6 steps, left to right, arrows):

1. **Sense** — D455 colour + depth at 30 fps, IMU at 400 Hz
2. **Understand** — object detection (YOLOv8s) + scene segmentation (DeepLabV3+) on the phone's AI chip (NPU)
3. **Map** — 16 × 16 m local 3D map of floor, obstacles and drop-offs; position from visual odometry
4. **Plan** — incremental D* Lite path planner
5. **Validate** — one safety authority decides: GO / CAUTION / STOP
6. **Speak** — one instruction at a time, voice + vibration

Hardware strip below the flow:

- **Intel RealSense D455** — depth + colour + motion sensor
- **OnePlus phone, Snapdragon SM8850** — Hexagon NPU for the AI models
- **Head mount** — camera and phone together, hands free

Callout: *100 % on-device — works offline, private by design.*

> Speaker notes: Each frame goes through the same chain. The AI models run on the phone's neural processor, which keeps the main processor free for mapping and planning. Before anything is spoken, a single safety validator checks every rule; guidance only words its decision, so screen, voice and vibration can never disagree.

---

## Slide 4 — Performance (measured on the phone)

Left: bar chart — AI model time per frame on the NPU (ms), CPH2745 / SM8850:

| Model | Task | ms per frame |
|---|---|---|
| YOLOv8s (INT8) | Object detection | 3.0 |
| YOLOv8s-seg (INT8) | Detection + person masks | 4.3 |
| DeepLabV3+ (INT8) | Floor, wall, door, stairs | 12.9 |

Right: four big-number callouts:

- **30 fps** — full pipeline, camera to screen in **35–50 ms**
- **~16 ms** — all AI vision models per frame on the NPU (budget: 33 ms)
- **~1–2 ms** — path re-planning on average (D* Lite)
- **~1.1 s** — on-phone scene description (Gemma 4 E2B, after a one-time 8 s warm-up)

Footnote: NPU times from 100 runs per model with Qualcomm's profiler; app models are 12–16 MB each.

> Speaker notes: The whole vision stack fits comfortably inside the frame budget: about 16 milliseconds of neural-network time per frame, leaving headroom. Re-planning is fast enough to react to people walking past. The language model is slower, which is why it only adds comments and never drives safety.

---

## Slide 5 — Safety by design

Left — safety rules (icon rows):

- **Never "go" without a verified path** — walking advice only when the path ahead keeps ≥ 0.22 m from obstacles
- **STOP close, then re-route** — STOP only for something within 0.5 m, then "Stop, way is left / right"
- **Trust the sensors, or stop** — camera covered, frozen, lagging or unplugged → "STOP! Camera …"
- **Calm start** — 1 s warm-up ("Wait, checking the way") instead of a false first-frame STOP
- **AI cannot override** — scene descriptions are filtered: no "go ahead" while the path is blocked

Right — how it is tested (stat cards):

- **12** automated test groups on the PC (safety, planner, map, guidance, sensors)
- **100,000** random frames checked against the safety invariant
- **Recorded walks** replayed offline to test every fix on real data
- **Phase 7 field protocol** — 9 scenarios (wall, corridor, doorway, stairs, people, chair …) × 3 walks, scored PASS / FAIL

> Speaker notes: Safety is not spread across features — one validator makes every decision, and it is tested with synthetic scenes, random-frame stress tests and real recorded walks. The field protocol measures, for example, that a STOP at a wall comes in time and is spoken within 0.6 seconds.

---

## Slide 6 — Status and roadmap (dark background)

Three-column timeline:

**Done**
- On-phone perception (NPU), safety layer, 3D map, path planner
- Voice + vibration guidance in English and Tamil
- Scene description (Gemma), sensor health, test mode and walk recorder
- D455 motion sensor working: gravity for floor and drop-off checks

**In progress**
- Visual-inertial odometry (OpenVINS): steadier position on blank walls, glossy floors and fast head turns — running in test mode
- Field validation walks (9 scenarios × 3)

**Next**
- Rigid, level head mount
- Outdoor use (Phase 8)
- Pilot with blind and low-vision users

Closing line: *Ask: pilot partners and user testers.*

> Speaker notes: The indoor system is complete and running on the phone. The current work makes position tracking more robust, then we validate in structured field walks. Next we move outdoors and into a pilot with real users — that is where we would like your involvement.
