# 03 — Flow

## 1. Per frame (stream thread, 30 fps)

```mermaid
flowchart TD
    A["D455 frameset<br/>colour 640×480 RGBA + depth 640×480"] --> B["Sensor health<br/>(depth coverage, frozen, lag, warm-up)"]
    A --> C["YOLOv8s on the NPU<br/>objects + person masks"]
    A --> D["Latest DeepLab result<br/>(segmentation worker)"]
    IMU["D455 IMU samples (~400 Hz)"] --> E["Gravity at the frame's capture time<br/>(phone IMU if the D455 has no data)"]
    E --> F
    C --> F["Layer 1 safety<br/>floor fit, corridor, obstacles, head height,<br/>low / trip, drop-off, stairs"]
    D --> F
    C --> G["Tracker: velocity, time-to-collision"]
    A --> H["VO worker → pose"]
    A --> HV["VIO worker (OpenVINS, shadow)"]
    IMU --> HV
    H --> I["Local map update<br/>occupancy, drops, classes"]
    F --> I
    I --> J["Cost map + D* Lite plan (5 Hz)"]
    B --> K{{"Safety validator"}}
    F --> K
    G --> K
    J --> K
    K --> L["Guidance: one instruction code"]
    L --> M["Telemetry JSON"]
    K --> M
    M --> N["Kotlin: voice + vibration,<br/>validation CSV, Gemma triggers"]
    M --> O["Flutter: banners, 3D view, badge"]
```

## 2. Start-up

1. User taps **START** (or the spoken start command); Kotlin asks for USB permission if needed.
2. Native: pipeline starts colour + depth at 30 fps (`debug.hsv2.fps` to change) and the D455 IMU
   at its highest rate; models are already loaded on the NPU.
3. **Warm-up (1 s, up to 3 s while depth is sparse):** sensor health reports *warmup* → "Wait,
   checking the way", no movement advice, no STOP alarm; the 0.5 m latch is not set.
4. Floor fit, IMU gravity and map settle; the planner gets its first path.
5. VIO (if `debug.hsv2.vio 1`) starts its thread and initialises once the user stands still and
   then moves.

## 3. Object ahead → STOP → re-route

```mermaid
sequenceDiagram
    participant U as User (walking)
    participant S as Safety + detections
    participant V as Validator
    participant P as Planner
    participant G as Guidance / voice
    U->>S: chair 0.9 m ahead
    S->>V: obstacle ahead 0.9 m (beyond 0.5 m)
    V->>G: CAUTION, detour allowed if the path turns away
    G-->>U: "Chair 1 metre ahead, path goes right."
    U->>S: chair 0.45 m ahead
    S->>V: object ahead ≤ 0.5 m → latch
    V->>G: HALT (obstacle ahead)
    G-->>U: "STOP! Chair directly ahead."
    P->>V: verified path round the right side
    V->>G: HALT + verified way
    G-->>U: "Stopped, a chair blocks, way on the right."
    U->>S: turns right, nothing ≤ 0.7 m for 0.5 s
    S->>V: latch released
    V->>G: GO, mayMove
    G-->>U: "Nothing within 3 metres, path goes right."
```

The planner keeps its target through short "blocked" moments (~2 s), so the way round keeps its
side; if that side really closes, it switches.

## 4. Voice and vibration

1. Telemetry arrives every 100 ms; `VoiceGuide` reads the guide code, object ahead (class,
   distance) and the validator verdict.
2. **Situation sentence** built from code + object + distance + direction (English and Tamil).
3. **Priorities:** camera fault 100 › object ahead 95 › stop 90 › stairs 85 › no clear way 70 ›
   turns 60 › door 45 › bear 40 › straight 30 › wait 20. A strictly higher priority interrupts speech.
4. **Cooldowns and repeats:** the same instruction is not repeated within its cooldown; STOP-type
   messages repeat every 3–5 s while they last; a closer object or a new object is spoken again.
5. **Vibration** follows the same alert (STOP pattern, left / right, camera lost).
6. **Gemma (✨ Live):** on events (stopped > 2 s, door / stairs / turn, new object 0.5–3 m, open
   walking every 15 s) the latest colour frame + sensor facts go to Gemma; the answer is filtered
   (no "go / move forward" while walking is not allowed) and spoken after the instruction, at the
   lowest priority, dropped if older than ~3 s.

## 5. IMU and VIO

```mermaid
flowchart LR
    HID["D455 HID reports (38 bytes)"] --> DEC["librealsense: decode<br/>accel mg, gyro 0.1°/s"]
    DEC --> IS["ImuStreamer: raw history (2 s),<br/>low-passed gravity, rates"]
    IS -- "gravity at frame time,<br/>|g| 8–11.6 gate" --> SAF["Safety floor fit"]
    IS -- "raw samples" --> REC["Recorder v4"]
    IS -- "raw samples" --> VIO["OpenVINS thread"]
    IS -- "no data 3 s" --> WD["Watchdog: restart once,<br/>then phone IMU"]
    VIO --> TEL["Telemetry 'vio' (shadow)"]
```

## 6. Recording and offline replay

1. **Record** (button): the recorder writes `walk_<date>.hsr` on the phone — grey, depth,
   phone VO and IMU, segmentation, raw D455 IMU — plus colour snapshots (2 Hz).
2. Copy to the PC (`adb pull`).
3. Replay the same stages on the PC:
   - `native_tests/replay_map` — safety, map, planner, 3D snapshots (old vs new code side by side);
   - `native_tests/replay_vo` — visual odometry;
   - `tools/vio/replay_vio` — OpenVINS on the walk vs the phone's VO;
   - `tools/vlm/describe_test.py` — scene-description models on the snapshots.
4. Fixes are measured on the same walks before going back to the phone.

## 7. Validation walk (Phase 7)

Tap ✓ → pick a scenario (S1 wall … S9 chair) → walk → tap the amber banner → a CSV per walk on
the phone → `tools/validate_walks.py --pull` scores PASS / FAIL per check
([protocol](../phase7-validation.md)).
