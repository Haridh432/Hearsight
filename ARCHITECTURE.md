# HearSight + Argus System Architecture & Workflow

## Overview
**HearSight + Argus** is an offline assistive navigation and voice assistant system designed for blind and visually impaired individuals. It integrates **Intel RealSense D455 3D perception**, **Qualcomm Hexagon NPU AI acceleration (QNN / QAIRT)**, **Argus offline speech processing**, and an **accessible Flutter mobile interface**.

---

## 1. End-to-End Architecture Diagram

```mermaid
graph TB
    %% ==========================================
    %% 1. HARDWARE LAYER
    %% ==========================================
    subgraph HW ["1. SENSORY & HARDWARE LAYER"]
        RS["Intel RealSense D455<br/>• RGB Camera (1080p @ 30 FPS)<br/>• Stereo Depth Sensor (848x480 @ 30 FPS)<br/>• Bosch BMI085 IMU (Gyro + Accel @ 200 Hz)"]
        MIC["Audio Input Devices<br/>• Bluetooth SCO Headset / Bone Conduction<br/>• Built-in Smartphone Mic Array"]
        SOC["Qualcomm Snapdragon SoC<br/>• Qualcomm Hexagon NPU (HTP v73/v75/v81)<br/>• ARM Cortex-A CPU (arm64-v8a)"]
        HAPTIC_DEV["Linear Resonant Actuator (LRA)<br/>• Directional Haptic Motor"]
    end

    %% ==========================================
    %% 2. NATIVE C++ PERCEPTION ENGINE
    %% ==========================================
    subgraph NATIVE ["2. NATIVE PERCEPTION & INFERENCE ENGINE (libhsv2_lite.so & librealsense2.so)"]
        direction TB

        subgraph INGEST ["Sensor Ingestion & Preprocessing"]
            RS_LIB["librealsense2.so<br/>UVC / V4L2 Device Driver"]
            FRAME_SYNC["native-lib.cpp<br/>Depth-RGB Deprojection & Alignment"]
            IMU_PROC["imu.cpp & imu_sync.cpp<br/>Gravity Calibration & EKF Orientation"]
        end

        subgraph QNN_ENGINE ["Qualcomm AI Runtime (QAIRT / QNN v2.47.0)"]
            QNN_RT["qnn_runtime.cpp<br/>HTP Backend Dynamic Loader (dlopen)"]
            HTP_SO["libQnnHtp.so / libQnnSystem.so<br/>Hexagon NPU Execution Context"]
            YOLO_NET["yolo_detector.cpp<br/>YOLOv8 Obstacle & Hazard Detector"]
            SEG_NET["semseg.cpp<br/>DeepLabv3+ Traversable Path Segmentation"]
        end

        subgraph SPATIAL_PLAN ["Spatial Reasoning & Safety Assessment"]
            TRACKER["tracker.cpp<br/>3D Multi-Object Tracking & Kalman Filter"]
            TTC["safety.cpp & safety_validator.cpp<br/>Time-to-Collision (TTC) & Hazard Cones"]
            LOCAL_MAP["local_map.cpp<br/>2.5D Metric Occupancy Grid"]
            NAV_PLAN["nav_plan.cpp<br/>D* Lite Path Planner & Evasion Vectors"]
        end
    end

    %% ==========================================
    %% 3. ANDROID PLATFORM & ARGUS VOICE ENGINE
    %% ==========================================
    subgraph ANDROID ["3. ANDROID PLATFORM & ARGUS VOICE ASSISTANT (Kotlin / JNI)"]
        direction TB

        JNI["MainActivity.kt<br/>• JNI Native Bridge<br/>• Sensor Health Monitor<br/>• Lifecycle & Permission Guard"]

        subgraph SPEECH_STACK ["Argus Offline Voice Pipeline"]
            AUDIO_ROUTE["AudioRoute.kt<br/>Bluetooth SCO / A2DP Priority Routing"]
            MIC_CAP["MicCapture.kt<br/>16 kHz 16-bit Mono PCM Streaming"]
            VAD["SileroVad.kt<br/>Silero Voice Activity Detection (ONNX)"]
            ASR["MoonshineAsr.kt / ParakeetAsr.kt<br/>Sherpa-ONNX Local Offline Speech-to-Text"]
            INTENT["VoiceCommandProcessor.kt<br/>Intent Parser & Voice Command Router"]
            LLM["LiteRT-LM<br/>Gemma 4 E2B On-Device LLM (Scene Explainer)"]
        end

        subgraph FEEDBACK_SYS ["Multimodal Output System"]
            EARCON["Earcon.kt<br/>Directional Stereo Cues & Spatial Audio Tones"]
            VOICE_GUIDE["VoiceGuide.kt<br/>TTS Spoken Alerts & Evasion Guidance"]
            VIBE_MGR["AccessibilityStatusProvider.kt<br/>VibratorManager Directional Haptic Pulses"]
        end
    end

    %% ==========================================
    %% 4. FLUTTER USER INTERFACE & ACCESSIBILITY
    %% ==========================================
    subgraph FLUTTER ["4. FLUTTER USER INTERFACE & ACCESSIBILITY LAYER (Dart)"]
        direction TB
        BRIDGE["Flutter Platform Channels<br/>• MethodChannel: com.hsv2.hsv2_lite/control<br/>• EventChannel: com.hsv2.hsv2_lite/events"]
        APP_STATE["app_state.dart (Provider)<br/>• Camera Feed State & Sensor Health<br/>• Active Guidance Mode & Obstacle Buffer<br/>• Push-to-Talk Mic State"]
        UI_VIEW["home_screen.dart & UI Components<br/>• High-Contrast Live Video / Depth HUD<br/>• Directional Compass & Distance Indicators<br/>• TalkBack & Screen-Reader Accessible Semantics<br/>• Accessible Push-to-Talk Button"]
    end

    %% ==========================================
    %% INTER-LAYER DATA FLOW & SIGNALS
    %% ==========================================

    RS -->|USB 3.0 Type-C| RS_LIB
    MIC -->|Audio Streams| AUDIO_ROUTE
    SOC -.->|Hexagon DSP / NPU Hardware| HTP_SO

    RS_LIB --> FRAME_SYNC
    RS_LIB --> IMU_PROC
    FRAME_SYNC --> QNN_RT
    IMU_PROC --> TRACKER

    QNN_RT <--> HTP_SO
    HTP_SO --> YOLO_NET
    HTP_SO --> SEG_NET
    YOLO_NET -->|3D Bounding Boxes| TRACKER
    SEG_NET -->|Path Traversability Mask| LOCAL_MAP

    TRACKER --> TTC
    LOCAL_MAP --> NAV_PLAN
    TTC --> NAV_PLAN

    NAV_PLAN -->|Perception & Hazard Structs| JNI
    FRAME_SYNC -->|RGB/Depth Frames| JNI

    AUDIO_ROUTE --> MIC_CAP
    MIC_CAP --> VAD
    VAD -->|Voice Windows| ASR
    ASR -->|Transcribed Text| INTENT
    INTENT -->|Open Scene Queries| LLM
    LLM -->|Scene Answers| VOICE_GUIDE
    INTENT -->|Command Actions| JNI
    JNI -->|Active Hazards & Directions| VOICE_GUIDE
    JNI -->|Collision Warning Tones| EARCON
    JNI -->|Proximity Pulses| VIBE_MGR

    VIBE_MGR --> HAPTIC_DEV
    EARCON -->|Spatial Audio Out| MIC
    VOICE_GUIDE -->|Spoken Voice Out| MIC

    JNI <==>|MethodChannel / EventChannel| BRIDGE
    BRIDGE <==> APP_STATE
    APP_STATE <==> UI_VIEW
    UI_VIEW -->|User Tap / Push-to-Talk Trigger| APP_STATE
```

---

## 2. Core Subsystems

### A. Sensory Ingestion & Perception (C++ Native)
- **Library Target**: `libhsv2_lite.so` & `librealsense2.so`
- **Source Files**: `native-lib.cpp`, `imu.cpp`, `tracker.cpp`, `safety.cpp`, `nav_plan.cpp`, `local_map.cpp`
- **Key Functions**:
  - Ingests synchronized RGB (1080p) + Depth (848x480) streams from Intel RealSense D455.
  - Aligns depth to color coordinates and deprojects metric 3D point clouds.
  - Calibrates vertical orientation using the camera's internal Bosch BMI085 IMU.
  - Tracks obstacles across consecutive frames using Kalman filters.
  - Computes Time-to-Collision (TTC) and enforces safety zones (Clear, Caution, Danger).
  - Dynamically computes local path deviations and detour vectors with D* Lite.

### B. Qualcomm AI Runtime (QNN / QAIRT v2.47.0)
- **Source Files**: `qnn_runtime.cpp`, `yolo_detector.cpp`, `semseg.cpp`
- **Hardware Acceleration**: Executes deep neural networks on Qualcomm Hexagon NPU (HTP backends v73, v75, v81) via dynamic runtime linking (`libQnnHtp.so`).
- **Models**:
  - **YOLOv8**: Real-time object detection (pedestrians, vehicles, obstacles, doors, stairs).
  - **DeepLabv3+**: Semantic scene segmentation (sidewalk, roadway, terrain traversability).

### C. Argus Offline Speech & Voice Engine (Kotlin)
- **Package**: `com.hsv2.hsv2_lite.voice`
- **Source Files**: `AudioRoute.kt`, `MicCapture.kt`, `SileroVad.kt`, `MoonshineAsr.kt`, `ParakeetAsr.kt`, `VoiceCommandProcessor.kt`, `Earcon.kt`, `VoiceAssistantManager.kt`
- **Key Functions**:
  - Automatically negotiates audio endpoints (Bluetooth SCO headsets, smart glasses, internal microphone).
  - Continuously samples 16 kHz 16-bit mono PCM audio.
  - Applies Silero VAD to detect user speech frames and discard background noise.
  - Transcribes speech entirely on-device via Sherpa-ONNX (Moonshine/Parakeet models) without internet dependency.
  - Parses voice intents (`"What's in front of me?"`, `"Stop"`, `"Where am I?"`, `"Battery status"`).
  - Delivers spatial audio earcons and synthesized voice guidance.

### D. User Interface & State Management (Flutter / Dart)
- **Package**: `lib/` (`main.dart`, `screens/home_screen.dart`, `providers/app_state.dart`)
- **Key Functions**:
  - High-contrast visual interface for low-vision users.
  - TalkBack screen-reader labeling on all interactive elements.
  - Real-time HUD showing camera status, active obstacle distance, directional arrows, and push-to-talk mic button.
  - Communicates with native layer via asynchronous `MethodChannel` and streaming `EventChannel`.
