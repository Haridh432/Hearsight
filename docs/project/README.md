# HearSight — project documentation

HearSight is a head-worn navigation assistant for blind and low-vision people. An Intel
RealSense D455 depth camera and an Android phone are worn together; the phone sees the space
ahead, builds a local 3D map, plans a safe path and speaks one clear instruction at a time
("Chair 2 metres ahead, path goes right."), in English or Tamil, with vibration cues. All
processing runs on the phone — no cloud, no network.

| Document | What it covers |
|---|---|
| [01 — Overview](01-overview.md) | The problem, who it is for, what it does, hardware, current status |
| [02 — Architecture](02-architecture.md) | Layers, modules, threads, data contracts, the single safety authority |
| [03 — Flow](03-flow.md) | Per-frame pipeline, start-up, STOP → re-route, voice, IMU / VIO, recording and replay |
| [04 — Tech stack](04-tech-stack.md) | Languages, frameworks, libraries (versions, licences), AI models, tools |
| [05 — Benchmark](05-benchmark.md) | Measured timings on the phone and PC, method, and what is not measured yet |

Related: the repository [README](../../README.md) (build, run, debug switches),
[Phase 7 validation protocol](../phase7-validation.md), [client presentation](../client_presentation.md).

*State of the documents: 22 Oct 2026, `main` at commit a370238.*
