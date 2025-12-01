# SensorFusionSim
A deterministic, modular, multi-threaded C++20 sensor-fusion pipeline with motion modeling, Kalman filtering, trajectory solving, engagement FSM, and live visualization.
## Overview
- Deterministic timing through a shared `VirtualClock` that every module advances explicitly.
- Thread-safe `CommunicationBus` with per-message-type queues and fan-out subscription.
- Motion-driven synthetic sensing (IMU accel/gyro + LiDAR range) from `TargetMotionGenerator`.
- 2D constant-velocity Kalman `TargetTracker` with gyro-based heading integration.
- `TrajectorySolver` producing azimuth/elevation offsets and stability scores.
- `EngagementController` finite state machine with dwell, safety fallback, and heartbeat publishing.
- Visualization path that serializes bus traffic to JSON for `Visualization/liveplot_server.py` (matplotlib).

## Architecture
```
                           +-----------------+
                           |  VirtualClock   |
                           +-----------------+
                                      |
                                      v
+---------------+   +-----------------------+   +------------------+   +----------------------+   +----------------------+   +-----------------------+   +-----------------------+
| TargetMotion  |-->| SensorManager (IMU/L) |-->| CommunicationBus |-->| TargetTracker (KF)  |-->| TrajectorySolver     |-->| EngagementController  |-->| VisualizationPublisher|--> liveplot_server.py
+---------------+   +-----------------------+   +------------------+   +----------------------+   +----------------------+   +-----------------------+   +-----------------------+
```

## Threading & Timing Model
| Module | Threading | Timing Source | Publishes | Subscribes |
| --- | --- | --- | --- | --- |
| VirtualClock | No thread; monotonic time base | N/A | N/A | N/A |
| CommunicationBus | Single `std::jthread` worker draining per-type queues | Does not advance clock | All message types | N/A |
| TargetMotionGenerator | Called inside SensorManager thread | `VirtualClock` | N/A | N/A |
| SensorManager | `std::jthread`; 1 ms tick | Advances `VirtualClock` every tick | `SensorFrame` | N/A |
| TargetTracker | `std::jthread`; 20 ms tick | Advances `VirtualClock` every tick | `TrackerState` | `SensorFrame` |
| TrajectorySolver | `std::jthread`; configurable tick | Advances `VirtualClock` every tick | `KinematicSolution` | `TrackerState` |
| EngagementController | `std::jthread`; 20 ms tick + heartbeat | Advances `VirtualClock` every tick | `EngagementState` | `KinematicSolution` |
| VisualizationPublisher | Bus callbacks (no internal thread) | N/A | JSON to LivePlotClient | All bus topics |
| LivePlotClient | `std::jthread` handling TCP connect/reconnect | N/A | TCP JSON lines | N/A |
| liveplot_server.py | Python thread for socket + matplotlib animation | Wall time | Plots | JSON input |

All workers share the same `VirtualClock` instance; every loop advances time after its work, yielding deterministic, repeatable ordering independent of wall-clock jitter.

## Modules
- **VirtualClock** (`VirtualTime/`): Deterministic clock with explicit `advance()`/`reset()`. Negative deltas are ignored to maintain monotonicity.
- **CommunicationBus** (`CommunicationBus/`): Type-safe pub/sub bus with per-message queues, optional overflow dropping, and a single worker that performs deterministic fan-out to registered handlers.
- **TargetMotion** (`TargetMotion/`): `TargetMotionGenerator` produces orbit-to-spiral kinematics (position, velocity, acceleration, yaw rate) keyed to `VirtualClock`.
- **SensorManager** (`SensorManager/`): Synthesizes IMU accel/gyro and top-down LiDAR range from `TargetMotion`; publishes `SensorFrame` at per-sensor rates.
- **TargetTracker (Kalman)** (`TargetTracker/`): 2D constant-velocity Kalman filter; integrates gyro to heading, converts LiDAR range to Cartesian, fuses with IMU accel, and outputs pose/velocity/confidence.
- **TrajectorySolver** (`TrajectorySolver/`): Converts tracker state to azimuth/elevation offsets and stability score; classifies stability against configurable threshold.
- **EngagementController (FSM)** (`EngagementController/`): Finite state machine (Idle → Acquiring → Tracking → Aligning → Ready → Executing → Completed, with Safe recovery). Uses dwell timers, stability thresholds, and data freshness to transition.
- **VisualizationPublisher + LivePlotClient** (`Visualization/`): Subscribes to all bus topics, serializes to compact JSON, and streams to `liveplot_server.py` which renders range, az/el/stability, confidence, and FSM state.
- **DataLogger**: Bus subscriber for future logging expansion (present but not central to the default pipeline).

## Message Contracts
Key message types are defined in `include/SensorFusionSim/Messages.hpp` and carried on the bus:

```cpp
struct SensorFrame {
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Vector3f imu_accel;
    Eigen::Vector3f imu_gyro;
    float lidar_range{0.0f};
    bool dropout_flag{false}, spike_flag{false}, stuck_flag{false};
    float noise_sigma{0.0f};
};

struct TrackerState {
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Vector3f position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
    float confidence{0.0f};
    float covariance_trace{0.0f};
};

struct KinematicSolution {
    std::chrono::steady_clock::time_point timestamp;
    float azimuth_offset{0.0f};
    float elevation_offset{0.0f};
    float stability_score{0.0f};
    bool is_stable{false};
};

enum class EngagementState {
    Idle, Acquiring, Tracking, Aligning, Ready, Executing, Completed, Safe
};
```

## Pipeline Walkthrough (single simulation tick)
```
VirtualClock.tick(dt)
 1) TargetMotionGenerator::update()
 2) SensorManager samples motion, builds SensorFrame (IMU accel+gravity, gyro yaw rate, LiDAR range) -> publish
 3) CommunicationBus worker fans out SensorFrame to subscribers
 4) TargetTracker integrates gyro to heading, runs KF predict/update, computes confidence -> publish TrackerState
 5) TrajectorySolver computes azimuth/elevation/stability from TrackerState -> publish KinematicSolution
 6) EngagementController processes solution, enforces dwell/safety/timeouts, emits EngagementState heartbeat
 7) VisualizationPublisher serializes all messages to JSON -> LivePlotClient -> liveplot_server.py plots
```

Pipeline diagram:
```
SensorFrame --> TargetTracker --> TrajectorySolver --> EngagementController --> VisualizationPublisher --> Plot Server
        ^             |                   |                     |                        |
        |             |                   |                     |                        |
   TargetMotion   (gyro heading)   (stability threshold)   (dwell/heartbeat)         JSON serialization
        |
 VirtualClock advances in every worker loop
```

## Component Interactions
- `SensorManager` and `TargetMotionGenerator` share the `VirtualClock`; each 1 ms sensor tick both generates data and advances time.
- `TargetTracker` subscribes to `SensorFrame`, integrates `imu_gyro` to heading, fuses LiDAR-derived XY with IMU accel, and publishes `TrackerState`.
- `TrajectorySolver` listens for `TrackerState`, computes az/el and stability from confidence, covariance trace, and lateral speed, and publishes `KinematicSolution`.
- `EngagementController` consumes `KinematicSolution`, applies dwell timers and data staleness checks, transitions the FSM, and publishes `EngagementState` heartbeats.
- `VisualizationPublisher` subscribes to all topics and uses `JsonSerializer` to emit compact JSON lines to `LivePlotClient`, which manages the TCP link to `liveplot_server.py`.
- `CommunicationBus` isolates publishers/subscribers: thread-safe enqueue, single worker thread for deterministic ordering, optional overflow dropping.

## Why This Architecture Matters
- **Deterministic simulation**: A shared `VirtualClock` prevents wall-clock jitter from contaminating filter timing, making runs repeatable for tuning and testing.
- **Typed pub/sub boundary**: Clear message contracts decouple sensing, estimation, control, and visualization, matching robotics/embedded patterns used for real systems.
- **Single-writer bus**: A dedicated bus worker provides ordered delivery without cross-thread locks inside modules, reducing race conditions in concurrent pipelines.
- **Testable units**: Each module (clock, bus, tracker, solver, FSM, serialization) is isolated and covered by unit tests, mirroring production-grade component boundaries.
- **Realtime-friendly**: `std::jthread`/`stop_token` loops, compact data structures, and no dynamic polymorphism in the hot path keep the code close to deployable embedded style.

## Build & Install
Prerequisites:
- C++20 compiler
- CMake ≥ 3.21
- Eigen3
- pthreads (POSIX)
- Python 3 with matplotlib (for visualization)

Configure and build:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Run unit tests (optional, `BUILD_TESTING=ON`):
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

## Run the Simulator
1. Start the plot server:
```bash
python3 Visualization/liveplot_server.py
```
2. In another terminal, run the simulator binary:
```bash
./build/SensorFusionSim
```
The default `main` (`src/main.cpp`) configures a spiral motion, one IMU/LiDAR sensor, and runs the full pipeline for ~15 seconds while streaming JSON to the plot server.

## How to Integrate Real Sensors
- Replace or augment `SensorManager` to ingest real IMU/LiDAR drivers and populate `SensorFrame`.
- Preserve the `VirtualClock` contract by advancing it in step with incoming samples to keep downstream timing consistent.
- Publish `SensorFrame` via `CommunicationBus`; `TargetTracker`, `TrajectorySolver`, `EngagementController`, and `VisualizationPublisher` require no changes if message contracts are honored.
- Optionally extend `SystemEvent` publishing for driver faults or health monitoring.

## Repository Layout
- `src/main.cpp` – wiring of all modules and default simulation parameters.
- `VirtualTime/` – deterministic clock implementation.
- `CommunicationBus/` – pub/sub infrastructure.
- `TargetMotion/` – synthetic target kinematics.
- `SensorManager/` – synthetic IMU/LiDAR publisher.
- `TargetTracker/` – Kalman filter tracker.
- `TrajectorySolver/` – az/el + stability computation.
- `EngagementController/` – engagement FSM.
- `Visualization/` – JSON serialization, TCP client, matplotlib server.
- `tests/` – Catch2-based unit tests per module.

## Project Goals
- A clean publish/subscribe architecture in modern C++20
- Fully deterministic timing via a custom VirtualClock
- Modular separation of sensing → tracking → solving → engagement
- A real-world style guidance/engagement FSM
- Working Kalman filtering with uncertainty propagation
- End-to-end telemetry and real-time visualization
  
## License
Apache 2.0. See [License](LICENSE).

## Contact

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Murat_Toğunçhan_Düzgün-blue.svg)](https://www.linkedin.com/in/togunchan/)
[![GitHub](https://img.shields.io/badge/GitHub-togunchan-black.svg)](https://github.com/togunchan)
