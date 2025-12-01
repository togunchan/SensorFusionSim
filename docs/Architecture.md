# SensorFusionSim Architecture

SensorFusionSim is structured as a set of small modules tied together by a publish/subscribe bus and a shared `VirtualClock`. Each module runs in its own `std::jthread`, advances the virtual time as it steps, and exchanges typed messages instead of calling directly into each other. This mirrors embedded or robotics stacks where deterministic timing, clear data ownership, and low coupling are required.

## Core modules and responsibilities
- **VirtualTime::VirtualClock** — Provides a simulation-only clock (`now()`, `advance()`), so every module uses the same notion of time without touching wall-clock time.
- **CommunicationBus** — Single-threaded fan-out of published messages (`SensorFrame`, `TrackerState`, `KinematicSolution`, `EngagementState`, `SystemEvent`) to registered subscribers. Buffers per type keep the producers and consumers decoupled.
- **SensorManager** — Drives the target motion model, samples virtual IMU + LiDAR data (with noise/fault flags), and publishes `SensorFrame` messages at a fixed rate.
- **TargetTracker** — Runs a planar constant-velocity Kalman filter to fuse LiDAR range + IMU gyro/accel into a `TrackerState` (position, velocity, confidence, covariance_trace).
- **TrajectorySolver** — Converts the tracker estimate into angular offsets (azimuth/elevation) and a stability score, packaged as a `KinematicSolution`.
- **EngagementController** — A finite state machine (FSM) that steps through engagement phases (Idle → … → Completed or Safe) based on solution stability, confidence, data freshness, and dwell timers; publishes `EngagementState`.
- **VisualizationPublisher** — Subscribes to all upstream data, serializes selected fields to JSON lines, and streams them to `LivePlotClient`.
- **liveplot_server.py** — A lightweight TCP server that ingests the JSON lines and renders range/solution/confidence/engagement time series using Matplotlib.

## Pub/sub pipeline
- Typical data path: **SensorManager → TargetTracker → TrajectorySolver → EngagementController → VisualizationPublisher → liveplot_server.py**.
- Message flow:
  - `SensorFrame` (IMU accel/gyro, LiDAR range, noise/fault flags) is published by SensorManager and consumed by TargetTracker + VisualizationPublisher.
  - `TrackerState` (position, velocity, confidence, covariance_trace) is published by TargetTracker and consumed by TrajectorySolver + VisualizationPublisher.
  - `KinematicSolution` (azimuth_offset, elevation_offset, stability_score, is_stable) is published by TrajectorySolver and consumed by EngagementController + VisualizationPublisher.
  - `EngagementState` (FSM enum) is published by EngagementController and consumed by VisualizationPublisher.
  - `SystemEvent` is available for logging/diagnostics (not visualized by default).
- The CommunicationBus uses per-type queues and a single worker that pops messages and invokes subscribers, giving deterministic ordering while keeping producers non-blocking.

## Virtual time and threading model
- Each module owns a `std::jthread` running a tight loop with a fixed tick interval (e.g., SensorManager at 1 ms, Tracker at 20 ms).
- At the end of each loop, the module calls `VirtualClock::advance()` by the same tick duration. Because the same `VirtualClock` instance is shared, all timestamps (`timestamp` fields in messages) advance in lockstep without being tied to wall-clock jitter.
- `std::stop_token` is used to request clean shutdowns; the loop checks `stop_requested()` and exits after publishing any final state.
- This pattern matches typical embedded/defense simulations: deterministic time advancement, periodic tasks, and explicit inter-task messaging instead of shared mutable state.

## Why this architecture maps to real systems
- **Loose coupling** — Sensors, tracker, solver, controller, and visualization communicate via typed messages, allowing components to be replaced (e.g., hardware sensors) without rewriting consumers.
- **Deterministic timing** — Virtual time plus fixed ticks make the simulation reproducible and debuggable, similar to flight software “time bases.”
- **Clear ownership** — Each module owns its state (filter covariance, FSM state, motion parameters), reducing synchronization complexity.
- **Observability** — Every major stage publishes lightweight summaries (confidence, stability, engagement enum) that can be logged or visualized in real time, matching operational telemetry needs.
