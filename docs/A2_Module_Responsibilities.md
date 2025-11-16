# SensorFusionSim - Module Responsibilities (v0.2)

## 1. SensorManager
Generates synthetic IMU/LiDAR data streams, manages detailed noise and fault-injection models (drift, dropout, spike, stuck), and publishes thread-safe sensor frames at a defined real-time update rate.

## 2. TargetTracker
Consumes time-synchronized sensor frames, performs multi-sensor fusion, computes position/velocity/confidence estimates, and maintains a short-term state history.

## 3. TrajectorySolver
Takes fused state estimates and computes kinematic alignment results, including orientation offsets, predicted motion alignment, and solver stability metrics.

## 4. EngagementController (Robot Operations Controller)
Implements the neutral robotic state machine (Idle → Initializing → Sensing → Fusing → KinematicSolving → Stabilizing → Operational → Completed → Safe), coordinating module transitions based on data readiness and solver stability.

## 5. CommunicationBus
Provides internal message routing with a lightweight publish–subscribe model, simulating UDP/CAN-like transport, without performing serialization logic (kept separate for SRP).

## 6. DataLogger
Stores structured telemetry, solver outputs, events, and state transitions using a deterministic MiniDB/SQLite backend and supports full replay for debugging and analysis.

## 7. Visualization
Renders sensor streams, fusion states, kinematic solver outputs, and runtime trends through a Qt/ImGui interface, updating in near-real-time and optionally integrating historical trend views from DataLogger.
