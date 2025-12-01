# Data Flow per Simulation Tick

This describes what happens during one simulation iteration, using the shared `VirtualClock` as the timeline.

1. **VirtualClock advances**  
   - Input: previous `VirtualClock::TimePoint`.  
   - Output: new `timestamp` used by all modules for this tick.
2. **TargetMotionGenerator computes pose**  
   - Input: virtual time `t`, `MotionConfig`.  
   - Output: `position`, `velocity`, `acceleration`, `yawRate` for the synthetic target. Key fields: XY spiral radius, centripetal acceleration, yaw rate `omega`.
3. **SensorManager builds `SensorFrame`**  
   - Input: motion outputs.  
   - Output: `SensorFrame {timestamp, imu_accel, imu_gyro, lidar_range, noise_sigma, fault flags}`. Uses acceleration+gravity for IMU accel, yaw rate for gyro.z, XY range for LiDAR.
4. **SensorFrame is published on CommunicationBus**  
   - Input: `SensorFrame`.  
   - Output: queued delivery to subscribers (TargetTracker, VisualizationPublisher).
5. **TargetTracker consumes the frame**  
   - Input: `SensorFrame`.  
   - Output: `TrackerState {timestamp, position, velocity, confidence, covariance_trace}`. Runs predict/update of the Kalman filter, updates heading from gyro, converts range→Cartesian, and refreshes confidence.
6. **TrajectorySolver consumes TrackerState**  
   - Input: `TrackerState`.  
   - Output: `KinematicSolution {timestamp, azimuth_offset, elevation_offset, stability_score, is_stable}`. Computes pointing angles and stability from confidence, covariance_trace, and speed.
7. **EngagementController consumes KinematicSolution**  
   - Input: `KinematicSolution`.  
   - Output: `EngagementState` enum (Idle/Acquiring/…/Safe). Applies dwell timers, stability thresholds, data timeout, and safe recovery logic, then publishes the current state.
8. **VisualizationPublisher streams JSON**  
   - Input: `SensorFrame`, `TrackerState`, `KinematicSolution`, `EngagementState`.  
   - Output: JSON lines over TCP (via `LivePlotClient`) to the Python `liveplot_server.py`, which renders time series for LiDAR range, az/el/stability, tracker confidence, and FSM state.

## Message fields consumed at each hop
- **Tracker**: uses `SensorFrame.imu_accel`, `.imu_gyro`, `.lidar_range`, `.noise_sigma`, `.timestamp`.
- **Solver**: uses `TrackerState.position`, `.velocity`, `.confidence`, `.covariance_trace`.
- **Controller**: uses `KinematicSolution.stability_score`, `.is_stable`, and `.timestamp` for staleness checks.
- **Visualization**: forwards key scalar series (range, az/el, stability_score, confidence, engagement enum) to the plot server.

## Extending to real sensors
- Replace **SensorManager** with hardware-backed drivers that fill the same `SensorFrame` fields (IMU accel/gyro, LiDAR or radar range, noise/fault indicators).
- Keep the CommunicationBus and downstream consumers unchanged; as long as `SensorFrame` is populated with real measurements and timestamps, the tracker, solver, controller, and visualization will process them identically.
