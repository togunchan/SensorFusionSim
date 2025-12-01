# Mathematical Models

This document summarizes the math used across the motion generator, tracker, solver, and controller. The intent is to show how each equation maps directly to the code.

## TargetMotionGenerator
- Angular position grows linearly: `theta(t) = omega * t` where `omega = angularSpeedRadSec`.
- Radius evolution:
  - Orbit with fixed radius `R = initialRadius` until `t >= spiralStartTimeSec`.
  - After that, shrink radially: `R(t) = max(0, initialRadius - radialShrinkRate * (t - spiralStartTimeSec))`.
- Position (XY spiral/orbit, fixed altitude):  
  `x = R * cos(theta)`, `y = R * sin(theta)`, `z = altitude`.
- Radial rate: `dR/dt = 0` before the spiral, `dR/dt = -radialShrinkRate` during the spiral.
- Velocity (chain rule on `x(t), y(t)`):  
  `vx = dR/dt * cos(theta) - R * omega * sin(theta)`  
  `vy = dR/dt * sin(theta) + R * omega * cos(theta)`  
  `vz = 0`.
- Acceleration (centripetal dominant):  
  `ax ≈ -R * omega^2 * cos(theta)`, `ay ≈ -R * omega^2 * sin(theta)`, `az = 0`. The code uses this simplified centripetal term to keep the dynamics readable.
- `yawRate` is simply `omega`, the rotation rate around the vertical Z axis.

## TargetTracker (Kalman filter)
- State vector: `x = [px, py, vx, vy]^T` (planar position/velocity; Z is kept constant elsewhere).
- Transition for a constant-velocity model with acceleration input `a = [ax, ay]^T`:  
  `F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]`  
  `B = [[0.5*dt^2, 0], [0, 0.5*dt^2], [dt, 0], [0, dt]]`.
- Process noise assumes acceleration variance `qVar = accelNoise^2`:  
  `Q` is diagonal with position terms `0.25*dt^4*qVar` and velocity terms `dt^2*qVar`.
- Heading integration: `theta += gyro_z * dt` using IMU gyro about Z. The angle is wrapped with `std::remainder` to `[-pi, pi]` for numerical stability.
- Measurement: LiDAR range `r` plus heading `theta` are converted to Cartesian: `z = [r*cos(theta), r*sin(theta)]^T`.
- Measurement model: `H = [[1, 0, 0, 0], [0, 1, 0, 0]]` extracts position; measurement noise `R = sigma^2 * I`, where `sigma = max(0.5, noise_sigma + 0.2)` accounts for base LiDAR noise plus configured sigma.
- Kalman steps:
  - Predict: `x = F*x + B*a`, `P = F*P*F^T + Q`, advancing to the measurement timestamp.
  - Update: compute innovation `y = z - H*x`, innovation covariance `S = H*P*H^T + R`, gain `K = P*H^T*S^{-1}`, then `x = x + K*y`, `P = (I - K*H)*P`.
- `covariance_trace = trace(P)` is published as a scalar uncertainty summary used downstream; larger traces mean less certainty.

## Confidence model in TargetTracker
- **uncertaintyScore**: `1 / (1 + 0.5*(posVar + 0.2*velVar))` penalizes large position/velocity variance.
- **innovationScore**: `exp(-||innovation|| / (sqrt(trace(S)) + 1e-3))` shrinks when residuals grow relative to expected noise.
- **ageScore**: `exp(-ageSec / 1.5)` decays as measurements get old.
- Confidence blend: `blend = 0.4*innovationScore + 0.4*uncertaintyScore + 0.2*ageScore`; the tracker smooths it with the previous value and clamps to `[0.05, 0.98]`. Additional decay is applied when `now - lastUpdate > maxUpdateInterval`.

## TrajectorySolver stability score
- Inputs: tracker `position`, `velocity`, `confidence`, and `covariance_trace`.
- Angular solution:
  - Azimuth offset: `atan2(py, px)`.
  - Elevation offset: `atan2(pz, sqrt(px^2 + py^2))`.
- Stability terms:
  - **speedScore**: `1 / (1 + 0.2 * ||v_xy||)`; faster lateral motion lowers stability.
  - **covScore**: `1 / (1 + 0.05 * covariance_trace)`; large uncertainty lowers stability.
  - **confidence**: direct tracker confidence.
- Combined score: `stability_score = clamp(0.5*confidence + 0.3*covScore + 0.2*speedScore, 0, 1)`.
- A solution is flagged stable when `stability_score >= stabilityThreshold` **and** `confidence > 0.2`.

## EngagementController FSM
- States: `Idle → Acquiring → Tracking → Aligning → Ready → Executing → Completed`, plus a `Safe` fallback.
- Key parameters:
  - `minStabilityToAlign` — baseline stability requirement; multiplied by factors per state (0.4–1.1) to tighten requirements as the controller progresses.
  - `minDwell` — minimum time per state; additional per-state offsets add inertia (e.g., Aligning requires `minDwell + 300 ms`).
  - `safeRecovery` — time the FSM must spend in `Safe` before re-attempting acquisition.
  - `dataTimeout` — if no fresh `KinematicSolution` arrives in this window, transition to `Safe`.
  - `heartbeat` — periodic re-publication of the current state for observability.
- Transitions:
  - Forward path: When stability meets the scaled thresholds **and** the state has dwelled long enough, the FSM advances one step toward `Completed`.
  - Safe fallback: If data is stale or `stability_score < 0.02`, move to `Safe` immediately.
  - Recovery: From `Safe`, after `safeRecovery` and with `sol.is_stable` plus moderate stability (`factor 0.6`), return to `Acquiring`.
