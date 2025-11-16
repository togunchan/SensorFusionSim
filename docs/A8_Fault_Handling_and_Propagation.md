# SensorFusionSim - Fault Handling & Fault Propagation Model (v0.1)

This document defines how SensorFusionSim detects, reports, and reacts to faults during execution. The goal is to keep the system observable, deterministic, and capable of degrading gracefully under failure conditions instead of failing silently.

Fault handling is described at three levels:

- Module-level faults
- Communication-level faults
- System-level faults

All faults are surfaced through structured `SystemEvent` messages, enriched with severity information and suitable for logging and visualization.

---

## 1. Scope & Goals

SensorFusionSim’s fault handling model is designed to:

- Make failures **observable** via `SystemEvent` messages.
- Preserve **determinism** even when faults occur (no hidden side effects).
- Allow **degraded operation** when possible instead of hard failure.
- Provide a clear path for **recovery** (soft/hard restart, replay).
- Keep fault propagation explicit and traceable across modules.

This specification complements:

- A4 – Thread Model & Concurrency Strategy  
- A5 – Architectural Rationale  
- A7 – Message Timing & Ordering Guarantees  

---

## 2. Fault Model Overview

Faults are categorized into three levels:

1. **Module-level faults** – Issues confined to a single module (SensorManager, TargetTracker, TrajectorySolver, EngagementController).
2. **Communication-level faults** – Queue overflows, subscriber exceptions, and backpressure in CommunicationBus.
3. **System-level faults** – Global timeouts, virtual clock anomalies, determinism violations.

Each fault produces at least one `SystemEvent` with:

- `type` (e.g., `FaultInjected`, `StateTimeout`, `SystemStall`)
- severity (`INFO`, `WARN`, `ERROR`, `CRITICAL`)
- descriptive message
- timestamp (virtual simulation time)

---

## 3. Module-Level Faults

### 3.1 SensorManager Faults

**Examples:**

- Sensor update loop misses deadlines.
- IMU or LiDAR data becomes invalid (NaN, INF, out-of-range).
- Sensor thread terminates unexpectedly.

**Handling strategy:**

- Encode sensor anomalies into `SensorFrame` fault flags where applicable.
- Emit a `SystemEvent` (e.g., `FaultInjected`) summarizing the issue.
- Optionally perform a **soft restart** of the affected sensor thread.
- Allow EngagementController to move the system into a safer state if persistent.

---

### 3.2 TargetTracker Faults

**Examples:**

- Fusion calculations produce NaN or INF.
- Inconsistent timing between IMU and LiDAR inputs.
- SPSC buffer overflow on the sensor → tracker path.

**Handling strategy:**

- Discard invalid fusion results (NaN/INF) and avoid publishing corrupted `TrackerState`.
- Reduce `confidence` to reflect degraded tracking quality.
- Emit `SystemEvent` entries when repeated fusion anomalies occur.
- Ensure downstream modules never see a `TrackerState` derived from invalid input.

---

### 3.3 TrajectorySolver Faults

**Examples:**

- Computed solutions are unstable (low stability score).
- Solver loop exceeds its expected execution deadline.

**Handling strategy:**

- Mark solutions with `is_stable = false` when below a defined stability threshold.
- Emit a `SystemEvent` describing unstable solutions or deadline misses.
- Allow EngagementController to treat unstable solutions as advisory or ignore them based on policy.

---

### 3.4 EngagementController Faults

**Examples:**

- Illegal state machine transition.
- Timeouts waiting for required inputs (e.g., no recent `KinematicSolution`).
- Internal logic exceptions.

**Handling strategy:**

- Each illegal transition produces a `SystemEvent` with type `FaultInjected` or `StateTimeout`.
- On critical conditions, the controller transitions the system to a `Safe` state.
- Exceptions inside controller logic are caught, translated into `SystemEvent`, and may trigger safe-mode behavior.

---

## 4. Communication-Level Faults

### 4.1 Queue Overflows

**Scenario:**

- A message queue (ring buffer or bus queue) reaches capacity.

**Handling strategy:**

- The overflow policy is explicit and deterministic (e.g., “drop oldest” or “drop newest”), chosen via configuration.
- Each dropped message generates a `SystemEvent` describing:
  - message type
  - queue name
  - policy in effect

This ensures message loss is never silent.

---

### 4.2 Subscriber Exceptions

**Scenario:**

- A subscriber callback throws an exception during message delivery.

**Handling strategy:**

- The CommunicationBus worker catches all subscriber exceptions.
- The bus worker itself must not terminate as a result of subscriber failures.
- A `SystemEvent` with type `FaultInjected` is emitted, including:
  - subscriber identifier
  - message type involved
  - a short description (where available)
- Simulation continues unless configuration requires a halt on subscriber error.

---

### 4.3 Backpressure / Dead Consumers

**Scenario:**

- A subscriber stops consuming messages fast enough, causing queue growth.

**Handling strategy:**

- Queue depth is monitored by the CommunicationBus.
- When depth exceeds a configured threshold, a `SystemEvent` (e.g., `SubscriberBackpressure`) is emitted.
- Future versions may:
  - expand queue capacity,
  - apply priority-based dropping,
  - or throttle publishers.

---

## 5. System-Level Faults

### 5.1 Global Timeout / System Stall

**Scenario:**

- No new messages are produced for a configured interval (e.g., 100 ms or more).

**Handling strategy:**

- Emit a `SystemEvent` with type `SystemStall`.
- EngagementController may:
  - transition to `Safe` state,
  - trigger a partial or full restart,
  - or suspend certain operations.

---

### 5.2 Virtual Clock Anomalies

**Scenario:**

- The virtual simulation clock jumps unexpectedly forward or backward.
- Timestamps violate monotonicity assumptions.

**Handling strategy:**

- Emit a `SystemEvent` describing the anomaly and affected range.
- During replay, such events can be used to detect and analyze timing issues.
- Future extensions may include corrective logic for clock anomalies.

---

### 5.3 Determinism Violations

**Scenario:**

- Observed message ordering or state transitions violate expected deterministic behavior.

**Handling strategy:**

- If detection logic is enabled, the system logs:
  - current queue contents,
  - message histories,
  - and relevant state machine context.
- Emit a `SystemEvent` indicating a determinism violation.
- System may enter `Safe` mode or halt, depending on configuration.

---

## 6. Severity Levels

Every `SystemEvent` is tagged with a severity level:

- **INFO** – Normal operational messages and benign notifications.
- **WARN** – Degraded behaviour but system remains operational.
- **ERROR** – Significant fault requiring attention or recovery.
- **CRITICAL** – System-level fault; controller is expected to move the system into a safe state or stop the simulation.

These levels are intended for:

- Logging filters
- Visual dashboards
- Future monitoring and alerting tools

---

## 7. Recovery Strategy

SensorFusionSim supports the following recovery concepts:

- **Soft restart**
  - Restart a specific sensor or module thread without resetting the entire system.
  - Typical for transient sensor failures.

- **Hard restart**
  - Reset critical processing modules (TargetTracker, TrajectorySolver, CommunicationBus) while preserving logs.
  - Used when internal state may be inconsistent.

- **Replay-based restore**
  - Use logged messages to reconstruct a known-good state.
  - Enables detailed post-mortem analysis and simulation of “what if” scenarios.

The exact recovery actions are policy-driven and can be extended in future versions.

---

## 8. Interaction with Determinism

Fault handling must never:

- silently drop messages without recording a `SystemEvent`,
- reorder messages in a way that breaks A7 guarantees,
- introduce hidden state changes not visible through logs.

The combination of:

- A6 – Data Structures & Memory Layout  
- A7 – Message Timing & Ordering Guarantees  
- A8 – Fault Handling & Fault Propagation Model  

ensures that **even under fault conditions**, SensorFusionSim remains traceable and suitable for deterministic replay.

---

## End of A8 - Fault Handling & Fault Propagation Model (v0.1)