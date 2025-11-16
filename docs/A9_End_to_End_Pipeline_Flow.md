# SensorFusionSim - End-to-End Pipeline Flow Specification (v0.1)

This document defines the end-to-end execution pipeline of SensorFusionSim, describing how data, control signals, timestamps, and state-machine transitions propagate through the system. Combined with A4 (Thread Model), A5 (Architectural Rationale), A7 (Message Timing & Ordering), and A8 (Fault Propagation), this specification provides a complete overview of the simulation's runtime behavior.

---

## 1. Overview

SensorFusionSim operates as a coordinated pipeline of sensor production, state estimation, kinematic solving, controller decision-making, and event logging. Each module runs on its own deterministic execution thread and communicates exclusively via structured messages delivered through the CommunicationBus.

Goals of this pipeline specification:
- Describe how a `SensorFrame` flows through every module.
- Show how thread, timing, and state machine interactions create a unified execution.
- Define how faults, delays, and solver outputs propagate end-to-end.
- Provide a reproducible model for deterministic replay.

---

## 2. End-to-End Data Flow

The following canonical flow describes how data moves through the system:

```
SensorManager -> TargetTracker -> TrajectorySolver -> EngagementController
                           v             v                     v
                     TrackerState   KinematicSolution     SystemEvent
```

**Step 1 - Sensor Production**
- SensorManager generates a `SensorFrame` using the virtual simulation clock.
- The frame is pushed into the SPSC lock-free buffer.

**Step 2 - Tracking / Fusion**
- TargetTracker consumes the next `SensorFrame` as soon as it becomes available.
- Produces a fused state estimate (`TrackerState`).
- Publishes the state into the CommunicationBus.

**Step 3 - Kinematic Solving**
- TrajectorySolver is tick-driven (e.g., 60 Hz).
- On each tick, retrieves the latest `TrackerState`.
- Computes azimuth/elevation offsets and stability score.
- Publishes a `KinematicSolution`.

**Step 4 - Controller State Machine**
- EngagementController receives the `KinematicSolution`.
- Determines whether to advance the state machine, whether stability thresholds are satisfied, and whether alignment is complete.
- Emits `SystemEvent` messages via the CommunicationBus.

**Step 5 - Logging & Visualization**
- DataLogger writes every message type to persistent logs.
- Visualization renders real-time state, orientation, errors, and transitions.

---

## 3. End-to-End Control Flow

Threads influence how data is processed over time:

```
Sensor Thread(s) -> Tracker Thread -> Solver Thread -> Controller Thread -> Bus Worker Thread
```

**Sensor Threads**
- Periodically wake according to the sensor update rate (e.g., IMU: 200-500 Hz).
- Produce high-frequency `SensorFrame` messages.

**Tracker Thread**
- Blocks until a new frame is available in the SPSC buffer.
- Performs state estimation immediately.

**Solver Thread**
- Runs periodic ticks and optional event wake-ups.
- Reacts faster when major `TrackerState` changes occur.

**Controller Thread**
- Hybrid: event-driven for solver updates plus a low-frequency tick for timeout checks and safety logic.

**Bus Worker Thread**
- Single-threaded dispatcher that enforces FIFO ordering and causal ordering (A7).

Control flow ensures modules do not block each other and that timing remains deterministic across runs.

---

## 4. Timing & Virtual Clock Integration

Every message uses virtual simulation time, not wall-clock time.

**Virtual clock controls:**
- Sensor timestamps
- Tracker fusion cycle
- Solver tick timestamps
- Controller event timestamps
- Replay determinism

**Wall-clock controls:**
- Thread sleep durations (pacing)
- Perceived real-time behavior

This hybrid model provides deterministic internal logic, real-time pacing, reproducible logs, and predictable solver behavior. All ordering rules from A7 apply here.

---

## 5. State Machine Interaction

The EngagementController state machine affects-and is affected by-the pipeline:

```
Idle -> Acquiring -> Tracking -> Aligning -> Ready -> Executing -> Completed -> Safe
```

**How states influence the pipeline:**
- Acquiring: waits for stable `TrackerState` messages.
- Tracking: requires consistent fusion updates.
- Aligning: waits for stable `KinematicSolution` with high confidence.
- Ready: solver stability threshold satisfied.
- Executing: reacts in real time to controller decisions.
- Safe: triggered on fault or timeout.

**How pipeline events trigger state transitions:**
- Large changes in `TrackerState` may trigger Acquiring -> Tracking.
- Stable `KinematicSolution` may trigger Aligning -> Ready.
- `SystemEvent(FaultInjected)` may trigger any state -> Safe.

---

## 6. Fault Flow Integration

Fault propagation follows the A8 specification:
1. SensorManager injects fault metadata in `SensorFrame`.
2. TargetTracker includes fault flags in confidence outputs.
3. TrajectorySolver reduces stability score if fault impact is detected.
4. EngagementController emits `SystemEvent(FaultInjected)` when thresholds or timeouts are violated.
5. DataLogger records the full chain, enabling postmortem reconstruction.

This ensures that faults never silently vanish and every module reacts explicitly.

---

## 7. Replay & Deterministic Execution

Replay is guaranteed consistent because:
- Virtual timestamps are identical on each run.
- Per-type FIFO ordering always holds.
- CommunicationBus dispatch is single-threaded.
- Logs preserve chronological message order.
- The controller receives the same update sequence, producing the same transitions.

This is critical for debugging, simulation validation, certification-oriented workflows, and regression testing.

---

## 8. Unified Pipeline Diagram

```
      +--------------------+
      |   SensorManager    |
      +---------+----------+
                | SensorFrame
                v
      +--------------------+
      |   TargetTracker    |
      +---------+----------+
                | TrackerState
                v
      +--------------------+
      |  TrajectorySolver  |
      +---------+----------+
                | KinematicSolution
                v
      +--------------------+
      | EngagementControl  |
      +---------+----------+
         | SystemEvent      \
         |                   \
         v                    v
   +------------+      +-------------+
   | DataLogger |      | Visualization|
   +------------+      +-------------+

        [ CommunicationBus Worker Thread ]
```

---

## 9. Engineering Guarantees

| Guarantee              | Description                                         |
|------------------------|-----------------------------------------------------|
| Causal Ordering        | Downstream messages never arrive before upstream ones.|
| Per-Type FIFO          | No message type is ever reordered.                  |
| Deterministic Replay   | Full reproducibility from logs.                     |
| Thread Isolation       | Each module owns its own execution thread.          |
| Fault Traceability     | Every fault produces an observable event.           |
| Virtual-Time Consistency | All timestamps derive from the same timebase.     |

---

## End of A9 - End-to-End Pipeline Flow Specification (v0.1)
