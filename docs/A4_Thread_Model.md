# SensorFusionSim - Thread Model & Concurrency Strategy (v0.2)

This document defines the threading, timing, and concurrency model of SensorFusionSim. The goal is to provide a deterministic, debuggable, and extensible real-time simulation pipeline that still reflects modern robotics and embedded system practices.

---

## 1. Objectives

The thread model of SensorFusionSim is designed to:
- Support real-time sensor pipelines with configurable threading.
- Use lock-free ingestion paths for high-frequency sensor data.
- Provide a hybrid solver and controller triggering model (event + periodic).
- Implement a worker-based CommunicationBus for predictable fan-out.
- Achieve fully deterministic replay through a virtual simulation clock.
- Remain simple enough to reason about, but advanced enough to reflect real-world systems.

---

## 2. High-Level Thread Topology

At a high level, SensorFusionSim uses a small set of long-lived threads:
- Sensor thread(s) - owned by SensorManager (configurable).
- Tracker thread - consumes SensorFrame objects from a lock-free ring buffer.
- Solver thread - runs a hybrid event + periodic loop.
- Controller thread - hybrid event-driven + low-frequency tick.
- Bus worker thread - executes pub/sub message fan-out.
- Optional UI / visualization thread - outside the core scope of this document.

```
[SensorManager] --SensorFrame--> [Lock-free RingBuffer] --> [TargetTracker Thread]
                                                    |
                                       TrackerState |
                                                    v
                                           [Solver Thread]
                                       KinematicSolution |
                                                         v
                                             [Controller Thread]
                                            SystemEvent  |
                                                         v
                                     [CommunicationBus Worker Thread]
                                           /                \
                                    [DataLogger]        [Visualization]
```

All threads are created using std::jthread to ensure RAII-based lifecycle management and clean cancellation.

---

## 3. Module Thread Ownership

### 3.1 SensorManager

**Threading mode (configurable):**
- Single-threaded mode (default):
  - One std::jthread drives all sensor updates at their configured rates.
  - Used when sensor count is small or performance requirements are modest.
- Thread-per-sensor mode (advanced):
  - Each sensor instance gets its own std::jthread.
  - Suitable for heterogeneous update rates or high-frequency scenarios.

**Responsibilities:**
- Generates SensorFrame objects at configured intervals.
- Pushes frames into a lock-free SPSC ring buffer consumed by TargetTracker.
- Uses the virtual simulation clock for timestamps.

### 3.2 TargetTracker

**Threading:**
- Runs on a dedicated std::jthread.
- Continuously polls or blocks on the lock-free ring buffer for new SensorFrame objects.

**Responsibilities:**
- Consumes SensorFrame instances in arrival order.
- Performs sensor fusion and state estimation.
- Produces TrackerState messages and publishes them via CommunicationBus.

### 3.3 TrajectorySolver

**Threading:**
- Runs on a dedicated std::jthread.

**Triggering model (hybrid):**
- Periodic tick (e.g., 60 Hz):
  - On each tick, computes a new KinematicSolution based on the latest TrackerState.
- Event-triggered updates:
  - Can optionally wake earlier if a significant TrackerState update arrives (e.g., large delta in position/velocity).

**Responsibilities:**
- Computes neutral alignment parameters (azimuth/elevation offsets).
- Evaluates stability and produces a KinematicSolution.
- Publishes KinematicSolution to CommunicationBus.

### 3.4 EngagementController

**Threading:**
- Runs on a dedicated std::jthread with a hybrid loop:
  - Event-driven: reacts to incoming KinematicSolution messages and SystemEvents.
  - Low-frequency tick: periodic housekeeping (e.g., timeout checks, health states).

**Responsibilities:**
- Implements the robotic state machine (Idle -> Acquiring -> Tracking -> Aligning -> Ready -> Executing -> Completed -> Safe).
- Emits SystemEvent messages to describe transitions and notable events.

### 3.5 CommunicationBus

**Threading:**
- Uses a single worker thread (std::jthread) responsible for:
  - Reading incoming messages from internal queues.
  - Dispatching them to subscribed modules (DataLogger, Visualization, EngagementController, etc.).

**Responsibilities:**
- Maintains per-message-type queues (e.g., SensorFrame, TrackerState, KinematicSolution, SystemEvent).
- Guarantees per-type message ordering (FIFO for each message type).
- Decouples producers and consumers to keep hot paths lightweight.

### 3.6 DataLogger and Visualization

**Threading:**
- Typically run without their own heavy worker threads for the core logic:
  - DataLogger can operate in the bus worker context or with a lightweight queue.
  - Visualization may have its own UI thread (outside core concurrency spec).

**Responsibilities:**
- DataLogger: persists messages and timestamps for replay.
- Visualization: renders live plots, timelines, and state transitions.

---

## 4. CommunicationBus Execution Model

The CommunicationBus coordinates all pub/sub communication using:
- A single worker thread that:
  - Reads from inbound queues (one per message type).
  - Delivers messages to subscribed callbacks or module endpoints.
- Producer side pattern:
  - Modules (SensorManager, TargetTracker, TrajectorySolver, EngagementController) call a non-blocking publish() API.
  - publish() enqueues the message (move-constructed) into the corresponding queue.
  - No module blocks on subscriber execution.
- Consumer side pattern:
  - Subscribers register handlers (e.g., onTrackerState(const TrackerState&)).
  - Bus worker invokes handlers in FIFO order per message type.

This model trades minimal latency for predictability, simplicity, and debuggability, which is ideal for a simulation platform.

---

## 5. Timing & Determinism

SensorFusionSim uses a hybrid timing model:
- Virtual simulation clock:
  - All timestamps in SensorFrame, TrackerState, KinematicSolution, and SystemEvent come from a shared virtual clock.
  - The virtual clock advances in discrete steps controlled by the simulation scheduler.
- Wall-clock real-time execution:
  - Threads sleep using std::this_thread::sleep_for based on desired real-time rates.
  - The real-time pacing ensures that the simulation "feels" real-time.
- Deterministic replay:
  - Because timestamps and message order are tied to the virtual clock and queue order, it is possible to:
    - Record input streams and events.
    - Replay them deterministically independent of wall-clock timing.

Determinism target: bit-for-bit stable behavior given the same input sequence and configuration.

---

## 6. Synchronization & Data Structures

### 6.1 Lock-Free Ingestion
- Sensor -> Tracker path uses an SPSC lock-free ring buffer:
  - Single producer: SensorManager thread(s).
  - Single consumer: TargetTracker thread.
  - Implemented with std::atomic and memory orderings (memory_order_acquire / memory_order_release).

### 6.2 Queues in CommunicationBus
- For each message type, CommunicationBus uses a bounded or unbounded queue:
  - Can be a simple mutex-protected queue initially.
  - May be evolved into lock-free structures later if needed.
- std::mutex + std::condition_variable used inside the bus worker for:
  - Waiting on new messages.
  - Waking up when producers enqueue.

### 6.3 Thread Lifecycle
- All long-lived threads are created as std::jthread.
- Shutdown sequence:
  - A shared std::atomic<bool> stop flag or cooperative stop token is signaled.
  - Each thread breaks its loop and joins automatically via jthread destructor.

---

## 7. Failure Modes & Debugging

The thread model supports:
- Graceful overload handling:
  - If consumers lag, queues grow; this is observable via metrics in DataLogger.
  - Future extensions can include backpressure or dropping policies.
- Replay-based debugging:
  - Logged messages (frames, states, solutions, events) can be replayed without live threads.
  - Helps debug state machine transitions and fusion anomalies.
- Deterministic unit/integration tests:
  - Threads can be replaced with deterministic stepping in tests:
    - Manually advancing the virtual clock.
    - Injecting messages directly into queues.

---

## 8. Roadmap

Planned concurrency-related evolutions:
- Configurable thread-per-module vs. thread-pool execution for advanced scenarios.
- Optional lock-free queues in CommunicationBus if profiling demands it.
- Priority-aware scheduling for high-frequency sensors.
- Exposing debug views for queue depths, thread status, and timing jitter in Visualization.

---

## End of A4 - Thread Model & Concurrency Strategy (v0.2)

This specification forms the concurrency contract for SensorFusionSim. All future implementations of modules must respect these threading, timing, and messaging rules to preserve determinism and maintainability.
