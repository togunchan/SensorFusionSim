# SensorFusionSim - Architectural Rationale (v0.1)

This document explains the reasoning behind the major architectural decisions of SensorFusionSim. Each subsection corresponds to a deliberate choice made to improve determinism, scalability, debuggability, or real-time fidelity.

---

## 1. Why Lock-Free Ingestion for Sensor -> Tracker Path?

**Decision:** Use a single-producer single-consumer (SPSC) lock-free ring buffer.

**Rationale:**
- Sensor data, especially IMU, arrives at high frequency.
- Mutex-based queues introduce latency spikes and priority inversion.
- SPSC perfectly matches the natural roles:
  - Producer: SensorManager
  - Consumer: TargetTracker
- Lock-free structures eliminate contention and reduce jitter.
- This aligns with real robotics middleware (ROS2, PX4, Autoware).

**Consequences:**
- Requires correct use of memory-order semantics.
- Bounded buffer size must be tuned to avoid overflow.

---

## 2. Why Configurable Sensor Threading?

**Decision:** Support both single-threaded and thread-per-sensor modes.

**Rationale:**
- Some systems simulate only 1-2 sensors, so single-threaded is ideal.
- Some systems simulate heterogeneous sensors with different update rates:
  - IMU: 200-500 Hz
  - LiDAR: 10-20 Hz
- Thread-per-sensor prevents slower sensors from blocking fast sensors.
- Configuration flexibility mirrors real robotics stacks.

**Consequences:**
- Thread-per-sensor increases overhead but improves realism.
- Simulator users can choose performance versus fidelity.

---

## 3. Why Hybrid Triggering in TrajectorySolver?

**Decision:** Use a combination of periodic ticks and event-triggered updates.

**Rationale:**
- Kinematic solvers (alignment/orientation math) typically run at fixed control rates (50-200 Hz).
- Major state changes require immediate solver reaction.
- Model predictive control loops and predictive algorithms benefit from hybrid triggering.

**Consequences:**
- Requires careful handling to avoid duplicate computations.
- Provides better real-time responsiveness.

---

## 4. Why a Hybrid Controller (Event-Driven + Tick)?

**Decision:** EngagementController processes both events and low-frequency housekeeping ticks.

**Rationale:**
- State machine transitions are event-driven.
- All real systems need periodic checks for stale data, timeout logic, and safety fallback.
- Fully event-driven controllers miss edge conditions; ticks solve this.

**Consequences:**
- Slightly more complex loop logic.
- Greatly improved reliability.

---

## 5. Why a Single Worker CommunicationBus Thread?

**Decision:** Use a dedicated worker thread for message fan-out.

**Rationale:**
- Multithreaded fan-out leads to nondeterministic message interleaving.
- A single worker thread ensures per-message-type FIFO ordering, reproducible runs, and predictable log sequences.
- Decouples producers from consumers, providing a fast publish() path.

**Consequences:**
- Slight delay between publish and delivery (microseconds to milliseconds).
- Simpler, more debuggable architecture.

---

## 6. Why Full Deterministic Replay?

**Decision:** All module timestamps use a shared virtual simulation clock.

**Rationale:**
- Real-time debugging is extremely hard.
- Deterministic replay allows identical reruns, post-mortem analysis, academic reproducibility, and state-machine debugging.
- Critical for robotics research and simulation.

**Consequences:**
- Virtual clock management introduces an additional layer.
- Increases correctness and traceability dramatically.

---

## 7. Why Hybrid Timing Model (Real-Time + Virtual Clock)?

**Decision:** Use wall-clock time for pacing and a virtual clock for simulation time.

**Rationale:**
- Wall-clock pacing makes the simulation feel real and helps visualize behavior.
- Virtual clock timestamps guarantee replay determinism and keep logs plus solver math consistent.
- This approach is common in flight controllers and robotics simulators.

**Consequences:**
- Requires careful clock synchronization design.
- Enables replay-any-scenario-identically capability.

---

## 8. Summary of Architectural Benefits

SensorFusionSim's architectural decisions deliver:
- Deterministic behavior
- Real-time fidelity
- Predictable concurrency
- Configurable performance scaling
- Replay-based debugging
- Future-proof modularity

This rationale acts as the justification layer for the A4 concurrency specification and the system's long-term evolution.

---

## End of A5 - Architectural Rationale (v0.1)
