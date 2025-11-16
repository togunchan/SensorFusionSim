# SensorFusionSim - Initialization & Shutdown Sequence Specification (v0.1)

This document defines the global initialization and shutdown sequences of SensorFusionSim. It specifies the ordering, dependencies, and lifecycle contracts for all core modules to ensure deterministic startup, safe shutdown, and predictable behavior under fault or abort conditions.

This specification complements:
- A4 - Thread Model & Concurrency Strategy
- A5 - Architectural Rationale
- A7 - Message Timing & Ordering Guarantees
- A8 - Fault Propagation & Handling
- A10 - Configuration Model Specification

---

## 1. Goals

The lifecycle model of SensorFusionSim is designed to:
- Start all modules in a well-defined, dependency-respecting order.
- Ensure that no module publishes before CommunicationBus is ready.
- Guarantee clean shutdown with proper thread joining and queue draining.
- Propagate stop signals in a controlled, observable way.
- Support deterministic behavior for both normal runs and fault-induced stops.

---

## 2. Global System Lifecycle

The platform follows a global lifecycle with the following states:

```
Created -> Initializing -> Ready -> Running -> Stopping -> Stopped
```

**Created**
- Process started, configuration not yet loaded.

**Initializing**
- Configuration loaded and validated (A10).
- Modules are constructed but not fully started.

**Ready**
- All modules are initialized, threads created, but no sensor data is flowing yet.
- System is ready to enter the main simulation loop.

**Running**
- Sensor -> Tracker -> Solver -> Controller -> Logger/Visualization pipeline is active (A9).

**Stopping**
- A global stop is requested (user command, fault, or external signal).
- Threads begin cooperative shutdown.

**Stopped**
- All threads joined, queues drained or cleared, logs flushed.
- Process may exit or restart.

---

## 3. Startup Sequence (Initialization Ordering)

The startup sequence follows strict ordering to respect dependencies and avoid race conditions. The high-level order is:

1. Configuration System
2. CommunicationBus
3. VirtualClock
4. DataLogger
5. Visualization (optional)
6. TargetTracker (without consuming yet)
7. TrajectorySolver
8. EngagementController
9. SensorManager
10. Transition to Running

### 3.1 Step-by-Step Initialization

**Step 1 - Load & Validate Configuration**
- Load `default_config.json` and optional `user_config.json`.
- Perform merge, validation, and freeze as specified in A10.
- If validation fails:
  - Emit a critical `SystemEvent` (if logging is available).
  - Abort initialization and remain in Created or exit.

**Step 2 - Initialize CommunicationBus**
- Construct CommunicationBus queues and register message types.
- Start the bus worker `std::jthread`.
- From this point, publish/subscribe APIs are valid.

**Step 3 - Initialize VirtualClock**
- Construct the virtual clock with parameters from the timing configuration.
- Ensure all modules use virtual time for timestamps but do not yet consume or produce messages.

**Step 4 - Initialize DataLogger**
- Open log files or database connections according to logging configuration.
- Subscribe to all relevant message types via CommunicationBus.
- Validate write access to the log directory.

**Step 5 - Initialize Visualization (Optional)**
- Create visualization context (GUI, CLI dashboards, etc.).
- Subscribe to state and event streams (`SensorFrame`, `TrackerState`, `KinematicSolution`, `SystemEvent`).
- Visualization must not block the main pipeline.

**Step 6 - Initialize TargetTracker**
- Construct TargetTracker with tracker namespace configuration.
- Register subscriptions for `SensorFrame` via CommunicationBus if needed.
- Prepare internal state; do not process frames until SensorManager starts.

**Step 7 - Initialize TrajectorySolver**
- Construct TrajectorySolver with solver configuration.
- Start solver `std::jthread` in idle or tick-ready state.
- Ensure it can receive `TrackerState` from CommunicationBus.

**Step 8 - Initialize EngagementController**
- Construct EngagementController with controller config.
- Set initial state to Idle.
- Register subscriptions for `KinematicSolution` and `SystemEvent` (for self-observation if needed).
- Start controller `std::jthread`.

**Step 9 - Initialize SensorManager**
- Construct SensorManager with sensors configuration.
- Create sensor thread(s) according to the chosen threading mode (single-threaded or thread-per-sensor per A4).
- Connect SensorManager output to SPSC ring buffer and/or CommunicationBus.

**Step 10 - Transition to Ready -> Running**
- Once all modules report successful initialization:
  - Emit `SystemEvent`: `StateTransition` from global Initializing -> Ready.
  - Start the main simulation:
    - SensorManager begins generating `SensorFrame` messages.
    - Global lifecycle transitions Ready -> Running.
    - Emit `SystemEvent` indicating system is now Running.

---

## 4. Shutdown Sequence (Teardown Ordering)

Shutdown reverses the critical dependencies and ensures that no module is left publishing into a destroyed or inactive infrastructure.

High-level order:

1. Stop SensorManager
2. Stop TargetTracker
3. Stop TrajectorySolver
4. Stop EngagementController
5. Stop Visualization
6. Stop DataLogger
7. Stop CommunicationBus
8. Final resource cleanup

### 4.1 Shutdown Triggers

Shutdown may be initiated by:
- User command (e.g., stop or exit in the shell).
- External signal (e.g., SIGINT / Ctrl+C).
- Fault condition escalated by EngagementController (A8).
- End-of-scenario or test harness completion.

A global stop request is propagated using stop tokens (`std::stop_source` / `std::jthread`) or atomic stop flags, depending on implementation.

### 4.2 Step-by-Step Shutdown

**Step 1 - Enter Stopping State**
- Global lifecycle moves Running -> Stopping.
- EngagementController emits a `SystemEvent(StateTransition)` for the global stop.

**Step 2 - Stop SensorManager**
- Signal sensor threads to stop via stop tokens or flags.
- Ensure no new `SensorFrame` entries are written to the SPSC buffer.
- Threads join via `std::jthread` RAII.

**Step 3 - Stop TargetTracker**
- Signal the tracker thread to exit its loop after finishing any in-flight frame.
- Optionally drain remaining SPSC entries or drop them during shutdown.
- Join the tracker thread.

**Step 4 - Stop TrajectorySolver**
- Signal the solver thread to stop after completing any ongoing computation.
- Avoid starting new work.
- Join the solver thread.

**Step 5 - Stop EngagementController**
- Signal the controller thread to stop.
- Optionally emit a final `SystemEvent(StateTransition)` to Safe or Completed.
- Wait for the state machine to finalize its teardown logic.
- Join the controller thread.

**Step 6 - Stop Visualization (Optional)**
- Close visualization windows or consoles.
- Unsubscribe from bus topics if necessary.
- Ensure no blocking calls remain.

**Step 7 - Stop DataLogger**
- Flush remaining queued messages from CommunicationBus.
- Close log files and database handles.
- Ensure all logs are consistent and replay-ready.

**Step 8 - Stop CommunicationBus**
- Signal the bus worker thread to stop after processing current queues.
- Ensure no further `publish()` calls are accepted, or that late publishes are safely ignored.
- Join the bus worker thread.

**Step 9 - Transition to Stopped**
- Release remaining resources (memory, file handles, timers).
- Global lifecycle moves Stopping -> Stopped.
- Optionally emit a final `SystemEvent` during the last stages of shutdown.

---

## 5. Failure & Abort Handling

Not all shutdowns are graceful. This section defines how the system behaves under failure conditions.

### 5.1 Initialization Failure

If initialization fails at any step:
- Emit a critical `SystemEvent` describing the failure cause (if logging is available).
- Abort startup and prevent transition to Running.
- Deinitialize partially started modules in reverse order of their initialization.

### 5.2 Runtime Fatal Fault

If EngagementController decides that the system must halt (A8):
- Global state transitions to Stopping.
- A fault `SystemEvent` is emitted with details.
- Shutdown sequence follows the same order as normal shutdown but may:
  - Skip draining certain queues for speed.
  - Prioritize safe controller and sensor stop over complete log flushing.

### 5.3 Hard Abort

In extreme cases (assertion failures, unrecoverable logic errors):
- Process may terminate without a full shutdown.
- This mode is treated as a development/debug artifact, not normal operation.

---

## 6. Thread & Stop-Token Contracts

All long-lived threads in SensorFusionSim must:
- Be created as `std::jthread`.
- Check `std::stop_token` or equivalent stop condition periodically.
- Avoid blocking indefinitely on I/O or locks without observing stop requests.
- Join automatically via RAII on destruction of their owning object.

Each module exposes:
- `start()` - creates and starts its thread(s) where applicable.
- `requestStop()` - triggers cooperative shutdown.
- `join()` or implicit join - waits for shutdown to complete (usually handled via destructors).

These contracts make thread management predictable, testable, and safe.

---

## 7. Integration with Other Specifications

The lifecycle model integrates with:
- **A4 Thread Model** - startup/shutdown orders align with thread ownership and hot-path design.
- **A5 Architectural Rationale** - lifecycle follows the rationale for deterministic, fault-tolerant behavior.
- **A7 Timing & Ordering** - shutdown behavior preserves ordering guarantees for in-flight messages where possible.
- **A8 Fault Propagation** - faults can initiate shutdown through EngagementController decisions.
- **A9 End-to-End Pipeline** - startup and shutdown events map to pipeline activation and deactivation.
- **A10 Configuration Model** - configuration is fully validated and frozen before any module enters Running state.

---

## End of A11 - Initialization & Shutdown Sequence Specification (v0.1)
