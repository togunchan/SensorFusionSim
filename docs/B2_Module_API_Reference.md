# SensorFusionSim - Module API Reference (v0.1)

This document defines the public C++20 APIs exposed by the core modules of SensorFusionSim.  
It describes classes, methods, parameters, thread-safety guarantees, and error-handling contracts.

This reference is intended for:

- Developers extending SensorFusionSim with new sensors, trackers, or visualization backends.
- Test harness authors writing integration and replay tests.
- Reviewers validating that implementation matches the architectural specifications (A1-A9, B1, B3).

> **Note:** All interfaces in this document are *design targets*. Minor signature details may evolve during implementation, but behaviour and contracts must remain consistent.

---

## Table of Contents

1. [Common Types](#1-common-types)
2. [VirtualClock API](#2-virtualclock-api)
3. [CommunicationBus API](#3-communicationbus-api)
4. [SensorManager API](#4-sensormanager-api)
5. [TargetTracker API](#5-targettracker-api)
6. [TrajectorySolver API](#6-trajectorysolver-api)
7. [EngagementController API](#7-engagementcontroller-api)
8. [DataLogger API](#8-datalogger-api)
9. [Visualization API](#9-visualization-api)
10. [Thread-Safety Summary](#10-thread-safety-summary)
11. [Error Handling & Diagnostics](#11-error-handling--diagnostics)

---

## 1. Common Types

All modules share a set of core value types defined in the `sensorfusion` namespace.

### 1.1 SensorFrame

```cpp
namespace sensorfusion {

struct SensorFrame {
    std::chrono::steady_clock::time_point timestamp;

    // IMU data
    Eigen::Vector3f imu_accel;
    Eigen::Vector3f imu_gyro;

    // Simplified single-range LiDAR measurement
    float lidar_range;

    // Fault indicators
    bool dropout_flag;
    bool spike_flag;
    bool stuck_flag;

    // Noise characteristics
    float noise_sigma;
};

} // namespace sensorfusion
```

**Semantics**
- Represents a single synthetic sensor sample (IMU + LiDAR) plus fault metadata.
- Produced exclusively by SensorManager.
- Consumed by TargetTracker.
- Used in hot paths (SPSC ring buffer); must remain trivially copyable in the implementation.

---

### 1.2 TrackerState

```cpp
namespace sensorfusion {

struct TrackerState {
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    float confidence;
};

} // namespace sensorfusion
```

**Semantics**
- Represents the fused pose/velocity estimate for the current virtual time.
- Produced by TargetTracker.
- Consumed by TrajectorySolver.
- `confidence` reflects data quality (faults, noise, recent updates).

---

### 1.3 KinematicSolution

```cpp
namespace sensorfusion {

struct KinematicSolution {
    std::chrono::steady_clock::time_point timestamp;

    // Neutral robotic orientation parameters
    float azimuth_offset;
    float elevation_offset;

    // Stability metrics
    float stability_score;
    bool is_stable;
};

} // namespace sensorfusion
```

**Semantics**
- Produced by TrajectorySolver.
- Consumed by EngagementController and optionally Visualization.
- `stability_score` and `is_stable` express solver confidence and control stability.

---

### 1.4 SystemEvent

```cpp
namespace sensorfusion {

enum class SystemEventType {
    StateTransition,
    FusionUpdate,
    SolverUpdate,
    FaultInjected
};

struct SystemEvent {
    SystemEventType type;
    std::string description;
    std::chrono::steady_clock::time_point timestamp;
};

} // namespace sensorfusion
```

**Semantics**
- Produced primarily by EngagementController.
- Consumed by DataLogger and Visualization.
- Used to represent high-level system and fault events as described in A8 and A9.

---

## 2. VirtualClock API

The virtual clock provides a deterministic timebase for all simulation timestamps.

### 2.1 Class: VirtualClock

```cpp
namespace sensorfusion::time {

class VirtualClock {
public:
    VirtualClock();

    /// Returns the current virtual time.
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept;

    /// Advances the virtual clock by the specified duration.
    void advance(std::chrono::steady_clock::duration delta) noexcept;

    /// Resets the clock back to its initial epoch.
    void reset() noexcept;

private:
    std::chrono::steady_clock::time_point m_current;
};

} // namespace sensorfusion::time
```

**Behaviour**
- `now()` returns a monotonically increasing time point.
- `advance()` adds `delta` to the current time; delta may be positive or zero.
- `reset()` reinitializes the clock to a defined epoch (e.g., zero).

**Thread-Safety**
- In v0.1, VirtualClock is assumed to be used from a single controlling context.
- If shared across threads, external synchronization is required.

---

## 3. CommunicationBus API

The CommunicationBus implements the publish-subscribe backbone with a single worker thread.

### 3.1 Struct: BusConfig

```cpp
namespace sensorfusion::bus {

struct BusConfig {
    bool dropOnOverflow = false;
    std::size_t maxQueueSizePerType = 1024;
};

} // namespace sensorfusion::bus
```

**Semantics**
- `dropOnOverflow`: if true, messages may be dropped when per-type queues are full.
- `maxQueueSizePerType`: soft bound for each message queue.

---

### 3.2 Class: CommunicationBus

```cpp
namespace sensorfusion::bus {

class CommunicationBus {
public:
    using SensorFrameHandler = std::function<void(const sensorfusion::SensorFrame&)>;
    using TrackerStateHandler = std::function<void(const sensorfusion::TrackerState&)>;
    using KinematicSolutionHandler = std::function<void(const sensorfusion::KinematicSolution&)>;
    using SystemEventHandler = std::function<void(const sensorfusion::SystemEvent&)>;

    explicit CommunicationBus(const BusConfig& config = {});

    void start();
    void stop();

    void publish(const sensorfusion::SensorFrame& frame);
    void publish(const sensorfusion::TrackerState& state);
    void publish(const sensorfusion::KinematicSolution& solution);
    void publish(const sensorfusion::SystemEvent& event);

    void subscribe(SensorFrameHandler handler);
    void subscribe(TrackerStateHandler handler);
    void subscribe(KinematicSolutionHandler handler);
    void subscribe(SystemEventHandler handler);

private:
    BusConfig m_config;
    // Internal queues and worker thread details are hidden.
};

} // namespace sensorfusion::bus
```

**Thread-Safety**
- `publish(...)` is thread-safe and may be called concurrently.
- `subscribe(...)` should be called during initialization before `start()`.
- Handlers run on the bus worker thread and must return promptly.

**Ordering Guarantees**
- Per-type FIFO ordering is guaranteed as per A7.
- Causal ordering between message types relies on upstream modules.

---

## 4. SensorManager API

### 4.1 Struct: SensorConfig

```cpp
namespace sensorfusion::sensors {

struct SensorConfig {
    std::string id;
    double updateRateHz = 100.0;
    float noiseSigma = 0.01f;
    // Future fields: drift, bias, etc.
};

} // namespace sensorfusion::sensors
```

### 4.2 Class: SensorManager

```cpp
namespace sensorfusion::sensors {

class SensorManager {
public:
    SensorManager(time::VirtualClock& clock,
                  bus::CommunicationBus& bus);

    void configureSensor(const SensorConfig& config);
    void removeSensor(const std::string& id);
    void start();
    void stop();

private:
    time::VirtualClock& m_clock;
    bus::CommunicationBus& m_bus;
    // Internal state
};

} // namespace sensorfusion::sensors
```

**Behaviour**
- `configureSensor` sets or updates a sensor entry.
- `start()` spawns worker threads that generate SensorFrame instances.
- Frames use `VirtualClock::now()` timestamps and enter the ingestion path.

**Thread-Safety**
- `configureSensor` / `removeSensor` are safe before `start()`; runtime modifications are implementation-defined.
- `start()` / `stop()` must be invoked by a single controlling thread.

---

## 5. TargetTracker API

### 5.1 Struct: TrackerConfig

```cpp
namespace sensorfusion::tracking {

struct TrackerConfig {
    float initialConfidence = 0.0f;
    float maxUpdateIntervalMs = 100.0f;
};

} // namespace sensorfusion::tracking
```

### 5.2 Class: TargetTracker

```cpp
namespace sensorfusion::tracking {

class TargetTracker {
public:
    TargetTracker(const TrackerConfig& config,
                  time::VirtualClock& clock,
                  bus::CommunicationBus& bus);

    void start();
    void stop();

    [[nodiscard]] TrackerState latestState() const;

private:
    TrackerConfig m_config;
    time::VirtualClock& m_clock;
    bus::CommunicationBus& m_bus;

    TrackerState m_lastState;
};

} // namespace sensorfusion::tracking
```

**Behaviour**
- Subscribes to SensorFrame messages and publishes TrackerState.
- Maintains `m_lastState` for synchronous queries.

**Thread-Safety**
- `start()` / `stop()` managed externally.
- `latestState()` is safe for concurrent reads (implementation uses suitable synchronization).

---

## 6. TrajectorySolver API

### 6.1 Struct: SolverConfig

```cpp
namespace sensorfusion::solver {

struct SolverConfig {
    double tickRateHz = 60.0;
    float stabilityThreshold = 0.8f;
};

} // namespace sensorfusion::solver
```

### 6.2 Class: TrajectorySolver

```cpp
namespace sensorfusion::solver {

class TrajectorySolver {
public:
    TrajectorySolver(const SolverConfig& config,
                     time::VirtualClock& clock,
                     bus::CommunicationBus& bus);

    void start();
    void stop();

    [[nodiscard]] KinematicSolution latestSolution() const;

private:
    SolverConfig m_config;
    time::VirtualClock& m_clock;
    bus::CommunicationBus& m_bus;

    KinematicSolution m_lastSolution;
};

} // namespace sensorfusion::solver
```

**Behaviour**
- Subscribes to TrackerState, ticks at `tickRateHz`, and publishes KinematicSolution.

**Thread-Safety**
- `start()` / `stop()` orchestrated externally.
- `latestSolution()` safe for concurrent readers.

---

## 7. EngagementController API

### 7.1 Enum: EngagementState

```cpp
namespace sensorfusion::control {

enum class EngagementState {
    Idle,
    Acquiring,
    Tracking,
    Aligning,
    Ready,
    Executing,
    Completed,
    Safe
};

} // namespace sensorfusion::control
```

### 7.2 Struct: ControllerConfig

```cpp
namespace sensorfusion::control {

struct ControllerConfig {
    float minStabilityToAlign = 0.8f;
    float minConfidenceToTrack = 0.5f;
    std::chrono::milliseconds dataTimeout{500};
};

} // namespace sensorfusion::control
```

### 7.3 Class: EngagementController

```cpp
namespace sensorfusion::control {

class EngagementController {
public:
    EngagementController(const ControllerConfig& config,
                         time::VirtualClock& clock,
                         bus::CommunicationBus& bus);

    void start();
    void stop();

    [[nodiscard]] EngagementState state() const noexcept;

private:
    ControllerConfig m_config;
    time::VirtualClock& m_clock;
    bus::CommunicationBus& m_bus;

    EngagementState m_state{EngagementState::Idle};
};

} // namespace sensorfusion::control
```

**Behaviour**
- Subscribes to KinematicSolution and drives the state machine (Idle -> Acquiring -> ... -> Safe).
- Emits SystemEvent for transitions, faults, and solver updates.

**Thread-Safety**
- `state()` safe for concurrent reads.
- `start()` / `stop()` must not be invoked concurrently.

---

## 8. DataLogger API

### 8.1 Struct: LoggerConfig

```cpp
namespace sensorfusion::logging {

struct LoggerConfig {
    std::string outputPath;
    bool logSensorFrames = true;
    bool logTrackerStates = true;
    bool logKinematicSolutions = true;
    bool logSystemEvents = true;
};

} // namespace sensorfusion::logging
```

### 8.2 Class: DataLogger

```cpp
namespace sensorfusion::logging {

class DataLogger {
public:
    DataLogger(const LoggerConfig& config,
               bus::CommunicationBus& bus);

    void start();
    void stop();

private:
    LoggerConfig m_config;
    bus::CommunicationBus& m_bus;
};

} // namespace sensorfusion::logging
```

**Behaviour**
- Subscribes to SensorFrame, TrackerState, KinematicSolution, SystemEvent.
- Serializes messages for replay (see A6, B3).

**Thread-Safety**
- `start()` / `stop()` orchestrated externally; internal handlers must avoid blocking.

---

## 9. Visualization API

### 9.1 Struct: VisualizationConfig

```cpp
namespace sensorfusion::viz {

struct VisualizationConfig {
    bool enabled = true;
};

} // namespace sensorfusion::viz
```

### 9.2 Class: Visualization

```cpp
namespace sensorfusion::viz {

class Visualization {
public:
    Visualization(const VisualizationConfig& config,
                  bus::CommunicationBus& bus);

    void start();
    void stop();

private:
    VisualizationConfig m_config;
    bus::CommunicationBus& m_bus;
};

} // namespace sensorfusion::viz
```

**Behaviour**
- Subscribes to TrackerState, KinematicSolution, SystemEvent for live rendering.

---

## 10. Thread-Safety Summary

- `CommunicationBus::publish(...)` is thread-safe and non-blocking aside from queue contention.
- `start()` / `stop()` on modules are not re-entrant; call them in a controlled sequence.
- `latestState()`, `latestSolution()`, `EngagementController::state()` are safe for concurrent reads.
- `VirtualClock` is single-owned in v0.1; sharing requires extra synchronization.

Modules follow:
- Immutable-after-publish message semantics.
- Per-module worker threads using `std::jthread`.
- A single CommunicationBus worker to maintain deterministic ordering.

---

## 11. Error Handling & Diagnostics

### 11.1 Parameter Validation
- Public APIs validate configuration where feasible (e.g., negative rates throw `std::invalid_argument`).
- Logging path issues surface at `DataLogger::start()`.

### 11.2 Exceptions
- Constructors or `start()` may throw if configuration is invalid or resources cannot be created.
- Runtime faults (sensor anomalies, queue overflows) produce `SystemEvent` entries per A8 instead of exceptions.

### 11.3 SystemEvent Usage
- `StateTransition` for state changes in EngagementController.
- `FusionUpdate` for notable tracker events (optional).
- `SolverUpdate` for solver-related events (optional).
- `FaultInjected` for fault conditions (sensor flags, timeouts, queue overflows).

All events must be observable via DataLogger and Visualization.

---

## End of B2 - Module API Reference (v0.1)
