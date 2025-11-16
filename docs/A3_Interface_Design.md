# SensorFusionSim - Interface Design (v0.2)

This document defines the data structures, message contracts, and module interface boundaries for SensorFusionSim. All modules communicate through strongly defined data types to ensure deterministic, observable, and extensible system behavior.

---

## 1. SensorFrame

Produced by: SensorManager  
Consumed by: TargetTracker

```cpp
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
```

Description: Represents a single synthetic sensor packet containing IMU + LiDAR readings along with noise and injected-fault metadata.

---

## 2. TrackerState

Produced by: TargetTracker  
Consumed by: TrajectorySolver

```cpp
struct TrackerState {
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    float confidence;
};
```

Description: Contains the fused state estimate, delivering position, velocity, and a confidence score derived from multi-sensor fusion.

---

## 3. KinematicSolution

Produced by: TrajectorySolver  
Consumed by: EngagementController, Visualization

```cpp
struct KinematicSolution {
    std::chrono::steady_clock::time_point timestamp;

    // Neutral robotic orientation parameters
    float azimuth_offset;
    float elevation_offset;

    // Stability metrics
    float stability_score;
    bool is_stable;
};
```

Description: Encapsulates orientation offsets and solver stability information required for downstream system coordination.

---

## 4. SystemEvent

Produced by: EngagementController  
Consumed by: DataLogger, Visualization

```cpp
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
```

Description: Represents system-level events such as state transitions, fault injections, and update notifications.

---

## 5. CommunicationBus - Message Contracts

**Publishers**
- SensorManager -> SensorFrame
- TargetTracker -> TrackerState
- TrajectorySolver -> KinematicSolution
- EngagementController -> SystemEvent

**Subscribers**
- DataLogger -> all message types
- Visualization -> all message types
- EngagementController -> selective subscription (based on current state)

Description: Implements an internal publish-subscribe architecture that transports structured messages among modules without exposing implementation details.

---

## 6. Interface Flow Diagram

```
SensorManager
    |  (SensorFrame)
    v
TargetTracker
    |  (TrackerState)
    v
TrajectorySolver
    |  (KinematicSolution)
    v
EngagementController
    |   (SystemEvent)
    +----------> DataLogger
    +----------> Visualization

CommunicationBus (Pub/Sub backbone)
```

---

## End of A3 Interface Specification

This specification acts as the architectural contract between modules and remains stable unless a major system revision is required.
