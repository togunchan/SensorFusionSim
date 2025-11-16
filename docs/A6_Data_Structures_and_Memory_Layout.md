# SensorFusionSim - Data Structures & Memory Layout (v0.1)

This document defines the data representation, memory layout, and alignment rules for the core data structures of SensorFusionSim. The goal is to keep hot-path types simple, cache-friendly, and compatible with lock-free and deterministic execution.

---

## 1. Scope & Goals

The data structures in SensorFusionSim are designed to:

- Be **trivially copyable** wherever possible.
- Avoid dynamic allocation and virtual dispatch in hot paths.
- Maintain a **predictable memory layout** suitable for:
  - SPSC ring buffers
  - Lock-free or low-contention queues
  - Deterministic logging and replay
- Respect **Eigen** alignment constraints where needed.
- Minimize **false sharing** and cache line contention.

This specification applies primarily to:

- `SensorFrame`
- `TrackerState`
- `KinematicSolution`
- `SystemEvent`
- Hot-path containers such as the sensor -> tracker SPSC buffer.

---

## 2. General Design Rules

1. **POD-first design**

   - Core simulation structs (`SensorFrame`, `TrackerState`, `KinematicSolution`, `SystemEvent`) must:
     - Use only fundamental types, `std::chrono` timestamps, `Eigen` fixed-size vectors, and `std::string` where justified.
     - Avoid inheritance and virtual functions.
     - Avoid owning dynamic memory in hot paths.

2. **Trivially copyable where feasible**

   - Types passed through lock-free queues and SPSC ring buffers should be trivially copyable or move-only with trivial moves.
   - If a type becomes non-trivial, it should not be used directly as a ring buffer element without explicit review.

3. **No implicit padding assumptions**

   - Code must not depend on `sizeof(T)` being stable across compilers.
   - Alignment-sensitive behavior is expressed via `alignas` and documented here.

4. **Separation of hot vs cold fields**

   - Frequently accessed, time-critical fields are grouped together.
   - Rarely accessed metadata should be kept out of hot-path structs when possible or at least placed after hot fields.

---

## 3. Core Simulation Types

This section formalizes the intended layout properties of the key structs. C++ examples here are illustrative, not normative definitions.

### 3.1 SensorFrame

```cpp
struct SensorFrame {
    std::chrono::steady_clock::time_point timestamp;

    // IMU data
    Eigen::Vector3f imu_accel;
    Eigen::Vector3f imu_gyro;

    // Single-range LIDAR measurement
    float lidar_range;

    // Fault indicators
    bool dropout_flag;
    bool spike_flag;
    bool stuck_flag;

    // Noise characteristics
    float noise_sigma;
};
```

**Intent:**
- Represents one synthetic sensor sample.
- Used as the hot-path payload in the sensor -> tracker SPSC ring buffer.

**Layout rules:**
- SensorFrame is treated as a POD-style struct with no custom constructors, destructors, or virtual functions in the hot implementation.
- Eigen vectors use fixed-size `Eigen::Vector3f` to keep layout simple and avoid dynamic allocation.
- Any container storing SensorFrame (e.g., `std::vector<SensorFrame>`) must follow Eigen alignment guidelines if needed.

### 3.2 TrackerState

```cpp
struct TrackerState {
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    float confidence;
};
```

**Intent:** Contains the fused estimate used downstream by TrajectorySolver.

**Layout rules:**
- Same POD-style constraints as SensorFrame.
- `position` and `velocity` are hot fields and should remain adjacent for better spatial locality.
- `confidence` follows as a simple scalar.

### 3.3 KinematicSolution

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

**Intent:** Summarizes solver output: orientation offsets and stability.

**Layout rules:**
- Simple scalar-only struct.
- Kept trivially copyable to be easily routed via CommunicationBus.

### 3.4 SystemEvent

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

**Intent:** Represents high-level system events for logging and visualization.

**Layout rules:**
- SystemEvent is not a strict hot-path type and may own a `std::string` description.
- Still designed to remain reasonably lightweight and suitable for queued delivery via CommunicationBus.

---

## 4. Ring Buffer & Queue Layout

### 4.1 Sensor -> Tracker SPSC Ring Buffer

The SPSC buffer is designed for high-frequency, low-latency ingestion.

```cpp
template <typename T>
struct alignas(64) SpscIndices {
    std::atomic<std::size_t> head; // consumer-owned
    char pad1[64 - sizeof(std::atomic<std::size_t>)];

    std::atomic<std::size_t> tail; // producer-owned
    char pad2[64 - sizeof(std::atomic<std::size_t>)];
};

template <typename T, std::size_t Capacity>
struct SpscRingBuffer {
    SpscIndices<T> indices;
    T storage[Capacity];
};
```

**Layout rules:**
- `head` and `tail` are placed in separate cache lines using `alignas(64)` and padding to minimize false sharing.
- `storage` is an array of `T`, typically `SensorFrame`.

### 4.2 CommunicationBus Queues

- Each message type (SensorFrame, TrackerState, KinematicSolution, SystemEvent) has its own queue.
- Initial implementation can use `std::mutex` + `std::condition_variable` + `std::deque<T>`.
- Future optimization: replace with lock-free queues for specific types if profiling suggests it.

---

## 5. Eigen Usage & Alignment

SensorFusionSim uses Eigen for vector math:
- Primary types: `Eigen::Vector3f` for position, velocity, and IMU data.
- General rules:
  - Prefer fixed-size Eigen types to avoid heap allocations.
  - When storing Eigen types inside user-defined structs, be mindful of alignment. Use `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` or aligned allocators if needed.
  - For hot-path containers (e.g., ring buffers), test alignment behavior on target compilers.

The default assumption is that compilers and standard libraries handle fixed-size, stack-allocated Eigen vectors correctly for typical uses. If misalignment is observed, this section will be expanded with stricter requirements.

---

## 6. Logging & Replay Layout

Logging is designed to respect struct field order:
- When logging `SensorFrame`, columns should follow the in-memory field order (timestamp, `imu_accel.x`, `imu_accel.y`, `imu_accel.z`, `imu_gyro.x`, ...).
- `TrackerState` and `KinematicSolution` similarly map directly to column sequences.

This makes it easier to:
- Replay scenarios by reading logs and reconstructing structs.
- Compare logs to in-memory behavior during debugging.

SystemEvent logs may include:
- `timestamp`
- `type`
- `description`

---

## 7. Future Extensions

The following extensions are anticipated:
- Separate "cold" metadata structs for rarely accessed fields.
- Support for additional sensor types with different data layouts.
- More explicit Eigen alignment rules and allocator requirements.
- Cache-aware layout tuning guided by profiling on real workloads.

---

## End of A6 - Data Structures & Memory Layout (v0.1)
