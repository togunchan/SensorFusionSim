# SensorFusionSim - Testing & Replay Strategy (v0.1)

This document defines the testing philosophy, test levels, and replay strategy for SensorFusionSim.  
It complements the architectural specifications (A4-A9) and the module API reference (B2) by describing **how** the system will be verified in practice.

The goals are:

- Validate correctness of core algorithms and data structures.
- Ensure concurrency and timing guarantees remain intact over time.
- Exercise fault propagation and safety behaviour end-to-end.
- Provide deterministic replay for debugging and regression testing.

Testing is structured into multiple levels so that individual parts can be validated independently before exercising the full pipeline.

---

## 1. Objectives & Scope

### 1.1 Primary Objectives

SensorFusionSim's testing and replay strategy aims to:

- Confirm that **SensorFrame -> TrackerState -> KinematicSolution -> SystemEvent** flows behave as specified.
- Verify that the **thread model, message ordering, and timing guarantees** (A4, A7, A9) hold under realistic load.
- Validate **fault propagation** paths (A8) for dropout, spike, stuck, and overflow scenarios.
- Enable **deterministic replay** of recorded scenarios for debugging and regression.

### 1.2 Out of Scope (v0.1)

- Performance benchmarking under extreme load.
- Long-duration endurance testing (hours/days).
- Hardware-in-the-loop or real sensor integration.

These can be added incrementally in future revisions.

---

## 2. Test Levels Overview

SensorFusionSim uses a layered testing strategy:

1. **Level 0 - Build & Sanity Checks**
2. **Level 1 - Unit Tests (Pure Logic & Small Types)**
3. **Level 2 - Component Tests (Per-Module with Fakes)**
4. **Level 3 - Integration Tests (End-to-End Pipeline)**
5. **Level 4 - Replay & Regression Suites**

Each level builds on the previous one.

### 2.1 Level 0 - Build & Sanity Checks

- Ensure that all modules compile with C++20.
- Simple "smoke" runs that start and stop the simulation without assertions.
- Optional static analysis or linters can be added later.

### 2.2 Level 1 - Unit Tests

Focus on **small, deterministic pieces**:

- Mathematical helpers (e.g., orientation calculations, stability scoring).
- Data structure utilities (e.g., SPSC ring buffer indices logic).
- VirtualClock behaviour (`advance`, `reset`, monotonicity).
- Simple configuration parsing logic (if present).

These tests must:

- Be fully deterministic.
- Avoid threads and real time.
- Use small, direct assertions.

### 2.3 Level 2 - Component Tests

Test **each module in isolation** with lightweight fakes:

- `SensorManager` with a fake clock and fake output sink.
- `TargetTracker` with a synthetic `SensorFrame` feed.
- `TrajectorySolver` with synthetic `TrackerState` sequences.
- `EngagementController` with scripted `KinematicSolution` inputs.

Goals:

- Validate each module's behaviour without the entire pipeline.
- Verify error handling and fault handling in isolation.
- Confirm that public API contracts from B2 are respected.

### 2.4 Level 3 - Integration Tests

End-to-end tests exercising:

- SensorManager -> TargetTracker -> TrajectorySolver -> EngagementController plus DataLogger and Visualization.

Integration tests use:

- Real CommunicationBus instance.
- Real threads (jthreads) per module.
- VirtualClock driving timestamps.
- Short, controlled scenarios (e.g., 2-5 seconds of simulated time).

These tests:

- Assert on final states (e.g., EngagementController reaches `Ready`).
- Assert on message sequences and ordering.
- Validate that no deadlocks occur during startup or shutdown.

### 2.5 Level 4 - Replay & Regression Suites

Replay tests focus on:

- Loading recorded logs (SensorFrame, TrackerState, KinematicSolution, SystemEvent).
- Reconstructing scenarios using the replay engine.
- Verifying that high-level outcomes match the original run.

These tests form the backbone of **long-term regression**:

- Algorithm changes must preserve replay suites.
- Bugs found in the field become replay scenarios.

---

## 3. Test Infrastructure & Layout

### 3.1 Directory Layout

```
SensorFusionSim/
|-- docs/
|   |-- B3_Testing_and_Replay_Strategy.md
|-- tests/
|   |-- unit/
|   |   |-- Test_VirtualClock.cpp
|   |   |-- Test_SpscRingBuffer.cpp
|   |   |-- ...
|   |-- component/
|   |   |-- Test_SensorManager.cpp
|   |   |-- Test_TargetTracker.cpp
|   |   |-- ...
|   |-- integration/
|       |-- Test_FullPipeline_StableScenario.cpp
|       |-- Test_FullPipeline_FaultInjection.cpp
|       |-- Test_Replay_Scenarios.cpp
```

### 3.2 VirtualClock in Tests

- Treat VirtualClock as the single source of truth for time.
- Unit tests manually call `advance()` and assert on `now()`.
- Component tests inject a VirtualClock instance into modules.
- Integration tests control VirtualClock evolution (e.g., periodic advance calls).

---

## 4. Replay Strategy

### 4.1 Recording

- Log SensorFrame, TrackerState, KinematicSolution, and SystemEvent with timestamps and fields.
- DataLogger stores the data in a structured format (JSON or MiniDB rows).

### 4.2 Replaying

- Replay runner reads logs sequentially, recreates messages, and feeds them into a "replay mode" pipeline.
- Threads may be simplified; VirtualClock advances according to recorded timestamps.
- CommunicationBus ordering (A7) must be respected.

### 4.3 Determinism Criteria

- `SystemEvent` sequences match the original run.
- Engagement state transitions (Idle -> ... -> Safe) match exactly.
- Key numerical outputs match within tolerances.

---

## 5. Fault Injection Tests

### 5.1 Sensor-Level Faults

- Dropout (dropout_flag == true).
- Spike (spike_flag == true).
- Stuck (repeated identical values).

Expected:

- TargetTracker reduces confidence.
- TrajectorySolver lowers stability scores.
- EngagementController may transition to `Safe`.
- DataLogger records the fault chain.

### 5.2 Bus-Level Overflows

- Simulate saturating queues when `dropOnOverflow == true`.
- Expect SystemEvent with overflow metadata.
- System transitions according to A8/A9 (continue or go to `Safe`).

---

## 6. Concurrency & Timing Tests

### 6.1 Ordering Tests

- Publish known sequences of SensorFrame messages.
- Ensure TrackerState order matches and KinematicSolution never jumps ahead.
- Introduce delays to verify FIFO and causality remain intact.

### 6.2 Timing Tolerance Tests

- Use VirtualClock to simulate slow sensors or trackers.
- Validate timeouts, Safe state transitions, and SystemEvent emissions.

---

## 7. Example Test Scenarios

### 7.1 Stable Tracking Scenario

- No faults; expect high confidence, stability, and state reaching Ready/Executing.

### 7.2 Intermittent Dropout Scenario

- Periodic dropout flags; expect oscillating confidence and eventual Safe transition if thresholds exceeded.

### 7.3 Sudden Spike Scenario

- Inject spikes; expect delayed or reverted transitions due to stability drops.

### 7.4 Replay of a Recorded Fault Run

- Replay recorded faults; expect identical SystemEvent sequences and stability metrics.

---

## 8. Execution & Automation

### 8.1 Local Execution

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
ctest --test-dir build -L unit
ctest --test-dir build -L integration
```

### 8.2 Continuous Integration (Future)

- Build in Release with debug info.
- Run all tests on each push/merge request.
- Add jobs for long-running replay/regression suites as needed.

---

## 9. Roadmap for Testing Enhancements

- Property-based testing for mathematical components.
- Stress testing for CommunicationBus queues.
- Randomized fault campaigns.
- Performance benchmarks for latency/throughput.
- Coverage reporting to guide additional tests.

---

## End of B3 - Testing & Replay Strategy (v0.1)
