# SensorFusionSim - Coding Guidelines & Style Rules (v0.1)

This document defines the coding standards and style rules for the SensorFusionSim project. All C++ code, tests, and supporting utilities must follow these guidelines to ensure consistency, readability, and long-term maintainability.

These rules apply to:
- All C++20 source files under `SensorFusionSim/`
- All public headers and module APIs
- All tests, examples, and utilities

---

## 1. General Principles

1. **Clarity over cleverness**
   - Prefer simple, readable code over "smart" tricks.
   - If a construct is hard to explain in a code review, it does not belong here.
2. **Determinism first**
   - This is a real-time simulation platform. Any change that risks determinism or reproducibility must be carefully justified and documented.
3. **RAII everywhere**
   - Ownership, lifetimes, threads, and resources must be controlled via RAII objects whenever possible.
4. **Minimal dependencies**
   - Each module should expose a small, focused public interface and depend on as few other modules as possible.
5. **Fail fast in development, fail safe in production**
   - Use assertions and strict checks in debug builds; provide graceful handling in release builds where appropriate.

---

## 2. Language & Standard

1. **C++ Standard**
   - All code targets C++20.
   - Use standard library facilities before introducing custom implementations.
2. **Standard Library Usage**
   - Prefer `std::unique_ptr`, `std::shared_ptr` only where shared ownership is required.
   - Use `std::optional` for nullable values instead of raw pointers where ownership is not implied.
   - Use `<chrono>` for all time measurements and timestamps.
   - Use `<thread>`, `<stop_token>`, `<mutex>`, `<condition_variable>`, `<atomic>` for concurrency.
3. **Eigen Usage**
   - Use fixed-size types such as `Eigen::Vector3f` and avoid dynamic allocations in hot paths.
   - Respect Eigen alignment requirements where relevant.

---

## 3. File Organization & Structure

1. **Directory Layout**
   - Each module has its own `include/` and `src/` directories (e.g., `SensorManager/include/SensorManager/...`).
2. **Header/Source Pairing**
   - Public interfaces go into headers under `include/`.
   - Implementation details remain in `src/` and are not exposed to other modules.
3. **One Major Class per File**
   - Each major class or component should reside in its own file named after the class (e.g., `SensorManager/include/SensorManager/SensorManager.hpp`).
4. **Include Guards**
   - Use `#pragma once` in all headers.

---

## 4. Naming Conventions

1. **Files**
   - Headers: `PascalCase.hpp`
   - Sources: `PascalCase.cpp`
   - Tests: `Test_ModuleName.cpp` or similar.
2. **Namespaces**
   - Top-level: `sensorfusion`
   - Sub-namespaces per module (e.g., `sensorfusion::sensors`, `sensorfusion::tracker`).
3. **Classes & Structs**
   - Use PascalCase (e.g., `SensorManager`).
4. **Functions**
   - Use camelCase (e.g., `start()`, `requestStop()`).
5. **Member Variables**
   - Use `m_` prefix and camelCase (e.g., `m_thread`). Configuration structs may use plain names (e.g., `tickRateHz`).
6. **Constants**
   - Use `SCREAMING_SNAKE_CASE` (e.g., `MAX_SENSOR_COUNT`).
7. **Enums**
   - Enum types: PascalCase.
   - Enum values: PascalCase or `SCREAMING_SNAKE_CASE` depending on readability.

---

## 5. Comments & Documentation

1. **Language**
   - All comments and documentation are in English.
2. **Doxygen Style**
   - Public APIs should use Doxygen-style comments:

```cpp
/// Updates the tracker state using the latest SensorFrame.
/// @param frame Latest sensor fusion input.
void update(const SensorFrame& frame);
```

3. **Comment Philosophy**
   - Comment the "why", not the "what". Avoid restating obvious behavior.

---

## 6. Formatting & Layout

1. **Indentation**
   - Use 4 spaces per indentation level. No tabs.
2. **Line Length**
   - Soft limit around 100-120 characters.
3. **Braces**
   - Use Allman or K&R consistently (project may choose one and stick to it). Example (K&R):

```cpp
if (condition) {
    doSomething();
} else {
    doSomethingElse();
}
```

4. **Include Order**
   - Own header -> module headers -> other project headers -> standard library -> third-party.

---

## 7. Ownership & Memory Management

1. **Raw Pointers**
   - Avoid raw owning pointers. Non-owning pointers are allowed only when clearly documented and short-lived.
2. **Smart Pointers**
   - Use `std::unique_ptr` for exclusive ownership; `std::shared_ptr` only when shared ownership is required.
3. **References**
   - Use references for required non-null dependencies when lifetime is guaranteed externally.
4. **No new/delete in business logic**
   - Never call `new`/`delete` directly in high-level logic. Use RAII wrappers or containers.

---

## 8. Error Handling

1. **Exceptions**
   - Use exceptions for unrecoverable errors or configuration failures; do not use them for regular control flow.
2. **Return Codes**
   - Use `bool`, `std::optional`, or dedicated result types for recoverable conditions.
3. **Initialization Failures**
   - Constructors or `init()` may throw if configuration is invalid. Fail fast during startup.
4. **EngagementController & Fault Handling**
   - Faults must propagate as `SystemEvent` entries (see A8). Error handling should be explicit and observable.

---

## 9. Logging & Diagnostics

1. **Purpose**
   - Logging is for diagnostics and replay, not for normal control flow.
2. **Log Levels**
   - At minimum: Debug, Info, Warning, Error, Critical.
   - Avoid excessive logging in hot paths unless guarded by runtime or compile-time switches.
3. **Consistency**
   - Messages should be structured, concise, and include context (module, state, identifiers).

---

## 10. Concurrency & Threads

1. **Thread Creation**
   - Use `std::jthread` for long-lived worker threads; observe stop tokens periodically.
2. **Synchronization**
   - Prefer lock-free structures when justified (e.g., SPSC buffer). Otherwise use `std::mutex`/`std::condition_variable` with clear ownership rules.
3. **Shared State**
   - Shared mutable state must be protected or avoided; immutable-after-publish messages are preferred.
4. **No Ad-Hoc Threads**
   - Do not spawn unmanaged threads. All threads must be owned by a module and follow the lifecycle in A11.

---

## 11. Module Boundaries & Dependencies

1. **Single Responsibility**
   - Each module handles a single conceptual area (e.g., SensorManager -> sensor generation, TargetTracker -> state estimation).
2. **Dependency Direction**
   - High-level modules may depend on lower-level services, not vice versa; avoid cyclic dependencies.
3. **Interfaces over Implementations**
   - Public APIs should depend on abstract interfaces or value types rather than concrete implementations.

---

## 12. Testing & Assertions

1. **Unit Tests**
   - Each major class or subsystem should have dedicated tests.
2. **Assertions**
   - Use `assert()` for invariants that must hold in all sane builds, but do not rely solely on assertions for user-facing validation.
3. **Deterministic Tests**
   - Use the virtual clock and replay mechanisms to avoid time-dependent flakiness.

---

## 13. Forbidden Patterns

The following patterns are not allowed:
1. **Global Mutable State**
   - No global non-const variables; use dependency injection.
2. **Naked new/delete**
   - Manual memory management in business logic is forbidden.
3. **Mission-Critical Logic in Constructors**
   - Heavy logic or thread spawning should occur in explicit `start()` calls, not constructors.
4. **Thread-Safety Assumptions Without Documentation**
   - Thread-safety guarantees (or lack thereof) must be documented.
5. **Silent Failures**
   - Do not swallow exceptions or errors without logging or `SystemEvent` generation.

---

## 14. Examples

**Good Example**

```cpp
class TargetTracker {
public:
    explicit TargetTracker(const TrackerConfig& config);

    /// Updates internal state based on the latest sensor frame.
    void update(const SensorFrame& frame);

    /// Returns the last computed tracker state.
    TrackerState latestState() const;

private:
    TrackerConfig m_config;
    TrackerState m_state;
};
```

**Bad Example**

```cpp
class TargetTracker {
public:
    TargetTracker(); // unclear dependencies

    void doStuff(SensorFrame* f); // unclear ownership

    TrackerState* getState(); // unclear lifetime
};
```

---

## End of B1 - Coding Guidelines & Style Rules (v0.1)
