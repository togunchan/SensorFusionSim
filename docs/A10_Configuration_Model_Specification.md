# SensorFusionSim - Configuration Model Specification (v0.1)

This document defines the complete configuration model of SensorFusionSim, including schema structure, module namespaces, default-override resolution rules, validation pipeline, and lifecycle behavior. The configuration system ensures deterministic behavior, safe parameterization, and reproducible simulations across all modules.

---

## 1. Design Goals

The configuration model is designed to:
- Provide a single source of truth for all module parameters.
- Support default + user override merging.
- Guarantee type-safe and semantically valid configuration values.
- Enforce module-level namespacing to prevent cross-module leakage.
- Enable deterministic replay through immutable runtime config objects.
- Support future hot-reload extension points.

---

## 2. Configuration File Structure

The configuration model consists of two JSON files:

### 2.1 default_config.json
Included in the repository; contains safe baseline values.

### 2.2 user_config.json (optional)
User-provided overrides applied on top of defaults.

### 2.3 Merged Configuration
`merged = deep_merge(default_config, user_config)`

Deep-merge rules:
- Scalar values are overridden.
- Arrays are replaced.
- Objects are recursively merged.
- Missing fields fall back to default values.

---

## 3. Top-Level Schema Overview

The merged configuration must follow this top-level structure:

```json
{
  "sensors": { ... },
  "tracker": { ... },
  "solver": { ... },
  "controller": { ... },
  "timing": { ... },
  "bus": { ... },
  "logging": { ... }
}
```

Each block corresponds to an internal module in SensorFusionSim.

---

## 4. JSON Schema (Formal Definition)

SensorFusionSim uses JSON Schema Draft-07 for structural validation. A condensed version is shown below:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["sensors", "tracker", "solver", "controller", "timing", "bus", "logging"],

  "properties": {
    "sensors": {
      "type": "object",
      "properties": {
        "imu_rate_hz": { "type": "number", "minimum": 1, "maximum": 1000 },
        "lidar_rate_hz": { "type": "number", "minimum": 1, "maximum": 100 },
        "lidar_max_range": { "type": "number", "minimum": 0.1 }
      },
      "required": ["imu_rate_hz", "lidar_rate_hz", "lidar_max_range"]
    },

    "tracker": {
      "type": "object",
      "properties": {
        "max_velocity": { "type": "number", "minimum": 0 },
        "confidence_decay": { "type": "number", "minimum": 0, "maximum": 1 }
      },
      "required": ["max_velocity", "confidence_decay"]
    },

    "solver": {
      "type": "object",
      "properties": {
        "tick_rate_hz": { "type": "number", "minimum": 1, "maximum": 500 },
        "stability_threshold": { "type": "number", "minimum": 0, "maximum": 1 }
      },
      "required": ["tick_rate_hz", "stability_threshold"]
    },

    "controller": {
      "type": "object",
      "properties": {
        "timeout_ms": { "type": "integer", "minimum": 1 },
        "alignment_required_stability": { "type": "number", "minimum": 0, "maximum": 1 }
      },
      "required": ["timeout_ms", "alignment_required_stability"]
    },

    "timing": {
      "type": "object",
      "properties": {
        "virtual_time_step_ms": { "type": "integer", "minimum": 1 }
      },
      "required": ["virtual_time_step_ms"]
    },

    "bus": {
      "type": "object",
      "properties": {
        "queue_capacity": { "type": "integer", "minimum": 1 },
        "enable_backpressure": { "type": "boolean" }
      },
      "required": ["queue_capacity", "enable_backpressure"]
    },

    "logging": {
      "type": "object",
      "properties": {
        "enable": { "type": "boolean" },
        "directory": { "type": "string" }
      },
      "required": ["enable", "directory"]
    }
  }
}
```

This schema enforces strict correctness and prevents malformed config files.

---

## 5. Module-Level Namespacing Rules

Each module receives only its namespace from the merged configuration:

| Module              | Config Namespace |
|---------------------|------------------|
| SensorManager       | sensors          |
| TargetTracker       | tracker          |
| TrajectorySolver    | solver           |
| EngagementController| controller       |
| VirtualClock        | timing           |
| CommunicationBus    | bus              |
| DataLogger          | logging          |

Modules are forbidden from reading outside their namespace, guaranteeing clean, isolated behavior.

---

## 6. Validation Pipeline

SensorFusionSim uses three layers of validation:

### 6.1 Level 1 - Structural Validation (JSON Schema)
Ensures fields exist, have the right types, and meet min/max rules.

### 6.2 Level 2 - Semantic Validation
Custom rules such as:
- IMU rate must be faster than LiDAR rate.
- `solver.tick_rate_hz` cannot exceed tracker update rate.
- `stability_threshold` in [0, 1].
- `virtual_time_step_ms <= 5` ms (example constraint).

### 6.3 Level 3 - Cross-Module Validation
Ensures the whole system makes sense:
- `solver.tick_rate_hz <= sensors.imu_rate_hz`.
- `controller.timeout_ms >= (1000 / solver.tick_rate_hz)`.
- `queue_capacity` large enough for worst-case sensor bursts.
- `alignment_required_stability >= solver.stability_threshold`.

These rules prevent invalid or misleading configuration states.

---

## 7. Runtime Lifecycle

The configuration system follows a strict lifecycle:

**Load -> Merge -> Validate -> Freeze -> Inject -> Run**

1. **Load**
   - `default_config.json` loaded first.
   - `user_config.json` loaded if present.

2. **Merge**
   - Deep-merge resolves overrides.

3. **Validate**
   - JSON Schema -> semantic checks -> cross-module checks.

4. **Freeze**
   - Config converted into immutable C++ structs.
   - No mutation allowed, ensuring deterministic replay and protecting modules from accidental changes.

5. **Inject**
   - Each module receives only its namespace.

6. **Run**
   - Simulation begins with stable, validated, frozen configuration.

---

## 8. Future Extensions

The configuration model is structured for expansion:
- Hot-reload support with change notification.
- Module-level override files (e.g., `solver_config.json`).
- Parameter introspection API.
- Config visualization UI.
- Config diffing for debugging.

---

## End of A10 - Configuration Model Specification (v0.1)
