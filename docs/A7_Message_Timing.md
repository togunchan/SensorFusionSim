# SensorFusionSim - Message Timing & Ordering Guarantees (v0.1)

This document defines the timing, ordering, and delivery guarantees of SensorFusionSim's internal message transport system. The goal is to ensure deterministic, reproducible, and causally consistent execution across all modules in the real-time simulation pipeline.

These guarantees apply to the following message types:
- SensorFrame
- TrackerState
- KinematicSolution
- SystemEvent

---

## 1. Timing Definitions

Every message has up to three relevant timestamps.

### 1.1 Production Timestamp

The time assigned when the message is created, derived from the virtual simulation clock.

Example: `SensorManager -> SensorFrame.timestamp`

### 1.2 Enqueue Timestamp

The moment the message is inserted into the CommunicationBus queue, used to maintain per-type FIFO ordering.

### 1.3 Delivery Timestamp

The moment the CommunicationBus worker thread delivers the message to subscribers. It is not guaranteed to equal the enqueue timestamp, but ordering constraints still apply.

---

## 2. Timing Guarantees

**Guarantee 1 - Production timestamp always reflects virtual simulation time**
- Messages never use wall-clock time internally.
- Ensures deterministic replay and consistent solver behavior independent of real-time scheduling variation.

**Guarantee 2 - Enqueue time does not affect deterministic replay**
- Ordering is derived from production time, enqueue sequence, and per-type FIFO rules, not from wall-clock scheduling of producer threads.

**Guarantee 3 - Delivery threads never reorder messages of the same type**
- The CommunicationBus worker enforces message-type-level determinism.

---

## 3. Ordering Guarantees

### 3.1 Per-Type FIFO Ordering (Strict)

Messages of the same type are delivered in the exact order they were enqueued:

```
SensorFrame #12
SensorFrame #13
SensorFrame #14
```

Delivery will always be `12 -> 13 -> 14`, never out-of-order.

### 3.2 Cross-Type Ordering (Loose)

Different message types do not share a global ordering. For example, the sequence `SensorFrame #20`, `TrackerState #9`, `SensorFrame #21`, `SystemEvent #3` is valid. This keeps the system decoupled, fast, and predictable inside each type.

### 3.3 Causal Ordering (Hard Guarantee)

If message B is computed based on message A, then B is never delivered before A. This applies to:
- SensorFrame -> TrackerState
- TrackerState -> KinematicSolution
- KinematicSolution -> SystemEvent

### 3.4 No Cross-Thread Interleaving That Violates Causality

Even if producer threads run at different speeds:
- TargetTracker never publishes a state based on a frame that was not yet delivered.
- TrajectorySolver never receives a state before the bus has processed the corresponding frame.

---

## 4. Delivery Guarantees

### 4.1 Single Bus Worker = Deterministic Fan-Out

All message fan-out is performed by one dedicated `std::jthread`.

Consequences:
- Delivery order is stable.
- Only one thread touches subscriber callbacks at a time.
- Subscribers receive messages in consistent sequences.

### 4.2 Subscribers Are Invoked Synchronously

During delivery, the bus worker invokes handlers directly. No subscriber is invoked concurrently unless it creates its own worker thread.

### 4.3 Delivery Latency Is Bounded and Predictable

Because the bus worker processes per-type FIFO queues using a single-threaded dispatcher with blocking waits (`std::condition_variable`), delivery jitter remains extremely low.

---

## 5. Replay Determinism Guarantees

When logs are replayed:
- Production timestamps match the original execution.
- Enqueued messages follow the original ordering.
- The bus worker reconstructs identical delivery ordering.
- Controller state machine transitions occur in the same order.
- Visualization and DataLogger receive identical sequences.

Replay determinism is a core contract of SensorFusionSim.

---

## 6. Failure Handling Rules

### 6.1 Queue Overflows

If a ring buffer or message queue overflows:
- The simulator may drop messages only if explicitly configured.
- Dropped messages generate a `SystemEvent`.

### 6.2 Subscriber Exceptions

If a subscriber throws:
- The exception is caught inside the bus worker.
- A `SystemEvent` with `type = FaultInjected` is emitted.
- Simulation continues unless explicitly configured to halt, preventing corrupted ordering chains.

---

## 7. Summary of Guarantees

| Guarantee Type          | Description                                          |
|-------------------------|------------------------------------------------------|
| FIFO Ordering           | Per-type promise that no reorder occurs.             |
| Causal Ordering         | Downstream messages never arrive before upstream ones.|
| Deterministic Replay    | Same input = same output = same ordering.            |
| Virtual Clock Consistency | All timestamps derive from the same simulation timebase. |
| Single-Threaded Delivery| Message fan-out is globally stable and reproducible. |

These guarantees form the foundation of SensorFusionSim's determinism, reliability, and debuggability.

---

## End of A7 - Message Timing & Ordering Guarantees (v0.1)
