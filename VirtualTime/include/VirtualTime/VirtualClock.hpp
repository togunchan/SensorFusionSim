#pragma once

#include <chrono>

namespace sensorfusion::time
{
    /// All timestamps inside SensorFusionSim should be taken from this clock,
    /// not from std::chrono::steady_clock::now().
    class VirtualClock
    {
    public:
        using TimePoint = std::chrono::steady_clock::time_point;
        using Duration = std::chrono::steady_clock::duration;

        /// Constructs the clock at a well-defined epoch which is zero.
        VirtualClock() noexcept;

        // Returns current virtual time
        [[nodiscard]] TimePoint now() const noexcept;

        /// Advances the virtual time by the given duration.
        /// Negative deltas are ignored
        void advance(Duration delta) noexcept;

        /// Resets the clock back to the initial epoch.
        void reset() noexcept;

    private:
        TimePoint m_current;
    };
} // namespace sensorfusion::time
