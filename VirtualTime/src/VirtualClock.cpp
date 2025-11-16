#include <VirtualTime/VirtualClock.hpp>

namespace sensorfusion::time
{
    VirtualClock::VirtualClock() noexcept : m_current(TimePoint{}) {} // start at zero epoch

    VirtualClock::TimePoint VirtualClock::now() const noexcept
    {
        return m_current;
    }

    void VirtualClock::advance(Duration delta) noexcept
    {
        if (delta < Duration::zero())
        {
            return;
        }

        m_current += delta;
    }

    void VirtualClock::reset() noexcept
    {
        m_current = TimePoint{};
    }

} // namespace sensorfusion::time