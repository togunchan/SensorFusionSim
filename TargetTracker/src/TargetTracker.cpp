#include <TargetTracker/TargetTracker.hpp>

namespace sensorfusion::tracking
{

    TargetTracker::TargetTracker(const TrackerConfig &config,
                                 time::VirtualClock &clock,
                                 bus::CommunicationBus &bus)
        : m_config(config), m_clock(clock), m_bus(bus)
    {
    }

    void TargetTracker::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_bus.subscribe([this](const sensorfusion::SensorFrame &frame)
                        { this->handleSensorFrame(frame); });

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void TargetTracker::stop()
    {
    }

    TrackerState TargetTracker::latestState() const
    {
    }

    void TargetTracker::handleSensorFrame(const SensorFrame &frame)
    {
    }

    void TargetTracker::workerLoop(std::stop_token st)
    {
    }

} // namespace sensorfusion::tracking