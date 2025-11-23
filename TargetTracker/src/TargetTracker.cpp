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
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
            m_worker.request_stop();
    }

    TrackerState TargetTracker::latestState() const
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_lastState;
    }

    void TargetTracker::handleSensorFrame(const SensorFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);

        m_lastState.timestamp = frame.timestamp;
        m_lastState.confidence = std::min(1.0f, m_lastState.confidence + 0.1f);

        m_lastUpdate = frame.timestamp;
    }

    void TargetTracker::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        while (!st.stop_requested())
        {
            auto now = m_clock.now();
            {
                std::lock_guard<std::mutex> lock(m_stateMutex);

                auto delta = now - m_lastUpdate;

                // // Decrease confidence if updates have been missing for an extended period
                if (delta > std::chrono::milliseconds(static_cast<int>(m_config.maxUpdateInterval)))
                {
                    m_lastState.confidence = std::max(0.0f, m_lastState.confidence - 0.05f);
                }

                m_bus.publish(m_lastState);
            }

            std::this_thread::sleep_for(10ms);
            m_clock.advance(10ms);
        }
    }

} // namespace sensorfusion::tracking