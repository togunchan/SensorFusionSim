#include <SensorManager/SensorManager.hpp>
#include <chrono>

namespace sensorfusion::sensors
{
    SensorManager::SensorManager(time::VirtualClock &clock, bus::CommunicationBus &bus) : m_clock(clock), m_bus(bus) {}

    void SensorManager::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void SensorManager::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
        {
            m_worker.request_stop();
            m_worker.join();
        }
    }

    void SensorManager::addSensor(const SensorConfig &config)
    {
        m_sensors[config.id] = config;
    }

    void SensorManager::removeSensor(const std::string &id)
    {
        m_sensors.erase(id);
    }

    void SensorManager::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        while (!st.stop_requested())
        {
            auto now = m_clock.now();

            for (auto &[id, config] : m_sensors)
            {
                auto &nextTime = m_nextUpdate[id];
                if (now >= nextTime)
                {
                    sensorfusion::SensorFrame frame{};
                    frame.timestamp = now;
                    m_bus.publish(frame);

                    const auto interval = std::chrono::duration_cast<time::VirtualClock::Duration>(
                        std::chrono::duration<double>(1.0 / config.updateRateHz));
                    nextTime = now + interval;
                }
            }
            std::this_thread::sleep_for(1ms);
            m_clock.advance(1ms);
        }
    }
} // namespace sensorfusion::sensors
