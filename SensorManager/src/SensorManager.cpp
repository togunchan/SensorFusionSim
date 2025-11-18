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

    void SensorManager::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        while (!st.stop_requested())
        {
            std::this_thread::sleep_for(10ms);
        }
    }
} // namespace sensorfusion::sensors