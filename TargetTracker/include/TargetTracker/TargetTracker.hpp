#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <atomic>
#include <thread>
#include <mutex>

namespace sensorfusion::tracking
{
    struct TrackerConfig
    {
        float initialConfidence = 1.0f;
        float maxUpdateInterval = 100.0f;
    };

    class TargetTracker
    {
    public:
        TargetTracker(const TrackerConfig &config,
                      time::VirtualClock &clock,
                      bus::CommunicationBus &bus);

        void start();
        void stop();

        [[nodiscard]] TrackerState latestState() const;

    private:
        void handleSensorFrame(const SensorFrame &frame);
        void workerLoop(std::stop_token st);

        TrackerConfig m_config;
        time::VirtualClock &m_clock;
        bus::CommunicationBus &m_bus;

        std::jthread m_worker;
        std::atomic<bool> m_running{false};

        mutable std::mutex m_stateMutex;
        TrackerState m_lastState{};
        std::chrono::steady_clock::time_point m_lastUpdate{};
    };

} // namespace sensorfusion::tracking