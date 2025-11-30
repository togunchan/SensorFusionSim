#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <thread>
#include <mutex>

namespace sensorfusion::tracking
{
    struct TrackerConfig
    {
        float initialConfidence = 0.4f;
        float maxUpdateInterval = 200.0f; // milliseconds
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
        float m_theta = 0.0f;
        bool m_hasHeading = false;

        // Kalman filter state
        Eigen::Vector4f m_x{Eigen::Vector4f::Zero()}; // [px, py, vx, vy]^T
        Eigen::Matrix4f m_P{Eigen::Matrix4f::Identity()};
        Eigen::Vector2f m_lastAccel{Eigen::Vector2f::Zero()};

        SensorFrame m_latestFrame{};
        bool m_pendingFrame{false};
        float m_confidence{0.0f};

        time::VirtualClock::TimePoint m_lastPredict{};
        time::VirtualClock::TimePoint m_lastUpdate{};
        time::VirtualClock::TimePoint m_lastHeadingUpdate{};
    };

} // namespace sensorfusion::tracking
