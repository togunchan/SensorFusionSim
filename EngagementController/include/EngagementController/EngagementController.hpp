#pragma once

#include <VirtualTime/VirtualClock.hpp>
#include <SensorFusionSim/Messages.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace sensorfusion::control
{
    struct ControllerConfig
    {
        float minStabilityToAlign = 0.8f;
        float minConfidenceToTrack = 0.5f;
        std::chrono::milliseconds dataTimeout{500};
        std::chrono::milliseconds minDwell{700};
        std::chrono::milliseconds heartbeat{200};
        std::chrono::milliseconds safeRecovery{700};
    };

    class EngagementController
    {
    public:
        EngagementController(const ControllerConfig &config,
                             time::VirtualClock &clock,
                             bus::CommunicationBus &bus);

        void start();
        void stop();

        [[nodiscard]] EngagementState state() const noexcept;

    private:
        void handleSolution(const KinematicSolution &sol);
        void workerLoop(std::stop_token st);
        bool setState(EngagementState newState, time::VirtualClock::TimePoint now, bool forcePublish = false);
        void publishLocked(EngagementState state, time::VirtualClock::TimePoint now);

        ControllerConfig m_config;
        time::VirtualClock &m_clock;
        bus::CommunicationBus &m_bus;

        std::jthread m_worker;
        std::atomic<bool> m_running{false};

        mutable std::mutex m_mutex;
        EngagementState m_state{EngagementState::Idle};
        EngagementState m_lastPublished{EngagementState::Idle};
        bool m_hasSolution{false};

        KinematicSolution m_lastSolution{};
        std::chrono::steady_clock::time_point m_lastUpdate{};
        std::chrono::steady_clock::time_point m_lastPublish{};
        std::chrono::steady_clock::time_point m_stateEntered{};
    };
} // namespace sensorfusion::control
