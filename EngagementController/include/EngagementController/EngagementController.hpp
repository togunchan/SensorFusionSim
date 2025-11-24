#pragma once

#include <VirtualTime/VirtualClock.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace sensorfusion::control
{
    enum class EngagementState
    {
        Idle,
        Acquiring,
        Tracking,
        Aligning,
        Ready,
        Executing,
        Completed,
        Safe
    };

    struct ControllerConfig
    {
        float minStabilityToAlign = 0.8f;
        float minConfidenceToTrack = 0.5f;
        std::chrono::milliseconds dataTimeout{500};
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
        void transitionTo(EngagementState newState);

        ControllerConfig m_config;
        time::VirtualClock &m_clock;
        bus::CommunicationBus &m_bus;

        std::jthread m_worker;
        std::atomic<bool> m_running{false};

        mutable std::mutex m_mutex;
        EngagementState m_state{EngagementState::Idle};

        KinematicSolution m_lastSolution{};
        std::chrono::steady_clock::time_point m_lastUpdate{};
    };
} // namespace sensorfusion::control
