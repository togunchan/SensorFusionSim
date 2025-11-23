#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>

#include <atomic>
#include <thread>
#include <mutex>

namespace sensorfusion::solver
{
    struct SolverConfig
    {
        double tickRateHz = 60;
        float stabilityThreshold = 0.8f;
    };

    class TrajectorySolver
    {
    public:
        TrajectorySolver(const SolverConfig &config, time::VirtualClock &clock, bus::CommunicationBus &bus);

        void start();
        void stop();

        [[nodiscard]] KinematicSolution latestSolution() const;

    private:
        void handleTrackerState(const TrackerState &state);
        void workerLoop(std::stop_token st);

        SolverConfig m_config;
        time::VirtualClock &m_clock;
        bus::CommunicationBus &m_bus;

        std::jthread m_worker;
        std::atomic<bool> m_running{false};

        mutable std::mutex m_mutex;

        TrackerState m_lastTrackerState{};
        KinematicSolution m_lastSolution{};

        std::chrono::steady_clock::time_point m_lastUpdate{};
    };

} // namespace sensorfusion::solver