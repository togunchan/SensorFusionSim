#include <TrajectorySolver/TrajectorySolver.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace sensorfusion::solver
{
    TrajectorySolver::TrajectorySolver(const SolverConfig &config, time::VirtualClock &clock, bus::CommunicationBus &bus) : m_config(config), m_clock(clock), m_bus(bus)
    {
        m_lastTrackerState.timestamp = clock.now();
        m_lastTrackerState.position = Eigen::Vector3f::Zero();
        m_lastTrackerState.velocity = Eigen::Vector3f::Zero();
        m_lastTrackerState.confidence = 0.0f;

        m_lastSolution.timestamp = clock.now();
        m_lastSolution.azimuth_offset = 0.0f;
        m_lastSolution.elevation_offset = 0.0f;
        m_lastSolution.stability_score = 0.0f;
        m_lastSolution.is_stable = false;

        m_lastUpdate = clock.now();
    }

    void TrajectorySolver::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_bus.subscribe([this](const TrackerState &state)
                        { this->handleTrackerState(state); });

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void TrajectorySolver::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
            m_worker.request_stop();
    }

    KinematicSolution TrajectorySolver::latestSolution() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_lastSolution;
    }

    void TrajectorySolver::handleTrackerState(const TrackerState &state)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_lastTrackerState = state;
        m_lastUpdate = state.timestamp;
    }
}