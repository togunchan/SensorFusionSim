#include <TrajectorySolver/TrajectorySolver.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <chrono>

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

    KinematicSolution TrajectorySolver::computeSolution(const TrackerState &state, std::chrono::steady_clock::time_point simTime)
    {
        KinematicSolution sol;
        sol.timestamp = simTime;
        const auto &pos = state.position;
        const auto &vel = state.velocity;

        // 1. Azimuth angle (horizontal orientation)
        // Direction of the target projected onto the XY-plane.
        sol.azimuth_offset = std::atan2(pos.y(), pos.x());

        // 2. Elevation angle (vertical orientation)
        // Angle between the target and the ground plane.
        float horizontal = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
        sol.elevation_offset = std::atan2(pos.z(), horizontal);

        // 3. Stability score
        // Simple heuristic: high speed => lower stability.
        float speed = vel.norm();
        sol.stability_score = std::clamp(1.0f - speed * 0.1f, 0.0f, 1.0f);

        // Confidence acts as a multiplier: unreliable data => unstable solution.
        sol.stability_score *= state.confidence;

        // 4. Final stable/unstable decision
        sol.is_stable = (sol.stability_score >= m_config.stabilityThreshold);

        return sol;
    }

    void TrajectorySolver::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        // Convert the configured tick rate to the clock's duration type.
        const auto tickInterval = std::chrono::duration_cast<time::VirtualClock::Duration>(
            std::chrono::duration<double>(1.0 / m_config.tickRateHz));

        while (!st.stop_requested())
        {
            auto simTime = m_clock.now();

            KinematicSolution sol;

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                sol = computeSolution(m_lastTrackerState, simTime);
                m_lastSolution = sol;
            }

            m_bus.publish(sol);

            std::this_thread::sleep_for(tickInterval);
            m_clock.advance(tickInterval);
        }
    }
}
