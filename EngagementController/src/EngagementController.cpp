#include <EngagementController/EngagementController.hpp>

namespace sensorfusion::control
{

    EngagementController::EngagementController(const ControllerConfig &config,
                                               time::VirtualClock &clock,
                                               bus::CommunicationBus &bus)
        : m_config(config), m_clock(clock), m_bus(bus)
    {
        m_lastUpdate = clock.now();
        m_lastPublish = m_lastUpdate;
        m_stateEntered = m_lastUpdate;
    }

    void EngagementController::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_bus.subscribe([this](const KinematicSolution &sol)
                        { this->handleSolution(sol); });

        auto now = m_clock.now();
        setState(EngagementState::Idle, now, true); // immediate publish for visualization

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void EngagementController::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
        {
            m_worker.request_stop();
            m_worker.join();
        }
    }

    EngagementState EngagementController::state() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_state;
    }

    void EngagementController::handleSolution(const KinematicSolution &sol)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_lastSolution = sol;
        m_lastUpdate = sol.timestamp;
        m_hasSolution = true;
    }

    void EngagementController::publishLocked(EngagementState state, time::VirtualClock::TimePoint now)
    {
        m_bus.publish(state);
        m_lastPublished = state;
        m_lastPublish = now;
    }

    bool EngagementController::setState(EngagementState newState, time::VirtualClock::TimePoint now, bool forcePublish)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const bool changed = newState != m_state;

        if (changed)
        {
            m_state = newState;
            m_stateEntered = now;
        }

        if (changed || forcePublish)
        {
            publishLocked(m_state, now);
            return true;
        }
        return false;
    }

    void EngagementController::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;
        const auto tickInterval = 20ms;
        const auto heartbeatInterval = m_config.heartbeat;

        auto dwellFor = [&](EngagementState s)
        {
            using namespace std::chrono_literals;
            switch (s)
            {
            case EngagementState::Acquiring:
                return m_config.minDwell;
            case EngagementState::Tracking:
                return m_config.minDwell + 200ms;
            case EngagementState::Aligning:
                return m_config.minDwell + 300ms;
            case EngagementState::Ready:
                return m_config.minDwell + 200ms;
            case EngagementState::Executing:
                return m_config.minDwell + 400ms;
            default:
                return 0ms;
            }
        };

        while (!st.stop_requested())
        {
            EngagementState current;
            KinematicSolution sol;
            std::chrono::steady_clock::time_point lastUpdate;
            std::chrono::steady_clock::time_point lastPublish;
            std::chrono::steady_clock::time_point stateEntered;
            bool hasSolution = false;
            auto now = m_clock.now();

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                current = m_state;
                sol = m_lastSolution;
                lastUpdate = m_lastUpdate;
                lastPublish = m_lastPublish;
                stateEntered = m_stateEntered;
                hasSolution = m_hasSolution;
            }

            bool published = false;
            const bool stale = (now - lastUpdate) > m_config.dataTimeout;

            if (stale)
            {
                published = setState(EngagementState::Safe, now, current != EngagementState::Safe);
                current = EngagementState::Safe;
                lastPublish = published ? now : lastPublish;
            }
            else if (hasSolution)
            {
                if (sol.stability_score < 0.02f)
                {
                    published = setState(EngagementState::Safe, now, current != EngagementState::Safe);
                    current = EngagementState::Safe;
                    lastPublish = published ? now : lastPublish;
                }
                else
                {
                    auto meets = [&](float factor)
                    {
                        return sol.stability_score >= m_config.minStabilityToAlign * factor;
                    };

                    EngagementState next = current;
                    switch (current)
                    {
                    case EngagementState::Idle:
                        if (meets(0.4f))
                            next = EngagementState::Acquiring;
                        break;
                    case EngagementState::Acquiring:
                        if ((now - stateEntered) >= dwellFor(EngagementState::Acquiring) && meets(0.5f))
                            next = EngagementState::Tracking;
                        break;
                    case EngagementState::Tracking:
                        if ((now - stateEntered) >= dwellFor(EngagementState::Tracking) && meets(0.7f))
                            next = EngagementState::Aligning;
                        break;
                    case EngagementState::Aligning:
                        if ((now - stateEntered) >= dwellFor(EngagementState::Aligning) && meets(1.0f))
                            next = EngagementState::Ready;
                        break;
                    case EngagementState::Ready:
                        if ((now - stateEntered) >= dwellFor(EngagementState::Ready) && meets(1.1f))
                            next = EngagementState::Executing;
                        break;
                    case EngagementState::Executing:
                        if ((now - stateEntered) >= dwellFor(EngagementState::Executing) && meets(1.1f))
                            next = EngagementState::Completed;
                        break;
                    case EngagementState::Safe:
                        if ((now - stateEntered) >= m_config.safeRecovery && meets(0.6f) && sol.is_stable)
                            next = EngagementState::Acquiring;
                        break;
                    case EngagementState::Completed:
                        break;
                    }

                    if (next != current)
                    {
                        published = setState(next, now, true);
                        current = next;
                        lastPublish = published ? now : lastPublish;
                    }
                }
            }

            if (!published && (now - lastPublish) >= heartbeatInterval)
            {
                if (setState(current, now, true))
                {
                    lastPublish = now;
                }
            }
            std::this_thread::sleep_for(tickInterval);
            m_clock.advance(tickInterval);
        }
    }
} // namespace sensorfusion::control
