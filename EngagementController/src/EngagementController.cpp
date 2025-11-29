#include <EngagementController/EngagementController.hpp>

namespace sensorfusion::control
{

    EngagementController::EngagementController(const ControllerConfig &config,
                                               time::VirtualClock &clock,
                                               bus::CommunicationBus &bus)
        : m_config(config), m_clock(clock), m_bus(bus)
    {
        m_lastUpdate = clock.now();
    }

    void EngagementController::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_bus.subscribe([this](const KinematicSolution &sol)
                        { this->handleSolution(sol); });

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
    }

    void EngagementController::transitionTo(EngagementState newState)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_state = newState;
        m_bus.publish(newState);
    }

    void EngagementController::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;
        const auto tickInterval = 20ms;

        static const std::unordered_map<EngagementState, float> kFactors{
            {EngagementState::Idle, 0.25f},
            {EngagementState::Acquiring, 0.50f},
            {EngagementState::Tracking, 1.00f},
            {EngagementState::Aligning, 1.00f},
            {EngagementState::Ready, 1.10f},
            {EngagementState::Executing, 1.10f},
        };

        auto nextState = [](EngagementState s)
        {
            switch (s)
            {
            case EngagementState::Idle:
                return EngagementState::Acquiring;
            case EngagementState::Acquiring:
                return EngagementState::Tracking;
            case EngagementState::Tracking:
                return EngagementState::Aligning;
            case EngagementState::Aligning:
                return EngagementState::Ready;
            case EngagementState::Ready:
                return EngagementState::Executing;
            case EngagementState::Executing:
                return EngagementState::Completed;
            default:
                return s;
            }
        };

        while (!st.stop_requested())
        {

            EngagementState current;
            KinematicSolution sol;
            std::chrono::steady_clock::time_point lastUpdate;
            auto now = m_clock.now();

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                current = m_state;
                sol = m_lastSolution;
                lastUpdate = m_lastUpdate;
            }

            if ((now - lastUpdate) > m_config.dataTimeout)
            {
                transitionTo(EngagementState::Safe);
            }
            else if (current != EngagementState::Safe &&
                     current != EngagementState::Completed)
            {
                if (sol.stability_score < 0.05f)
                {
                    transitionTo(EngagementState::Safe);
                }
                else
                {
                    float required = m_config.minStabilityToAlign * kFactors.at(current);

                    if (sol.stability_score >= required)
                    {
                        transitionTo(nextState(current));
                    }
                }
            }
            std::this_thread::sleep_for(tickInterval);
            m_clock.advance(tickInterval);
        }
    }
} // namespace sensorfusion::control
