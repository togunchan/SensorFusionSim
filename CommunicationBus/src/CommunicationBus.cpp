#include <CommunicationBus.hpp>

namespace sensorfusion::bus
{
    CommunicationBus::CommunicationBus(const BusConfig &config) : m_config(config) {}

    // Start worker thread
    void CommunicationBus::start()
    {
        bool exptected = false;

        // compare_exchange_strong : if m_running == expected(false), set m_running to true and return true.
        // If it fails (m_running was already true), it returns false; the leading ! inverts it so the if triggers.
        if (!m_running.compare_exchange_strong(exptected, true))
        {
            // already running
            return;
        }

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void CommunicationBus::stop()
    {
        if (!m_worker.joinable())
            return;

        m_worker.request_stop();

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_hasWork = true; // wake up worker to exit
        }
        m_cv.notify_one();
        m_worker.join();
    }

    void CommunicationBus::publish(const sensorfusion::SensorFrame &frame)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_config.dropOnOverflow && m_sensorFrameQueue.size() >= m_config.maxQueueSizePerType)
            {
                // TODO: add System Event in the future
            }
            else
            {
                m_sensorFrameQueue.push(frame);
            }

            m_hasWork = true;
        }
        m_cv.notify_one();
    }

    void CommunicationBus::publish(const sensorfusion::TrackerState &state)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if (m_config.dropOnOverflow &&
                m_trackerStateQueue.size() >= m_config.maxQueueSizePerType)
            {
                // see comment above
            }
            else
            {
                m_trackerStateQueue.push(state);
            }

            m_hasWork = true;
        }
        m_cv.notify_one();
    }

    void CommunicationBus::publish(const sensorfusion::KinematicSolution &solution)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if (m_config.dropOnOverflow &&
                m_kinematicSolutionQueue.size() >= m_config.maxQueueSizePerType)
            {
                // see comment above
            }
            else
            {
                m_kinematicSolutionQueue.push(solution);
            }

            m_hasWork = true;
        }
        m_cv.notify_one();
    }

    void CommunicationBus::publish(const sensorfusion::SystemEvent &event)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if (m_config.dropOnOverflow &&
                m_systemEventQueue.size() >= m_config.maxQueueSizePerType)
            {
                // see comment above
            }
            else
            {
                m_systemEventQueue.push(event);
            }

            m_hasWork = true;
        }
        m_cv.notify_one();
    }

    void CommunicationBus::subscribe(SensorFrameHandler handler)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sensorFrameHandlers.emplace_back(std::move(handler));
    }
    void CommunicationBus::subscribe(TrackerStateHandler handler)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_trackerStateHandlers.emplace_back(std::move(handler));
    }
    void CommunicationBus::subscribe(KinematicSolutionHandler handler)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_kinematicSolutionHandlers.emplace_back(std::move(handler));
    }
    void CommunicationBus::subscribe(SystemEventHandler handler)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_systemEventHandlers.emplace_back(std::move(handler));
    }

    void CommunicationBus::workerLoop(std::stop_token st)
    {
        while (!st.stop_requested())
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&]
                      { return m_hasWork || st.stop_requested(); });

            if (st.stop_requested())
                break;

            std::queue<sensorfusion::SensorFrame> sensorFrames;
            std::queue<sensorfusion::TrackerState> trackerStates;
            std::queue<sensorfusion::KinematicSolution> kinematicSolutions;
            std::queue<sensorfusion::SystemEvent> systemEvents;

            sensorFrames.swap(m_sensorFrameQueue);
            trackerStates.swap(m_trackerStateQueue);
            kinematicSolutions.swap(m_kinematicSolutionQueue);
            systemEvents.swap(m_systemEventQueue);

            m_hasWork = false;
            lock.unlock();

            while (!sensorFrames.empty())
            {
                const auto &msg = sensorFrames.front();
                for (auto &h : m_sensorFrameHandlers)
                {
                    if (h)
                    {
                        h(msg);
                    }
                }
                sensorFrames.pop();
            }

            while (!trackerStates.empty())
            {
                const auto &msg = trackerStates.front();
                for (auto &h : m_trackerStateHandlers)
                {
                    if (h)
                    {
                        h(msg);
                    }
                }
                trackerStates.pop();
            }

            while (!kinematicSolutions.empty())
            {
                const auto &msg = kinematicSolutions.front();
                for (auto &h : m_kinematicSolutionHandlers)
                {
                    if (h)
                    {
                        h(msg);
                    }
                }
                kinematicSolutions.pop();
            }

            while (!systemEvents.empty())
            {
                const auto &msg = systemEvents.front();
                for (auto &h : m_systemEventHandlers)
                {
                    if (h)
                    {
                        h(msg);
                    }
                }
                systemEvents.pop();
            }

        } // while (!st.stop_requested())
    } // namespace sensorfusion::bus