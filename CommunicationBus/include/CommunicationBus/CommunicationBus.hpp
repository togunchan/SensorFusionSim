#pragma once

#include <functional>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

#include <SensorFusionSim/Messages.hpp>

namespace sensorfusion::bus
{
    struct BusConfig
    {
        bool dropOnOverflow = false;
        std::size_t maxQueueSizePerType = 1024;
    };
    class CommunicationBus
    {
    public:
        using SensorFrameHandler = std::function<void(const sensorfusion::SensorFrame &)>;
        using TrackerStateHandler = std::function<void(const sensorfusion::TrackerState &)>;
        using KinematicSolutionHandler = std::function<void(const sensorfusion::KinematicSolution &)>;
        using SystemEventHandler = std::function<void(const sensorfusion::SystemEvent &)>;

        explicit CommunicationBus(const BusConfig &config = {});

        void start();
        void stop();

        // Publish API - thread-safe
        void publish(const sensorfusion::SensorFrame &frame);
        void publish(const sensorfusion::TrackerState &state);
        void publish(const sensorfusion::KinematicSolution &solution);
        void publish(const sensorfusion::SystemEvent &event);

        // Subscription API - call before start()
        void subscribe(SensorFrameHandler handler);
        void subscribe(TrackerStateHandler handler);
        void subscribe(KinematicSolutionHandler handler);
        void subscribe(SystemEventHandler handler);

    private:
        // Internal helper to stop worker, single-threaded deterministic fan-out
        void workerLoop(std::stop_token st);

        BusConfig m_config{};

        // Queues for each message type
        std::queue<sensorfusion::SensorFrame> m_sensorFrameQueue;
        std::queue<sensorfusion::TrackerState> m_trackerStateQueue;
        std::queue<sensorfusion::KinematicSolution> m_kinematicSolutionQueue;
        std::queue<sensorfusion::SystemEvent> m_systemEventQueue;

        // Subscribers per message type
        std::vector<SensorFrameHandler> m_sensorFrameHandlers;
        std::vector<TrackerStateHandler> m_trackerStateHandlers;
        std::vector<KinematicSolutionHandler> m_kinematicSolutionHandlers;
        std::vector<SystemEventHandler> m_systemEventHandlers;

        // Synchronization
        std::mutex m_mutex;
        std::condition_variable m_cv;
        bool m_hasWork{false};

        std::atomic<bool> m_running{false};
        std::jthread m_worker;
    }; // class CommunicationBus
} // namespace sensorfusion::bus