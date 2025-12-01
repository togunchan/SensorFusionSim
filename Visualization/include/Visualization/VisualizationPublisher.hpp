#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <Visualization/LivePlotClient.hpp>
#include <Visualization/JsonSerializer.hpp>
#include <atomic>
#include <mutex>

namespace sensorfusion::viz
{
    // Bridges internal messages to the plotting client: subscribes on the CommunicationBus and
    // serializes key fields to JSON lines understood by liveplot_server.py.
    class VisualizationPublisher
    {
    public:
        VisualizationPublisher(bus::CommunicationBus &bus, LivePlotClient &client);

        void start();
        void stop();

    private:
        void handleSensorFrame(const sensorfusion::SensorFrame &f);
        void handleTrackerState(const sensorfusion::TrackerState &s);
        void handleKinematicSolution(const sensorfusion::KinematicSolution &k);
        void handleEngagementState(sensorfusion::control::EngagementState state);

        bus::CommunicationBus &m_bus;
        LivePlotClient &m_client;

        std::atomic<bool> m_running{false};
        std::mutex m_mutex;
    };
} // namespace sensorfusion::viz