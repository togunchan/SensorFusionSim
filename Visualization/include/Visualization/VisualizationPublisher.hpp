#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <Visualization/LivePlotClient.hpp>
#include <Visualization/JsonSerializer.hpp>
#include <atomic>
#include <mutex>

namespace sensorfusion::viz
{
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