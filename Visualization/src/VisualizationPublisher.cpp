#include <Visualization/VisualizationPublisher.hpp>

namespace sensorfusion::viz
{
    VisualizationPublisher::VisualizationPublisher(bus::CommunicationBus &bus, LivePlotClient &client) : m_bus(bus), m_client(client) {}

    void VisualizationPublisher::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return;

        m_bus.subscribe([this](const sensorfusion::SensorFrame &f)
                        { this->handleSensorFrame(f); });
        m_bus.subscribe([this](const sensorfusion::TrackerState &s)
                        { this->handleTrackerState(s); });

        m_bus.subscribe([this](const sensorfusion::KinematicSolution &k)
                        { this->handleKinematicSolution(k); });

        m_bus.subscribe([this](sensorfusion::control::EngagementState st)
                        { this->handleEngagementState(st); });
    }
    void VisualizationPublisher::stop()
    {
        bool wasRunning = m_running.exchange(false);
        if (!wasRunning)
            return;

        m_client.stop();
    }

    void VisualizationPublisher::handleSensorFrame(const sensorfusion::SensorFrame &f)
    {
        // Sends IMU accel/gyro and LiDAR range as a "sensor" JSON payload.
        m_client.send(JsonSerializer::toJson(f));
    }

    void VisualizationPublisher::handleTrackerState(const sensorfusion::TrackerState &s)
    {
        // Publishes position/velocity/confidence as "tracker" for plotting confidence over time.
        m_client.send(JsonSerializer::toJson(s));
    }

    void VisualizationPublisher::handleKinematicSolution(const sensorfusion::KinematicSolution &k)
    {
        // Sends azimuth/elevation/stability fields as "solution".
        m_client.send(JsonSerializer::toJson(k));
    }

    void VisualizationPublisher::handleEngagementState(sensorfusion::control::EngagementState st)
    {
        // Encodes the FSM state as an integer for the step-plot in the Python visualizer.
        m_client.send(JsonSerializer::toJson(st));
    }

} // namespace sensorfusion::viz
