#include <DataLogger/DataLogger.hpp>
#include <iomanip>
#include <chrono>

namespace sensorfusion::logging
{
    DataLogger::DataLogger(const LoggerConfig &config, bus::CommunicationBus &bus) : m_config(config), m_bus(bus) {}

    void DataLogger::start()
    {
        if (m_running)
            return;

        m_running = true;

        m_file.open(m_config.outputPath, std::ios::out | std::ios::trunc);

        if (m_config.logSensorFrames)
            m_bus.subscribe([this](const sensorfusion::SensorFrame &f)
                            { this->handleSensorFrame(f); });

        if (m_config.logTrackerStates)
            m_bus.subscribe([this](const sensorfusion::TrackerState &s)
                            { this->handleTrackerState(s); });

        if (m_config.logKinematicSolutions)
            m_bus.subscribe([this](const sensorfusion::KinematicSolution &k)
                            { this->handleKinematicSolution(k); });

        if (m_config.logSystemEvents)
            m_bus.subscribe([this](const sensorfusion::SystemEvent &e)
                            { this->handleSystemEvent(e); });
    }

    void DataLogger::stop()
    {
        if (!m_running)
            return;
        m_running = false;

        if (m_file.is_open())
            m_file.close();
    }

    void DataLogger::handleSensorFrame(const sensorfusion::SensorFrame &f)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_file << "[SensorFrame] acc=" << f.imu_accel.transpose()
               << " gyro=" << f.imu_gyro.transpose()
               << " range=" << f.lidar_range
               << "\n";
    }

    void DataLogger::handleTrackerState(const sensorfusion::TrackerState &s)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_file << "[TrackerState] pos=" << s.position.transpose()
               << " vel=" << s.velocity.transpose()
               << " conf=" << s.confidence
               << "\n";
    }

    void DataLogger::handleKinematicSolution(const sensorfusion::KinematicSolution &k)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_file << "[KinematicSolution] az=" << k.azimuth_offset
               << " el=" << k.elevation_offset
               << " st=" << k.stability_score
               << " stable=" << k.is_stable
               << "\n";
    }

    void DataLogger::handleSystemEvent(const sensorfusion::SystemEvent &e)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_file << "[SystemEvent] type=" << (int)e.type
               << " desc=" << e.description
               << "\n";
    }

} // namespace sensorfusion::logging