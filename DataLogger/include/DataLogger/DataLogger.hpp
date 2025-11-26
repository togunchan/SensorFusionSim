#pragma once

#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <mutex>
#include <fstream>

namespace sensorfusion::logging
{
    struct LoggerConfig
    {
        std::string outputPath;
        bool logSensorFrames = true;
        bool logTrackerStates = true;
        bool logKinematicSolutions = true;
        bool logSystemEvents = true;
    };

    class DataLogger
    {
    public:
        DataLogger(const LoggerConfig &config,
                   bus::CommunicationBus &bus);

        void start();
        void stop();

    private:
        void handleSensorFrame(const sensorfusion::SensorFrame &f);
        void handleTrackerState(const sensorfusion::TrackerState &s);
        void handleKinematicSolution(const sensorfusion::KinematicSolution &k);
        void handleSystemEvent(const sensorfusion::SystemEvent &e);

    private:
        LoggerConfig m_config;
        bus::CommunicationBus &m_bus;

        std::ofstream m_file;
        std::mutex m_mutex;

        bool m_running = false;
    };
} // namespace sensorfusion::logging