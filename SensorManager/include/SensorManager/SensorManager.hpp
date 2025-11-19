#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>

namespace sensorfusion::sensors
{

    struct SensorConfig
    {
        std::string id;
        double updateRateHz = 100.0;
        float noiseSigma = 0.01f;
    };

    class SensorManager
    {
    public:
        SensorManager(time::VirtualClock &clock, bus::CommunicationBus &bus);

        void start();
        void stop();

        void addSensor(const SensorConfig &config);
        void removeSensor(const std::string &id);
        sensorfusion::SensorFrame generateFrame(const SensorConfig &config);

    private:
        void workerLoop(std::stop_token st);

        time::VirtualClock &m_clock;
        bus::CommunicationBus &m_bus;
        std::jthread m_worker;
        std::atomic<bool> m_running{false};

        std::unordered_map<std::string, SensorConfig> m_sensors;
        std::unordered_map<std::string, std::chrono::steady_clock::time_point> m_nextUpdate;
    };

} // namespace sensorfusion::sensors
