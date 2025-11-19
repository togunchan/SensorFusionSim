#include <SensorManager/SensorManager.hpp>
#include <chrono>
#include <random>

namespace sensorfusion::sensors
{
    SensorManager::SensorManager(time::VirtualClock &clock, bus::CommunicationBus &bus) : m_clock(clock), m_bus(bus) {}

    void SensorManager::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void SensorManager::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
        {
            m_worker.request_stop();
            m_worker.join();
        }
    }

    void SensorManager::addSensor(const SensorConfig &config)
    {
        m_sensors[config.id] = config;
    }

    void SensorManager::removeSensor(const std::string &id)
    {
        m_sensors.erase(id);
    }

    sensorfusion::SensorFrame SensorManager::generateFrame(const SensorConfig &config)
    {
        // thread_local: Each thread gets its own independent copy of this variable.
        // This ensures that in multithreaded programs, every thread has its own rng instance,
        // avoiding race conditions and eliminating the need for synchronization mechanisms.
        static thread_local std::mt19937 rng{std::random_device{}()};
        std::normal_distribution<float> noise(0.0f, config.noiseSigma);

        sensorfusion::SensorFrame frame;
        frame.timestamp = m_clock.now();

        // Simple IMU model: device assumed stationary, small vibrations simulated with added noise
        frame.imu_accel = Eigen::Vector3f(
            0.0f + noise(rng), // X-axis: ideally 0, plus noise
            0.0f + noise(rng), // Y-axis: ideally 0, plus noise
            9.81f + noise(rng) // Z-axis: gravity (9.81 m/s^2) plus noise
        );

        // Gyroscope: ideally zero angular velocity when stationary,
        // but random noise is added to simulate realistic sensor drift
        frame.imu_gyro = Eigen::Vector3f(
            noise(rng),
            noise(rng),
            noise(rng));

        // Simple LIDAR model: fixed 10m distance measurement,
        // with noise added to mimic real-world sensor variation
        frame.lidar_range = 10.0f + noise(rng);
        frame.dropout_flag = false;
        frame.spike_flag = false;
        frame.stuck_flag = false;
        frame.noise_sigma = config.noiseSigma;

        return frame;
    }

    void SensorManager::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        while (!st.stop_requested())
        {
            auto now = m_clock.now();

            for (auto &[id, config] : m_sensors)
            {
                auto &nextTime = m_nextUpdate[id];
                if (now >= nextTime)
                {
                    auto frame = generateFrame(config);
                    frame.timestamp = now;
                    m_bus.publish(frame);

                    const auto interval = std::chrono::duration_cast<time::VirtualClock::Duration>(
                        std::chrono::duration<double>(1.0 / config.updateRateHz));
                    nextTime = now + interval;
                }
            }
            std::this_thread::sleep_for(1ms);
            m_clock.advance(1ms);
        }
    }
} // namespace sensorfusion::sensors
