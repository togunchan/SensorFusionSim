#include <SensorManager/SensorManager.hpp>
#include <chrono>
#include <random>
#include <cmath>

namespace sensorfusion::sensors
{
    SensorManager::SensorManager(time::VirtualClock &clock, bus::CommunicationBus &bus, const sensorfusion::motion::MotionConfig &motionCfg) : m_clock(clock), m_bus(bus), m_motion(motionCfg, clock) {}

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
        // thread_local: each thread gets its own rng instance to avoid contention
        static thread_local std::mt19937 rng{std::random_device{}()};
        std::normal_distribution<float> noise(0.0f, config.noiseSigma);

        sensorfusion::SensorFrame frame{};
        frame.timestamp = m_clock.now();

        // Simple stationary model with optional Gaussian noise
        frame.imu_accel = Eigen::Vector3f(
            noise(rng),
            noise(rng),
            9.81f + noise(rng));

        frame.imu_gyro = Eigen::Vector3f::Zero();
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

        const auto tick = 1ms;

        while (!st.stop_requested())
        {
            m_motion.update();

            SensorFrame f;
            f.timestamp = m_clock.now();

            auto pos = m_motion.position();
            f.lidar_range = std::hypot(pos.x(), pos.y());

            auto acc = m_motion.acceleration();
            acc.z() += 9.81f;
            f.imu_accel = acc;

            f.imu_gyro = Eigen::Vector3f(0.0f, 0.0f, m_motion.yawRate());

            f.noise_sigma = 0.0f;
            f.dropout_flag = false;
            f.stuck_flag = false;
            f.spike_flag = false;

            m_bus.publish(f);

            std::this_thread::sleep_for(tick);
            m_clock.advance(tick);
        }
    }
} // namespace sensorfusion::sensors
