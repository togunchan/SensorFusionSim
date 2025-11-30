#include <TargetTracker/TargetTracker.hpp>
#include <algorithm>
#include <cmath>

namespace sensorfusion::tracking
{

    TargetTracker::TargetTracker(const TrackerConfig &config,
                                 time::VirtualClock &clock,
                                 bus::CommunicationBus &bus)
        : m_config(config), m_clock(clock), m_bus(bus)
    {
        m_lastState.timestamp = clock.now();
        m_lastState.position = Eigen::Vector3f::Zero();
        m_lastState.velocity = Eigen::Vector3f::Zero();
        m_lastState.confidence = config.initialConfidence;
        m_confidence = config.initialConfidence;

        // Initialize covariance with modest uncertainty on position/velocity.
        m_P = Eigen::Matrix4f::Identity();
        m_P(0, 0) = m_P(1, 1) = 25.0f; // 5m std dev on position
        m_P(2, 2) = m_P(3, 3) = 4.0f;  // 2m/s std dev on velocity

        const auto now = clock.now();
        m_lastPredict = now;
        m_lastUpdate = now;
        m_lastHeadingUpdate = now;
    }

    void TargetTracker::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return; // already running

        m_bus.subscribe([this](const sensorfusion::SensorFrame &frame)
                        { this->handleSensorFrame(frame); });

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void TargetTracker::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
            m_worker.request_stop();
    }

    TrackerState TargetTracker::latestState() const
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_lastState;
    }

    void TargetTracker::handleSensorFrame(const SensorFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);

        // Integrate yaw to obtain heading for converting range to Cartesian.
        float dtHeading = std::chrono::duration<float>(frame.timestamp - m_lastHeadingUpdate).count();
        if (dtHeading < 0.0f)
            dtHeading = 0.0f;

        m_theta += frame.imu_gyro.z() * dtHeading;
        m_hasHeading = true;
        constexpr float pi = 3.14159265358979323846f;
        m_theta = std::remainder(m_theta, 2.0f * pi);

        m_latestFrame = frame;
        m_lastAccel = frame.imu_accel.head<2>();
        m_pendingFrame = true;
        m_lastUpdate = frame.timestamp;
        m_lastHeadingUpdate = frame.timestamp;
    }

    void TargetTracker::workerLoop(std::stop_token st)
    {
        using namespace std::chrono_literals;

        const auto tickInterval = 20ms;
        const float accelNoise = 0.8f; // process noise std dev (m/s^2)

        while (!st.stop_requested())
        {
            auto now = m_clock.now();

            SensorFrame frame{};
            bool hasFrame = false;
            float dtPredict = std::chrono::duration<float>(now - m_lastPredict).count();
            if (dtPredict < 0.0f)
                dtPredict = 0.0f;

            Eigen::Vector2f accel = m_lastAccel;

            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                if (m_pendingFrame)
                {
                    frame = m_latestFrame;
                    hasFrame = true;
                    accel = frame.imu_accel.head<2>();
                    // Predict up to the measurement timestamp for consistency.
                    dtPredict = std::chrono::duration<float>(frame.timestamp - m_lastPredict).count();
                    if (dtPredict < 0.0f)
                        dtPredict = 0.0f;
                }
            }

            // --- Predict step (constant-velocity with acceleration input) ---
            if (dtPredict > 0.0f)
            {
                Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
                F(0, 2) = dtPredict;
                F(1, 3) = dtPredict;

                Eigen::Matrix<float, 4, 2> B;
                B << 0.5f * dtPredict * dtPredict, 0.0f,
                    0.0f, 0.5f * dtPredict * dtPredict,
                    dtPredict, 0.0f,
                    0.0f, dtPredict;

                Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
                const float qVar = accelNoise * accelNoise;
                Q(0, 0) = 0.25f * dtPredict * dtPredict * dtPredict * dtPredict * qVar;
                Q(1, 1) = Q(0, 0);
                Q(2, 2) = dtPredict * dtPredict * qVar;
                Q(3, 3) = Q(2, 2);

                m_x = F * m_x + B * accel;
                m_P = F * m_P * F.transpose() + Q;
                m_lastPredict += std::chrono::duration_cast<time::VirtualClock::Duration>(
                    std::chrono::duration<float>(dtPredict));
            }

            Eigen::Vector2f innovation = Eigen::Vector2f::Zero();
            Eigen::Matrix2f S = Eigen::Matrix2f::Identity();

            // --- Update step when a new measurement is available ---
            if (hasFrame && m_hasHeading)
            {
                const float range = frame.lidar_range;
                const float measX = range * std::cos(m_theta);
                const float measY = range * std::sin(m_theta);
                Eigen::Vector2f z(measX, measY);

                Eigen::Matrix<float, 2, 4> H;
                H << 1, 0, 0, 0,
                    0, 1, 0, 0;

                const float measNoise = std::max(0.5f, frame.noise_sigma + 0.2f);
                Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * (measNoise * measNoise);

                Eigen::Vector2f zHat = H * m_x;
                innovation = z - zHat;
                S = H * m_P * H.transpose() + R;
                Eigen::Matrix<float, 4, 2> K = m_P * H.transpose() * S.inverse();

                m_x = m_x + K * innovation;
                Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
                m_P = (I - K * H) * m_P;

                m_lastUpdate = frame.timestamp;
            }

            // --- Confidence update (innovation + uncertainty + recency) ---
            const float posVar = m_P.block<2, 2>(0, 0).trace();
            const float velVar = m_P.block<2, 2>(2, 2).trace();
            const float uncertaintyScore = 1.0f / (1.0f + 0.5f * (posVar + 0.2f * velVar));

            const float innovationScale = std::sqrt(std::max(1e-6f, S.trace()));
            const float innovationScore = std::exp(-innovation.norm() / (innovationScale + 1e-3f));

            const float ageSec = std::chrono::duration<float>(now - m_lastUpdate).count();
            const float ageScore = std::exp(-ageSec / 1.5f);

            const float blended = 0.4f * innovationScore + 0.4f * uncertaintyScore + 0.2f * ageScore;
            m_confidence = std::clamp(0.5f * m_confidence + 0.5f * blended, 0.05f, 0.98f);

            // Additional decay if measurements are missing for too long.
            const auto maxGap = std::chrono::milliseconds(static_cast<int>(m_config.maxUpdateInterval));
            if ((now - m_lastUpdate) > maxGap)
            {
                const float decay = std::exp(-0.5f * std::chrono::duration<float>(now - m_lastUpdate - maxGap).count());
                m_confidence = std::clamp(m_confidence * decay, 0.05f, 0.98f);
            }

            {
                std::lock_guard<std::mutex> lock(m_stateMutex);
                m_lastState.timestamp = hasFrame ? frame.timestamp : now;
                m_lastState.position = Eigen::Vector3f(m_x(0), m_x(1), 0.0f);
                m_lastState.velocity = Eigen::Vector3f(m_x(2), m_x(3), 0.0f);
                m_lastState.confidence = m_confidence;
                m_lastState.covariance_trace = m_P.trace();

                m_bus.publish(m_lastState);
            }

            std::this_thread::sleep_for(tickInterval);
            m_clock.advance(tickInterval);
        }
    }

} // namespace sensorfusion::tracking
