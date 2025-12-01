#include <TargetMotion/TargetMotionGenerator.hpp>

#include <chrono>
#include <cmath>

namespace sensorfusion::motion
{
    using namespace std::chrono;

    TargetMotionGenerator::TargetMotionGenerator(const MotionConfig &cfg, Clock &clock) : m_cfg(cfg), m_clock(clock)
    {
        m_t0 = m_clock.now();
    }

    void TargetMotionGenerator::update()
    {
        // Compute elapsed simulation time in seconds
        const auto now = m_clock.now();
        const double t = duration_cast<duration<double>>(now - m_t0).count();

        const double omega = static_cast<double>(m_cfg.angularSpeedRadSec);

        // Radius: orbit first, then spiral-in
        double R = static_cast<double>(m_cfg.initialRadius);
        double dRdt = 0.0;

        if (t >= static_cast<double>(m_cfg.spiralStartTimeSec))
        {
            const double dtSpiral =
                t - static_cast<double>(m_cfg.spiralStartTimeSec);

            R = static_cast<double>(m_cfg.initialRadius) - static_cast<double>(m_cfg.radialShrinkRate) * dtSpiral;

            if (R < 0.0)
                R = 0.0;

            // During the spiral the radial distance shrinks at a constant speed,
            // so its derivative is a constant negative radialShrinkRate.
            dRdt = -static_cast<double>(m_cfg.radialShrinkRate);
        }

        const double theta = omega * t;

        const double c = std::cos(theta);
        const double s = std::sin(theta);

        // Position: orbit / spiral on XY, constant altitude on Z
        const double x = R * c;
        const double y = R * s;
        const double z = static_cast<double>(m_cfg.altitude);

        m_pos = Eigen::Vector3f(
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z));

        // Velocity: derivative of position with respect to time
        // vx = dR/dt * cos(theta) - R * omega * sin(theta)
        // vy = dR/dt * sin(theta) + R * omega * cos(theta)
        const double vx = dRdt * c - R * omega * s;
        const double vy = dRdt * s + R * omega * c;
        const double vz = 0.0;

        m_vel = Eigen::Vector3f(
            static_cast<float>(vx),
            static_cast<float>(vy),
            static_cast<float>(vz));

        // Acceleration (simplified model):
        // radial + centripetal component, we keep it readable:
        // ax ≈ -R * omega^2 * cos(theta)
        // ay ≈ -R * omega^2 * sin(theta)
        // az = 0
        // The centripetal term dominates; radial acceleration from dR/dt is ignored for clarity.
        const double ax = -R * omega * omega * c;
        const double ay = -R * omega * omega * s;
        const double az = 0.0;

        m_acc = Eigen::Vector3f(
            static_cast<float>(ax),
            static_cast<float>(ay),
            static_cast<float>(az));

        // Yaw rate: simply the angular speed around Z
        m_yawRate = static_cast<float>(omega);
    }

    Eigen::Vector3f TargetMotionGenerator::position() const
    {
        return m_pos;
    }

    Eigen::Vector3f TargetMotionGenerator::velocity() const
    {
        return m_vel;
    }

    Eigen::Vector3f TargetMotionGenerator::acceleration() const
    {
        return m_acc;
    }

    float TargetMotionGenerator::yawRate() const
    {
        return m_yawRate;
    }

} // namespace sensorfusion::motion