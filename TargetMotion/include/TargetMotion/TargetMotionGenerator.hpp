#pragma once

#include <VirtualTime/VirtualClock.hpp>
#include <Eigen/Core>

namespace sensorfusion::motion
{
    struct MotionConfig
    {
        float initialRadius = 20.0f;     // meters, starting radius
        float angularSpeedRadSec = 0.3f; // rad/s, orbit angular speed
        float spiralStartTimeSec = 5.0f; // seconds, when to start spiral-in
        float radialShrinkRate = 1.0f;   // m/s, how fast radius shrinks
        float altitude = 1.5f;           // meters, constant Z
    };

    class TargetMotionGenerator
    {
    public:
        using Clock = sensorfusion::time::VirtualClock;
        using TimePoint = Clock::TimePoint;

        TargetMotionGenerator(const MotionConfig &cfg, Clock &clock);

        // keep computing position/velocity/acc based on current virtual time
        void update();

        Eigen::Vector3f position() const;
        Eigen::Vector3f velocity() const;
        Eigen::Vector3f acceleration() const;

        // Yaw rate around Z, in rad/s
        float yawRate() const;

    private:
        MotionConfig m_cfg;
        Clock &m_clock;
        TimePoint m_t0; // reference start time

        Eigen::Vector3f m_pos{Eigen::Vector3f::Zero()};
        Eigen::Vector3f m_vel{Eigen::Vector3f::Zero()};
        Eigen::Vector3f m_acc{Eigen::Vector3f::Zero()};
        float m_yawRate{0.0f};
    };

} // namespace sensorfusion::motion