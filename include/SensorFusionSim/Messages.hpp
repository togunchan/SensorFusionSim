#pragma once

#include <chrono>
#include <string>
#include <Eigen/Core>

namespace sensorfusion
{
    // Represents a single synthetic sensor packet (IMU + LiDAR) plus fault metadata.
    struct SensorFrame
    {
        std::chrono::steady_clock::time_point timestamp;

        //  Inertial Measurement Unit (IMU) Data
        Eigen::Vector3f imu_accel;
        Eigen::Vector3f imu_gyro;

        // single-range LIDAR measurement
        float lidar_range{0.0f};

        // Fault indicators
        bool dropout_flag{false};
        bool spike_flag{false};
        bool stuck_flag{false};

        // Noise characteristics
        float noise_sigma{0.0f};
    };

    // Represents the fused pose/velocity estimate for the current virtual time.
    struct TrackerState
    {
        std::chrono::steady_clock::time_point timestamp;
        Eigen::Vector3f position{Eigen::Vector3f::Zero()};
        Eigen::Vector3f velocity{Eigen::Vector3f::Zero()};
        float confidence{0.0f};
    };

    // Summarizes solver output: orientation offsets and stability.
    struct KinematicSolution
    {
        std::chrono::steady_clock::time_point timestamp;

        // Neutral robotic orientation parameters
        float azimuth_offset{0.0f};
        float elevation_offset{0.0f};

        // Stability metrics
        float stability_score{0.0f};
        bool is_stable{false};
    };

    enum class SystemEventType
    {
        StateTransition,
        FusionUpdate,
        SolverUpdate,
        FaultInjected
    };

    // Represents high-level system events for logging and visualization.
    struct SystemEvent
    {
        std::chrono::steady_clock::time_point timestamp;
        SystemEventType type{SystemEventType::StateTransition};
        std::string description;
    };
} // namespace sensorfusion
