#include <Visualization/JsonSerializer.hpp>
#include <sstream>

namespace sensorfusion::viz
{
    std::string JsonSerializer::toJson(const SensorFrame &f)
    {
        std::ostringstream oss;
        oss << "{"
            << "\"kind\":\"sensor\","
            << "\"acc\":[" << f.imu_accel.x() << "," << f.imu_accel.y() << "," << f.imu_accel.z() << "],"
            << "\"gyro\":[" << f.imu_gyro.x() << "," << f.imu_gyro.y() << "," << f.imu_gyro.z() << "],"
            << "\"range\":" << f.lidar_range
            << "}";
        return oss.str();
    }

    std::string JsonSerializer::toJson(const TrackerState &s)
    {
        std::ostringstream oss;
        oss << "{"
            << "\"kind\":\"tracker\","
            << "\"pos\":[" << s.position.x() << "," << s.position.y() << "," << s.position.z() << "],"
            << "\"vel\":[" << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << "],"
            << "\"conf\":" << s.confidence
            << "}";
        return oss.str();
    }

    std::string JsonSerializer::toJson(const KinematicSolution &k)
    {
        std::ostringstream oss;
        oss << "{"
            << "\"kind\":\"solution\","
            << "\"az\":" << k.azimuth_offset << ","
            << "\"el\":" << k.elevation_offset << ","
            << "\"st\":" << k.stability_score << ","
            << "\"stable\":" << (k.is_stable ? 1 : 0)
            << "}";
        return oss.str();
    }

    std::string JsonSerializer::toJson(sensorfusion::control::EngagementState state)
    {
        std::ostringstream oss;
        oss << "{"
            << "\"kind\":\"engage\","
            << "\"state\":" << static_cast<int>(state)
            << "}";
        return oss.str();
    }
} // namespace sensorfusion::viz