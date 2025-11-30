#include <catch2/catch_test_macros.hpp>
#include <Visualization/JsonSerializer.hpp>
#include <SensorFusionSim/Messages.hpp>
#include <EngagementController/EngagementController.hpp>
#include <Eigen/Core>

TEST_CASE("JsonSerializer formats SensorFrame", "[JsonSerializer]")
{
    sensorfusion::SensorFrame f{};
    f.imu_accel = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    f.imu_gyro = Eigen::Vector3f(4.0f, 5.0f, 6.0f);
    f.lidar_range = 7.0f;

    const auto json = sensorfusion::viz::JsonSerializer::toJson(f);
    REQUIRE(json == R"({"kind":"sensor","acc":[1,2,3],"gyro":[4,5,6],"range":7})");
}

TEST_CASE("JsonSerializer formats TrackerState", "[JsonSerializer]")
{
    sensorfusion::TrackerState s{};
    s.position = Eigen::Vector3f(-1.0f, 0.0f, 2.0f);
    s.velocity = Eigen::Vector3f(3.0f, 4.0f, 5.0f);
    s.confidence = 0.75f;

    const auto json = sensorfusion::viz::JsonSerializer::toJson(s);
    REQUIRE(json == R"({"kind":"tracker","pos":[-1,0,2],"vel":[3,4,5],"conf":0.75})");
}

TEST_CASE("JsonSerializer formats KinematicSolution", "[JsonSerializer]")
{
    sensorfusion::KinematicSolution k{};
    k.azimuth_offset = 1.5f;
    k.elevation_offset = -0.5f;
    k.stability_score = 0.33f;
    k.is_stable = true;

    const auto json = sensorfusion::viz::JsonSerializer::toJson(k);
    REQUIRE(json == R"({"kind":"solution","az":1.5,"el":-0.5,"st":0.33,"stable":1})");
}

TEST_CASE("JsonSerializer formats EngagementState", "[JsonSerializer]")
{
    const auto json = sensorfusion::viz::JsonSerializer::toJson(sensorfusion::control::EngagementState::Ready);
    REQUIRE(json == R"({"kind":"engage","state":4})");
}
