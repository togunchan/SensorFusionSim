#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <TargetTracker/TargetTracker.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <Eigen/Core>

#include <chrono>
#include <functional>
#include <thread>

using namespace std::chrono_literals;
using Catch::Approx;

namespace
{
    bool waitFor(const std::function<bool()> &predicate, std::chrono::milliseconds timeout)
    {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (predicate())
                return true;
            std::this_thread::sleep_for(1ms);
        }
        return predicate();
    }
} // namespace

TEST_CASE("TargetTracker derives pose from lidar and yaw", "[TargetTracker]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::tracking::TrackerConfig cfg;
    cfg.initialConfidence = 0.45f;
    cfg.maxUpdateInterval = 400.0f;
    sensorfusion::tracking::TargetTracker tracker(cfg, clock, bus);

    tracker.start();
    bus.start();

    sensorfusion::SensorFrame frame1{};
    frame1.timestamp = clock.now() + 20ms;
    frame1.lidar_range = 10.0f;
    frame1.imu_gyro = Eigen::Vector3f::Zero();
    frame1.imu_accel = Eigen::Vector3f::Zero();
    bus.publish(frame1);

    sensorfusion::SensorFrame frame2{};
    frame2.timestamp = frame1.timestamp + 50ms;
    frame2.lidar_range = 10.6f; // small increase in range
    frame2.imu_gyro = Eigen::Vector3f::Zero();
    frame2.imu_accel = Eigen::Vector3f::Zero();
    bus.publish(frame2);

    const bool updated2 = waitFor([&]
                                  { return tracker.latestState().timestamp == frame2.timestamp; },
                                  200ms);

    auto state2 = tracker.latestState();

    tracker.stop();
    bus.stop();

    REQUIRE(updated2);
    REQUIRE(state2.position.x() == Approx(10.6f).margin(0.2f));
    REQUIRE(state2.position.y() == Approx(0.0f).margin(0.1f));
    REQUIRE(state2.velocity.x() > 0.05f);
    REQUIRE(std::abs(state2.velocity.y()) < 1.0f);
    REQUIRE(state2.covariance_trace > 0.0f);
    REQUIRE(state2.confidence > cfg.initialConfidence);
    REQUIRE(state2.confidence < 0.98f);
}

TEST_CASE("TargetTracker decreases confidence when updates are missing", "[TargetTracker]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::tracking::TrackerConfig cfg;
    cfg.initialConfidence = 0.7f;
    cfg.maxUpdateInterval = 30.0f; // milliseconds
    sensorfusion::tracking::TargetTracker tracker(cfg, clock, bus);

    tracker.start();
    bus.start();

    const bool degraded = waitFor([&]
                                  { return tracker.latestState().confidence < cfg.initialConfidence; },
                                  200ms);

    tracker.stop();
    bus.stop();

    REQUIRE(degraded);
}
