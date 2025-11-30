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

TEST_CASE("TargetTracker integrates acceleration into state", "[TargetTracker]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::tracking::TrackerConfig cfg;
    cfg.initialConfidence = 0.5f;
    cfg.maxUpdateInterval = 200.0f;
    sensorfusion::tracking::TargetTracker tracker(cfg, clock, bus);

    tracker.start();
    bus.start();

    sensorfusion::SensorFrame frame{};
    frame.timestamp = clock.now() + 50ms;
    frame.imu_accel = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    frame.imu_gyro = Eigen::Vector3f::Zero();
    bus.publish(frame);

    const bool updated = waitFor([&]
                                 { return tracker.latestState().timestamp == frame.timestamp; },
                                 200ms);

    auto state = tracker.latestState();

    tracker.stop();
    bus.stop();

    REQUIRE(updated);
    REQUIRE(state.velocity.x() == Approx(0.05f)); // dv = a * dt = 1 * 0.05s
    REQUIRE(state.position.x() == Approx(0.0f));
    REQUIRE(state.confidence == Approx(0.6f));
}

TEST_CASE("TargetTracker decreases confidence when updates are missing", "[TargetTracker]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::tracking::TrackerConfig cfg;
    cfg.initialConfidence = 1.0f;
    cfg.maxUpdateInterval = 5.0f; // milliseconds
    sensorfusion::tracking::TargetTracker tracker(cfg, clock, bus);

    tracker.start();
    bus.start();

    const bool degraded = waitFor([&]
                                  { return tracker.latestState().confidence < cfg.initialConfidence; },
                                  80ms);

    tracker.stop();
    bus.stop();

    REQUIRE(degraded);
}
