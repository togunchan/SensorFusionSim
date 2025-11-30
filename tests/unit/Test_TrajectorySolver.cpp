#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <TrajectorySolver/TrajectorySolver.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <Eigen/Core>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
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

TEST_CASE("TrajectorySolver computes orientation and stability from tracker state", "[TrajectorySolver]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::solver::SolverConfig cfg;
    cfg.tickRateHz = 200.0;
    cfg.stabilityThreshold = 0.7f;
    sensorfusion::solver::TrajectorySolver solver(cfg, clock, bus);

    std::mutex m;
    std::condition_variable cv;
    sensorfusion::KinematicSolution received{};
    bool hasSolution = false;

    bus.subscribe([&](const sensorfusion::KinematicSolution &k)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          received = k;
                          hasSolution = true;
                      }
                      cv.notify_all();
                  });

    solver.start();
    bus.start();

    sensorfusion::TrackerState state{};
    state.timestamp = clock.now();
    state.position = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    state.velocity = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
    state.confidence = 0.8f;
    state.covariance_trace = 2.0f;

    bus.publish(state);

    const bool delivered = waitFor([&]
                                   { return hasSolution; },
                                   200ms);

    solver.stop();
    bus.stop();

    REQUIRE(delivered);
    REQUIRE(received.azimuth_offset == Approx(std::atan2(1.0f, 1.0f)));

    const float horiz = std::sqrt(1.0f * 1.0f + 1.0f * 1.0f);
    REQUIRE(received.elevation_offset == Approx(std::atan2(1.0f, horiz)));

    const float speedScore = 1.0f / (1.0f + 0.2f * state.velocity.norm());
    const float covScore = 1.0f / (1.0f + 0.05f * state.covariance_trace);
    const float expectedStability = 0.5f * state.confidence + 0.3f * covScore + 0.2f * speedScore;
    REQUIRE(received.stability_score == Approx(expectedStability));
    REQUIRE(received.is_stable);
}

TEST_CASE("TrajectorySolver reports unstable solutions below threshold", "[TrajectorySolver]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::solver::SolverConfig cfg;
    cfg.tickRateHz = 100.0;
    cfg.stabilityThreshold = 0.9f;
    sensorfusion::solver::TrajectorySolver solver(cfg, clock, bus);

    std::mutex m;
    std::condition_variable cv;
    sensorfusion::KinematicSolution received{};
    bool hasSolution = false;

    bus.subscribe([&](const sensorfusion::KinematicSolution &k)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          received = k;
                          hasSolution = true;
                      }
                      cv.notify_all();
                  });

    solver.start();
    bus.start();

    sensorfusion::TrackerState noisy{};
    noisy.timestamp = clock.now();
    noisy.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    noisy.velocity = Eigen::Vector3f(8.0f, 0.0f, 0.0f); // high speed -> low stability
    noisy.confidence = 0.25f;
    noisy.covariance_trace = 80.0f;

    bus.publish(noisy);

    const bool delivered = waitFor([&]
                                   { return hasSolution; },
                                   200ms);

    solver.stop();
    bus.stop();

    REQUIRE(delivered);
    const float expectedSpeedScore = 1.0f / (1.0f + 0.2f * noisy.velocity.norm());
    const float expectedCovScore = 1.0f / (1.0f + 0.05f * noisy.covariance_trace);
    const float expectedLowStability = 0.5f * noisy.confidence + 0.3f * expectedCovScore + 0.2f * expectedSpeedScore;
    REQUIRE(received.stability_score == Approx(expectedLowStability));
    REQUIRE_FALSE(received.is_stable);
    REQUIRE(received.stability_score < cfg.stabilityThreshold);
}
