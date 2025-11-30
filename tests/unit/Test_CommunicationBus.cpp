#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>

#include <atomic>
#include <chrono>
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

TEST_CASE("CommunicationBus fans out messages to all subscribers", "[CommunicationBus]")
{
    sensorfusion::bus::CommunicationBus bus;

    std::mutex m;
    std::condition_variable cv;

    std::atomic<int> frameCount{0};
    std::atomic<int> trackerCount{0};
    std::atomic<int> solutionCount{0};
    std::atomic<int> eventCount{0};
    std::atomic<int> engagementCount{0};

    float capturedRange = 0.0f;
    float capturedConfidence = 0.0f;

    bus.subscribe([&](const sensorfusion::SensorFrame &f)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          capturedRange = f.lidar_range;
                          frameCount++;
                      }
                      cv.notify_all();
                  });
    bus.subscribe([&](const sensorfusion::TrackerState &s)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          capturedConfidence = s.confidence;
                          trackerCount++;
                      }
                      cv.notify_all();
                  });
    bus.subscribe([&](const sensorfusion::KinematicSolution &)
                  {
                      solutionCount++;
                      cv.notify_all();
                  });
    bus.subscribe([&](const sensorfusion::SystemEvent &)
                  {
                      eventCount++;
                      cv.notify_all();
                  });
    bus.subscribe([&](const sensorfusion::control::EngagementState &)
                  {
                      engagementCount++;
                      cv.notify_all();
                  });

    bus.start();

    sensorfusion::SensorFrame frame{};
    frame.lidar_range = 42.0f;
    bus.publish(frame);

    sensorfusion::TrackerState tracker{};
    tracker.confidence = 0.3f;
    bus.publish(tracker);

    sensorfusion::KinematicSolution solution{};
    solution.stability_score = 0.9f;
    bus.publish(solution);

    sensorfusion::SystemEvent evt{};
    evt.description = "fanout";
    bus.publish(evt);

    bus.publish(sensorfusion::control::EngagementState::Tracking);

    const bool delivered = waitFor(
        [&]
        {
            return frameCount.load() == 1 && trackerCount.load() == 1 &&
                   solutionCount.load() == 1 && eventCount.load() == 1 &&
                   engagementCount.load() == 1;
        },
        200ms);

    bus.stop();

    REQUIRE(delivered);
    REQUIRE(capturedRange == Approx(42.0f));
    REQUIRE(capturedConfidence == Approx(0.3f));
}

TEST_CASE("CommunicationBus drops messages when configured for overflow", "[CommunicationBus]")
{
    sensorfusion::bus::BusConfig config;
    config.dropOnOverflow = true;
    config.maxQueueSizePerType = 1;
    sensorfusion::bus::CommunicationBus bus(config);

    std::atomic<int> handled{0};
    bus.subscribe([&](const sensorfusion::SensorFrame &)
                  { handled++; });

    sensorfusion::SensorFrame frame{};

    // Queue fills before the worker starts, later publishes should be dropped.
    bus.publish(frame);
    bus.publish(frame);
    bus.publish(frame);

    bus.start();

    const bool singleDelivered = waitFor([&]
                                         { return handled.load() == 1; },
                                         200ms);
    bus.stop();

    REQUIRE(singleDelivered);
}
