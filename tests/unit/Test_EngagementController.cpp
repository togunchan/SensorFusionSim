#define private public
#include <EngagementController/EngagementController.hpp>
#undef private

#include <catch2/catch_test_macros.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

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

TEST_CASE("EngagementController progresses through nominal states when stability is sufficient", "[EngagementController]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;

    sensorfusion::control::ControllerConfig cfg;
    cfg.minStabilityToAlign = 0.5f;
    cfg.dataTimeout = 500ms;
    sensorfusion::control::EngagementController controller(cfg, clock, bus);

    // Seed a stable solution before the worker loop starts to avoid an early Safe transition.
    controller.m_lastSolution.stability_score = 1.0f;
    controller.m_lastSolution.timestamp = clock.now();
    controller.m_lastUpdate = clock.now();

    std::vector<sensorfusion::control::EngagementState> seen;
    std::mutex m;
    std::condition_variable cv;

    bus.subscribe([&](sensorfusion::control::EngagementState s)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          seen.push_back(s);
                      }
                      cv.notify_all();
                  });

    controller.start();
    bus.start();

    // Keep providing a stable solution to refresh lastUpdate.
    sensorfusion::KinematicSolution stable{};
    stable.stability_score = 1.0f;
    stable.timestamp = clock.now();
    bus.publish(stable);

    const bool reachedCompleted = waitFor([&]
                                          { return std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Completed) != seen.end(); },
                                          300ms);

    controller.stop();
    bus.stop();

    REQUIRE(reachedCompleted);
    REQUIRE(std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Safe) == seen.end());
}

TEST_CASE("EngagementController falls back to Safe when data times out", "[EngagementController]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;

    sensorfusion::control::ControllerConfig cfg;
    cfg.dataTimeout = 30ms;
    cfg.minStabilityToAlign = 0.5f;
    sensorfusion::control::EngagementController controller(cfg, clock, bus);

    // Provide an initial stable solution to avoid the low-stability Safe path.
    controller.m_lastSolution.stability_score = 1.0f;
    controller.m_lastSolution.timestamp = clock.now();
    controller.m_lastUpdate = clock.now();

    std::vector<sensorfusion::control::EngagementState> seen;
    std::mutex m;
    std::condition_variable cv;

    bus.subscribe([&](sensorfusion::control::EngagementState s)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          seen.push_back(s);
                      }
                      cv.notify_all();
                  });

    controller.start();
    bus.start();

    const bool reachedSafe = waitFor([&]
                                     { return std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Safe) != seen.end(); },
                                     150ms);

    controller.stop();
    bus.stop();

    REQUIRE(reachedSafe);
    REQUIRE(controller.state() == sensorfusion::control::EngagementState::Safe);
}
