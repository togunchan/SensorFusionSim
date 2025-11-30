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
#include <sstream>

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
    cfg.dataTimeout = 2s;
    cfg.minDwell = 120ms;
    cfg.heartbeat = 100ms;
    cfg.safeRecovery = 120ms;
    sensorfusion::control::EngagementController controller(cfg, clock, bus);

    std::vector<sensorfusion::control::EngagementState> seen;
    bus.subscribe([&](sensorfusion::control::EngagementState s)
                  {
                      seen.push_back(s);
                  });

    controller.start();
    bus.start();

    sensorfusion::KinematicSolution stable{};
    stable.stability_score = 0.95f;
    stable.is_stable = true;

    std::jthread feeder([&](std::stop_token st)
                        {
                            while (!st.stop_requested())
                            {
                                stable.timestamp = clock.now();
                                bus.publish(stable);
                                std::this_thread::sleep_for(30ms);
                            }
                        });

    const bool reachedCompleted = waitFor([&]
                                          { return std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Completed) != seen.end(); },
                                          5000ms);

    feeder.request_stop();
    feeder.join();
    controller.stop();
    bus.stop();

    std::ostringstream oss;
    for (auto s : seen)
    {
        oss << static_cast<int>(s) << " ";
    }
    INFO("Seen states: " << oss.str());

    REQUIRE(reachedCompleted);
    REQUIRE(std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Safe) == seen.end());
}

TEST_CASE("EngagementController falls back to Safe when data times out", "[EngagementController]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;

    sensorfusion::control::ControllerConfig cfg;
    cfg.dataTimeout = 80ms;
    cfg.minStabilityToAlign = 0.6f;
    cfg.heartbeat = 50ms;
    sensorfusion::control::EngagementController controller(cfg, clock, bus);

    std::vector<sensorfusion::control::EngagementState> seen;
    bus.subscribe([&](sensorfusion::control::EngagementState s)
                  {
                      seen.push_back(s);
                  });

    controller.start();
    bus.start();

    const bool reachedSafe = waitFor([&]
                                     { return std::find(seen.begin(), seen.end(), sensorfusion::control::EngagementState::Safe) != seen.end(); },
                                     400ms);

    controller.stop();
    bus.stop();

    REQUIRE(reachedSafe);
    REQUIRE(controller.state() == sensorfusion::control::EngagementState::Safe);
}
