#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <SensorManager/SensorManager.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <Eigen/Core>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>
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

TEST_CASE("SensorManager generateFrame reflects clock and config", "[SensorManager]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::motion::MotionConfig motionCfg{};
    sensorfusion::sensors::SensorManager manager(clock, bus, motionCfg);

    sensorfusion::sensors::SensorConfig cfg{};
    cfg.id = "imu";
    cfg.noiseSigma = 0.0f; // deterministic output

    auto before = clock.now();
    auto frame = manager.generateFrame(cfg);
    REQUIRE(frame.timestamp == before);
    REQUIRE(frame.noise_sigma == Approx(cfg.noiseSigma));
    REQUIRE(frame.dropout_flag == false);
    REQUIRE(frame.spike_flag == false);
    REQUIRE(frame.stuck_flag == false);
    REQUIRE(frame.imu_accel.z() == Approx(9.81f).margin(0.1f));
    REQUIRE(frame.imu_gyro.isApprox(Eigen::Vector3f::Zero(), 1e-6f));
}

TEST_CASE("SensorManager publishes frames at configured intervals", "[SensorManager]")
{
    sensorfusion::time::VirtualClock clock;
    sensorfusion::bus::CommunicationBus bus;
    sensorfusion::motion::MotionConfig motionCfg{};
    sensorfusion::sensors::SensorManager manager(clock, bus, motionCfg);

    sensorfusion::sensors::SensorConfig cfg{};
    cfg.id = "fast";
    cfg.updateRateHz = 500.0; // 2 ms period
    cfg.noiseSigma = 0.0f;    // deterministic values

    std::vector<sensorfusion::SensorFrame> captured;
    std::mutex m;
    std::condition_variable cv;

    bus.subscribe([&](const sensorfusion::SensorFrame &f)
                  {
                      {
                          std::lock_guard<std::mutex> lock(m);
                          captured.push_back(f);
                      }
                      cv.notify_all();
                  });

    bus.start();
    manager.addSensor(cfg);
    manager.start();

    const bool received = waitFor([&]
                                  { return captured.size() >= 3; },
                                  150ms);

    manager.stop();
    bus.stop();

    REQUIRE(received);
    REQUIRE(captured.front().noise_sigma == Approx(cfg.noiseSigma));
    REQUIRE(captured.size() >= 3);

    // Subsequent timestamps should progress forward by at least 1ms each step.
    for (std::size_t i = 1; i < captured.size(); ++i)
    {
        REQUIRE(captured[i].timestamp > captured[i - 1].timestamp);
        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(captured[i].timestamp - captured[i - 1].timestamp);
        REQUIRE(delta.count() >= 1);
    }
}
