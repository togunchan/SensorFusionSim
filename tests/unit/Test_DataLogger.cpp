#include <catch2/catch_test_macros.hpp>
#include <DataLogger/DataLogger.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <SensorFusionSim/Messages.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("DataLogger writes all enabled message types", "[DataLogger]")
{
    sensorfusion::bus::CommunicationBus bus;

    auto tempPath = std::filesystem::temp_directory_path() /
                    ("sensorfusion_logger_enabled_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".txt");

    sensorfusion::logging::LoggerConfig cfg;
    cfg.outputPath = tempPath.string();
    cfg.logSensorFrames = true;
    cfg.logTrackerStates = true;
    cfg.logKinematicSolutions = true;
    cfg.logSystemEvents = true;

    sensorfusion::logging::DataLogger logger(cfg, bus);

    logger.start();
    bus.start();

    sensorfusion::SensorFrame frame{};
    frame.lidar_range = 12.0f;
    bus.publish(frame);

    sensorfusion::TrackerState state{};
    state.confidence = 0.7f;
    bus.publish(state);

    sensorfusion::KinematicSolution sol{};
    sol.stability_score = 0.9f;
    bus.publish(sol);

    sensorfusion::SystemEvent evt{};
    evt.description = "test-event";
    bus.publish(evt);

    std::this_thread::sleep_for(50ms); // allow worker to flush

    logger.stop();
    bus.stop();

    std::ifstream in(tempPath);
    REQUIRE(in.good());
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    REQUIRE(contents.find("[SensorFrame]") != std::string::npos);
    REQUIRE(contents.find("[TrackerState]") != std::string::npos);
    REQUIRE(contents.find("[KinematicSolution]") != std::string::npos);
    REQUIRE(contents.find("[SystemEvent]") != std::string::npos);

    std::filesystem::remove(tempPath);
}

TEST_CASE("DataLogger respects disabled logging flags", "[DataLogger]")
{
    sensorfusion::bus::CommunicationBus bus;

    auto tempPath = std::filesystem::temp_directory_path() /
                    ("sensorfusion_logger_disabled_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".txt");

    sensorfusion::logging::LoggerConfig cfg;
    cfg.outputPath = tempPath.string();
    cfg.logSensorFrames = false;
    cfg.logTrackerStates = false;
    cfg.logKinematicSolutions = false;
    cfg.logSystemEvents = false;

    sensorfusion::logging::DataLogger logger(cfg, bus);

    logger.start();
    bus.start();

    bus.publish(sensorfusion::SensorFrame{});
    bus.publish(sensorfusion::TrackerState{});
    bus.publish(sensorfusion::KinematicSolution{});
    bus.publish(sensorfusion::SystemEvent{});

    std::this_thread::sleep_for(30ms);

    logger.stop();
    bus.stop();

    std::ifstream in(tempPath);
    REQUIRE(in.good());
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    REQUIRE(contents.empty());

    std::filesystem::remove(tempPath);
}
