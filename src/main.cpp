#include <SensorManager/SensorManager.hpp>
#include <Visualization/VisualizationPublisher.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <TrajectorySolver/TrajectorySolver.hpp>
#include <TargetTracker/TargetTracker.hpp>
#include <EngagementController/EngagementController.hpp>
#include <VirtualTime/VirtualClock.hpp>
#include <chrono>
#include <thread>

using namespace sensorfusion;

int main()
{
    time::VirtualClock clock;
    bus::CommunicationBus bus;

    // Configs
    sensors::SensorConfig sensorCfg;
    sensorCfg.id = "imu";
    sensorCfg.updateRateHz = 50.0;
    sensorCfg.noiseSigma = 0.1f;

    solver::SolverConfig solverCfg;
    solverCfg.tickRateHz = 50.0;
    solverCfg.stabilityThreshold = 0.5f;

    tracking::TrackerConfig trackerCfg;
    trackerCfg.initialConfidence = 0.5f;
    trackerCfg.maxUpdateInterval = 200.0f;

    control::ControllerConfig ctrlCfg;
    ctrlCfg.minStabilityToAlign = 0.5f;
    ctrlCfg.dataTimeout = std::chrono::milliseconds(300);

    // Modules
    sensors::SensorManager sensorManager(clock, bus);
    solver::TrajectorySolver solver(solverCfg, clock, bus);
    tracking::TargetTracker tracker(trackerCfg, clock, bus);
    control::EngagementController controller(ctrlCfg, clock, bus);

    // Visualization pipeline
    viz::LivePlotClient client("127.0.0.1", 5555);
    viz::VisualizationPublisher publisher(bus, client);

    sensorManager.addSensor(sensorCfg);

    // Start
    bus.start();
    client.start();
    publisher.start();
    sensorManager.start();
    solver.start();
    tracker.start();
    controller.start();

    // Run the simulation for 15 seconds
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // Stop
    controller.stop();
    tracker.stop();
    solver.stop();
    sensorManager.stop();
    publisher.stop();
    client.stop();
    bus.stop();
}
