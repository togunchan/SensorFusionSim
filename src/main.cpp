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

    motion::MotionConfig motionCfg;
    motionCfg.angularSpeedRadSec = 0.8f; // spiral angular speed
    motionCfg.initialRadius = 80.0f;     // start far (meters)
    motionCfg.radialShrinkRate = 0.7f;   // approach speed (m/s)
    motionCfg.spiralStartTimeSec = 6.0f; // start spiralling after 6 sec
    motionCfg.altitude = 25.0f;          // fixed altitude

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
    ctrlCfg.dataTimeout = std::chrono::milliseconds(500);

    // Modules
    sensors::SensorManager sensorManager(clock, bus, motionCfg);
    tracking::TargetTracker tracker(trackerCfg, clock, bus);
    solver::TrajectorySolver solver(solverCfg, clock, bus);
    control::EngagementController controller(ctrlCfg, clock, bus);

    // Visualization pipeline
    viz::LivePlotClient client("127.0.0.1", 5555);
    viz::VisualizationPublisher publisher(bus, client);

    sensorManager.addSensor(sensorCfg);

    bus.start();
    client.start();
    publisher.start();
    sensorManager.start();
    tracker.start();
    solver.start();
    controller.start();

    std::this_thread::sleep_for(std::chrono::seconds(15));

    controller.stop();
    solver.stop();
    tracker.stop();
    sensorManager.stop();
    publisher.stop();
    client.stop();
    bus.stop();
}
