#define private public
#include <Visualization/LivePlotClient.hpp>
#undef private
#include <Visualization/VisualizationPublisher.hpp>
#include <Visualization/JsonSerializer.hpp>
#include <CommunicationBus/CommunicationBus.hpp>
#include <SensorFusionSim/Messages.hpp>

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Core>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <string>
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

TEST_CASE("VisualizationPublisher forwards bus payloads as JSON over LivePlotClient", "[VisualizationPublisher]")
{
    sensorfusion::bus::CommunicationBus bus;

    int sockets[2]{-1, -1};
    REQUIRE(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);

    sensorfusion::viz::LivePlotClient client("127.0.0.1", 0);
    client.m_socketFd = sockets[0];
    client.m_running = true; // allow stop() to close the socket

    sensorfusion::viz::VisualizationPublisher publisher(bus, client);
    publisher.start();
    bus.start();

    sensorfusion::SensorFrame frame{};
    frame.imu_accel = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    frame.imu_gyro = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    frame.lidar_range = 5.0f;

    sensorfusion::TrackerState tracker{};
    tracker.position = Eigen::Vector3f(2.0f, 3.0f, 4.0f);
    tracker.velocity = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    tracker.confidence = 0.9f;

    sensorfusion::KinematicSolution sol{};
    sol.azimuth_offset = 0.25f;
    sol.elevation_offset = -0.5f;
    sol.stability_score = 0.8f;
    sol.is_stable = true;

    const auto expectedSensor = sensorfusion::viz::JsonSerializer::toJson(frame);
    const auto expectedTracker = sensorfusion::viz::JsonSerializer::toJson(tracker);
    const auto expectedSol = sensorfusion::viz::JsonSerializer::toJson(sol);
    const auto expectedEngagement = sensorfusion::viz::JsonSerializer::toJson(sensorfusion::control::EngagementState::Aligning);

    bus.publish(frame);
    bus.publish(tracker);
    bus.publish(sol);
    bus.publish(sensorfusion::control::EngagementState::Aligning);

    std::string buffer;
    const auto expectedLines = 4;

    const bool received = waitFor([&]
                                  {
                                      char chunk[256];
                                      ssize_t n = recv(sockets[1], chunk, sizeof(chunk), MSG_DONTWAIT);
                                      if (n > 0)
                                      {
                                          buffer.append(chunk, chunk + n);
                                      }
                                      return std::count(buffer.begin(), buffer.end(), '\n') >= expectedLines;
                                  },
                                  200ms);

    publisher.stop();
    bus.stop();
    close(sockets[1]); // sockets[0] is closed by LivePlotClient::stop()

    REQUIRE(received);

    std::vector<std::string> lines;
    std::stringstream ss(buffer);
    std::string line;
    while (std::getline(ss, line))
    {
        if (!line.empty())
            lines.push_back(line);
    }

    REQUIRE(lines.size() >= static_cast<std::size_t>(expectedLines));
    REQUIRE(lines[0] == expectedSensor);
    REQUIRE(lines[1] == expectedTracker);
    REQUIRE(lines[2] == expectedSol);
    REQUIRE(lines[3] == expectedEngagement);
}
