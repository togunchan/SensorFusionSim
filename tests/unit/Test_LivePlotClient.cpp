#include <Visualization/LivePlotClient.hpp>
#include <thread>
#include <chrono>
#include <iostream>

using namespace sensorfusion::viz;

int main()
{
    LivePlotClient client("127.0.0.1", 5555);
    client.start();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "[Test] Sending test JSON..." << std::endl;

    for (int i = 0; i < 5; ++i)
    {
        std::string json =
            "{ \"kind\": \"test\", \"value\": " + std::to_string(i) + " }";

        client.send(json);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    client.stop();
}