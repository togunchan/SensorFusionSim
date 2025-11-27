#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <stop_token>

namespace sensorfusion::viz
{
    class LivePlotClient
    {
    public:
        LivePlotClient(const std::string &host, int port);
        ~LivePlotClient();

        void start();
        void stop();

        void send(const std::string &jsonMessage);

    private:
        void workerLoop(std::stop_token st);

        std::string m_host;
        int m_port;

        int m_socketFd = -1;
        std::mutex m_writeMutex;

        std::jthread m_worker;
        std::atomic<bool> m_running{false};
    };

} // namespace sensorfusion::viz
