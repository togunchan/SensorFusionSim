#include <Visualization/LivePlotClient.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <chrono>

namespace sensorfusion::viz
{
    LivePlotClient::LivePlotClient(const std::string &host, int port) : m_host(host), m_port(port) {}

    LivePlotClient::~LivePlotClient()
    {
        stop();
    }

    void LivePlotClient::start()
    {
        bool expected = false;
        if (!m_running.compare_exchange_strong(expected, true))
            return;

        m_worker = std::jthread([this](std::stop_token st)
                                { workerLoop(st); });
    }

    void LivePlotClient::stop()
    {
        if (!m_running.exchange(false))
            return;

        if (m_worker.joinable())
        {
            m_worker.request_stop();
            m_worker.join();
        }

        if (m_socketFd >= 0)
            close(m_socketFd);
    }

    void LivePlotClient::workerLoop(std::stop_token st)
    {
        while (!st.stop_requested())
        {
            m_socketFd = socket(AF_INET, SOCK_STREAM, 0);
            if (m_socketFd < 0)
            {
                std::cerr << "[LivePlotClient] Socket creation failed\n";
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(m_port);
            inet_pton(AF_INET, m_host.c_str(), &addr.sin_addr);

            if (connect(m_socketFd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                std::cerr << "[LivePlotClient] Connection failed, retrying...\n";
                close(m_socketFd);
                m_socketFd = -1;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            std::cout << "[LivePlotClient] Connected to plot server.\n";
            break;
        }
    }

    void LivePlotClient::send(const std::string &jsonMessage)
    {
        std::lock_guard<std::mutex> lock(m_writeMutex);
        if (m_socketFd < 0)
            return;

        std::string payload = jsonMessage + "\n";
        ::send(m_socketFd, payload.c_str(), payload.size(), 0);
    }
} // namespace sensorfusion::viz
