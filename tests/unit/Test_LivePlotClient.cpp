#define private public
#include <Visualization/LivePlotClient.hpp>
#undef private

#include <catch2/catch_test_macros.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>

TEST_CASE("LivePlotClient writes newline-terminated payloads and closes sockets", "[LivePlotClient]")
{
    using namespace sensorfusion::viz;

    int sockets[2]{-1, -1};
    REQUIRE(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);

    LivePlotClient client("127.0.0.1", 0);
    client.m_socketFd = sockets[0]; // Inject test socket
    client.m_running = true;        // Allow stop() to perform cleanup

    const std::string payload = R"({"hello":"world"})";
    client.send(payload);

    char buffer[128]{};
    const auto bytesRead = recv(sockets[1], buffer, sizeof(buffer), 0);
    REQUIRE(bytesRead > 0);
    std::string received(buffer, buffer + bytesRead);
    REQUIRE(received == payload + "\n"); // newline appended by send()

    client.stop(); // should close sockets[0]

    // Once closed, writes to the old descriptor should fail.
    errno = 0;
    REQUIRE(::send(sockets[0], "x", 1, 0) == -1);

    close(sockets[1]);
}
