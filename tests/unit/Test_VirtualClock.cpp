#include <catch2/catch_test_macros.hpp>
#include <VirtualTime/VirtualClock.hpp>

using sensorfusion::time::VirtualClock;
using namespace std::chrono_literals;

TEST_CASE("VirtualClock maintains a deterministic virtual timeline", "[VirtualClock]")
{
    VirtualClock clock;

    SECTION("starts at the epoch and does not regress")
    {
        auto t1 = clock.now();
        auto t2 = clock.now();
        REQUIRE(t1 == VirtualClock::TimePoint{});
        REQUIRE(t2 == t1);
    }

    SECTION("advances forward while ignoring negative deltas")
    {
        clock.advance(10ms);
        auto afterPositive = clock.now();
        REQUIRE(afterPositive - VirtualClock::TimePoint{} == 10ms);

        clock.advance(-5ms);
        REQUIRE(clock.now() == afterPositive);
    }

    SECTION("reset returns the clock to the initial epoch")
    {
        clock.advance(7ms);
        REQUIRE(clock.now() > VirtualClock::TimePoint{});
        clock.reset();
        REQUIRE(clock.now() == VirtualClock::TimePoint{});
    }
}
