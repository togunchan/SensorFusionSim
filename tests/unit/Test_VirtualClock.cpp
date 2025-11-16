#include <cassert>
#include <chrono>
#include <VirtualTime/VirtualClock.hpp>

using namespace sensorfusion::time;

int main()
{
    VirtualClock clock;

    // Test 1: now() should not regress or be uninitialized
    auto t1 = clock.now();
    auto t2 = clock.now();
    assert(t2 == t1);

    // Test 2: advance()
    auto before = clock.now();
    clock.advance(std::chrono::milliseconds(50));
    auto after = clock.now();
    assert(after - before == std::chrono::milliseconds(50));

    // Test 3: reset()
    clock.advance(std::chrono::milliseconds(10));
    clock.reset();
    auto r1 = clock.now();
    clock.advance(std::chrono::milliseconds(1));
    auto r2 = clock.now();
    assert(r2 > r1);

    return 0;
}