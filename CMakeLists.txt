cmake_minimum_required(VERSION 3.21)
project(SensorFusionSim LANGUAGES CXX)

include(CTest)

add_library(VirtualTime
    VirtualTime/src/VirtualClock.cpp
)

target_include_directories(VirtualTime
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/VirtualTime/include
)

target_compile_features(VirtualTime PUBLIC cxx_std_20)

if(BUILD_TESTING)
    add_executable(TestVirtualClock tests/unit/Test_VirtualClock.cpp)
    target_link_libraries(TestVirtualClock PRIVATE VirtualTime)
    target_compile_features(TestVirtualClock PRIVATE cxx_std_20)
    add_test(NAME VirtualClockTests COMMAND TestVirtualClock)
endif()
