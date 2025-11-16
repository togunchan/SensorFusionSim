cmake_minimum_required(VERSION 3.21)
project(SensorFusionSim LANGUAGES CXX)

add_library(VirtualTime
    VirtualTime/src/VirtualClock.cpp
)

target_include_directories(VirtualTime
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/VirtualTime/include
)

target_compile_features(VirtualTime PUBLIC cxx_std_20)