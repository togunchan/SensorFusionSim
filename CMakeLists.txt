cmake_minimum_required(VERSION 3.21)
project(SensorFusionSim LANGUAGES CXX)

include(CTest)
find_package(Eigen3 REQUIRED CONFIG)

# -------------------------------------------------------
# VirtualTime module
# -------------------------------------------------------
add_library(VirtualTime
    VirtualTime/src/VirtualClock.cpp
)

target_include_directories(VirtualTime
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/VirtualTime/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(VirtualTime PUBLIC cxx_std_20)
target_link_libraries(VirtualTime
    PUBLIC
        Eigen3::Eigen
)

# -------------------------------------------------------
# CommunicationBus module
# -------------------------------------------------------
add_library(CommunicationBus
    CommunicationBus/src/CommunicationBus.cpp
)

target_include_directories(CommunicationBus
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/CommunicationBus/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(CommunicationBus
    PUBLIC
        VirtualTime
        Eigen3::Eigen
)

# -------------------------------------------------------
# SensorManager module
# -------------------------------------------------------
add_library(SensorManager
    SensorManager/src/SensorManager.cpp
)

target_include_directories(SensorManager
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/SensorManager/include
)

target_link_libraries(SensorManager
    PUBLIC
        VirtualTime
        CommunicationBus
)

# -------------------------------------------------------
target_compile_features(CommunicationBus PUBLIC cxx_std_20)
# -------------------------------------------------------

# -------------------------------------------------------
# Unit Tests
# -------------------------------------------------------
if(BUILD_TESTING)
    enable_testing()

    add_executable(TestVirtualClock tests/unit/Test_VirtualClock.cpp)
    target_link_libraries(TestVirtualClock PRIVATE VirtualTime)
    target_compile_features(TestVirtualClock PRIVATE cxx_std_20)
    add_test(NAME VirtualClockTests COMMAND TestVirtualClock)
endif()
