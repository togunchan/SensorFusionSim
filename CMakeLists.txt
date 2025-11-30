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
        TargetMotion
)

# -------------------------------------------------------
# TargetTracker module
# -------------------------------------------------------
add_library(TargetTracker
    TargetTracker/src/TargetTracker.cpp
)

target_include_directories(TargetTracker
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/TargetTracker/include
)

target_link_libraries(TargetTracker
    PUBLIC
        VirtualTime
        CommunicationBus
)

# -------------------------------------------------------
# TrajectorySolver module
# -------------------------------------------------------
add_library(TrajectorySolver
    TrajectorySolver/src/TrajectorySolver.cpp
)

target_include_directories(TrajectorySolver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/TrajectorySolver/include
)

target_link_libraries(TrajectorySolver
    PUBLIC
        VirtualTime
        CommunicationBus
)

# -------------------------------------------------------
# EngagementController module
# -------------------------------------------------------
add_library(EngagementController
    EngagementController/src/EngagementController.cpp
)

target_include_directories(EngagementController
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/EngagementController/include
)

target_link_libraries(EngagementController
    PUBLIC
        VirtualTime
        CommunicationBus
)

# -------------------------------------------------------
# DataLogger module
# -------------------------------------------------------
add_library(DataLogger
    DataLogger/src/DataLogger.cpp
)

target_include_directories(DataLogger
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/DataLogger/include
)

target_link_libraries(DataLogger
    PUBLIC
        CommunicationBus
)
# -------------------------------------------------------
# Visualization module
# -------------------------------------------------------
add_library(Visualization
    Visualization/src/LivePlotClient.cpp
    Visualization/src/JsonSerializer.cpp 
    Visualization/src/VisualizationPublisher.cpp
)

target_include_directories(Visualization
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/include
)

target_link_libraries(Visualization
    PUBLIC
        pthread
        CommunicationBus
        EngagementController
)

# -------------------------------------------------------
# TargetMotion module
# -------------------------------------------------------
add_library(TargetMotion
    TargetMotion/src/TargetMotionGenerator.cpp
)

target_include_directories(TargetMotion
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/TargetMotion/include
)

target_link_libraries(TargetMotion
    PUBLIC
        VirtualTime
        Eigen3::Eigen
)

target_compile_features(TargetMotion PUBLIC cxx_std_20)

# -------------------------------------------------------
target_compile_features(Visualization PUBLIC cxx_std_20)
target_compile_features(CommunicationBus PUBLIC cxx_std_20)
# -------------------------------------------------------

# -------------------------------------------------------
# Main executable
# -------------------------------------------------------
add_executable(SensorFusionSim
    src/main.cpp
)

target_include_directories(SensorFusionSim
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(SensorFusionSim
    PRIVATE
        VirtualTime
        CommunicationBus
        SensorManager
        TargetTracker
        TrajectorySolver
        EngagementController
        DataLogger
        Visualization
)

target_compile_features(SensorFusionSim PRIVATE cxx_std_20)

# -------------------------------------------------------
# Unit Tests
# -------------------------------------------------------
if(BUILD_TESTING)
    enable_testing()
    # Prefer vendored Catch2 if present; otherwise rely on an installed package.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Catch2/CMakeLists.txt")
        set(CATCH_BUILD_TESTING OFF CACHE BOOL "" FORCE)
        add_subdirectory(third_party/Catch2)
    else()
        find_package(Catch2 3 REQUIRED)
    endif()

    set(TEST_SOURCES
        tests/unit/Test_VirtualClock.cpp
        tests/unit/Test_CommunicationBus.cpp
        tests/unit/Test_SensorManager.cpp
        tests/unit/Test_TargetTracker.cpp
        tests/unit/Test_TrajectorySolver.cpp
        tests/unit/Test_EngagementController.cpp
        tests/unit/Test_DataLogger.cpp
        tests/unit/Test_JsonSerializer.cpp
        tests/unit/Test_LivePlotClient.cpp
        tests/unit/Test_VisualizationPublisher.cpp
    )

    add_executable(SensorFusionSimTests ${TEST_SOURCES})
    target_link_libraries(SensorFusionSimTests
        PRIVATE
            Catch2::Catch2WithMain
            VirtualTime
            CommunicationBus
            SensorManager
            TargetTracker
            TrajectorySolver
            EngagementController
            DataLogger
            Visualization
    )
    target_compile_features(SensorFusionSimTests PRIVATE cxx_std_20)
    add_test(NAME SensorFusionSimTests COMMAND SensorFusionSimTests)
endif()
