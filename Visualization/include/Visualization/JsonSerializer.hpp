#pragma once
#include <CommunicationBus/CommunicationBus.hpp>
#include <EngagementController/EngagementController.hpp>
#include <string>

namespace sensorfusion::viz
{
    struct JsonSerializer
    {
        static std::string toJson(const SensorFrame &f);
        static std::string toJson(const TrackerState &s);
        static std::string toJson(const KinematicSolution &k);
        static std::string toJson(sensorfusion::control::EngagementState state);
    };
} // namespace sensorfusion::viz
