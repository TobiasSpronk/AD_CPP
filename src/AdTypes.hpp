#pragma once

#include <cstdint>

#include "AdConstants.hpp"

enum class LaneAssociationType
{
    UNKNOWN_LANE,
    LEFT_LANE,
    CENTER_LANE,
    RIGHT_LANE
};

struct VehicleType
{
    std::int32_t id;
    LaneAssociationType lane;
    float speed_mps; //(meter/s)
    float distance;  //(meter)
};

struct NeighborVehiclesType
{
    VehicleType vehicles_left[VEHICLES_PER_LANE]{};
    VehicleType vehicles_center[VEHICLES_PER_LANE]{};
    VehicleType vehicles_right[VEHICLES_PER_LANE]{};
};
