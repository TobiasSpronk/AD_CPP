#include <cmath>
#include <cstdint>
#include <iostream>

#include "AdConstants.hpp"
#include "AdFunctions.hpp"

float kph_to_mps(const float kph)
{
    float v_mps = kph / 3.6f;
    return v_mps;
}


float mps_to_kph(const float mps)
{
    float v_kph = mps * 3.6f;
    return v_kph;
}


void init_ego_vehicle(VehicleType &ego_vehicle)
{
    ego_vehicle = {EGO_VEHICLE_ID, LaneAssociationType::CENTER_LANE, kph_to_mps(135.0f), 0};
}


void init_vehicle(VehicleType &vehicle, const int id, float speed_kph, float distance, LaneAssociationType lane)
{
    vehicle = {id, lane, kph_to_mps(speed_kph), distance};
}


void init_vehicles(NeighborVehiclesType &vehicles)
{
    float speed[NEIGHBOR_VEHICLES]{150, 135, 110, 150, 90, 90};
    float distance[NEIGHBOR_VEHICLES]{-40, 60, 100, -40, 35, -55};
    for (int i = 0; i < NEIGHBOR_VEHICLES; i++)
    {
        int index = i % 2;
        LaneAssociationType lane = LaneAssociationType::UNKNOWN_LANE;
        if (i < 2)
        {
            lane = LaneAssociationType::LEFT_LANE;
            init_vehicle(vehicles.vehicles_left[index], i, speed[i], distance[i], lane = lane);
        }
        else if (i >= 2 && i < 4)
        {
            lane = LaneAssociationType::CENTER_LANE;
            init_vehicle(vehicles.vehicles_center[index], i, speed[i], distance[i], lane = lane);
        }
        else if (i >= 4 && i < 6)
        {
            lane = LaneAssociationType::RIGHT_LANE;
            init_vehicle(vehicles.vehicles_right[index], i, speed[i], distance[i], lane = lane);
        }
    }
}


void print_vehicle(const VehicleType &vehicle)
{
    std::cout << "ID: " << vehicle.id << '\n';
    std::cout << "Speed: " << vehicle.speed_mps << " (mps)" << '\n';
    std::cout << "Distance: " << vehicle.distance << " (m)" << '\n';
    switch (vehicle.lane)
    {
    case (LaneAssociationType::LEFT_LANE):
    {
        std::cout << "Lane: Left" << '\n';
        break;
    }
    case (LaneAssociationType::RIGHT_LANE):
    {
        std::cout << "Lane: Right" << '\n';
        break;
    }
    case (LaneAssociationType::CENTER_LANE):
    {
        std::cout << "Lane: Center" << '\n';
        break;
    }
    default:
    {
        std::cout << "Lane: Unknown" << '\n';
        break;
    }
    }
}


void print_neighbor_vehicles(const NeighborVehiclesType &vehicles)
{
    for (int i = 0; i < VEHICLES_PER_LANE; i++)
    {
        print_vehicle(vehicles.vehicles_left[i]);
        print_vehicle(vehicles.vehicles_right[i]);
        print_vehicle(vehicles.vehicles_center[i]);
    }
}


void print_scene(const VehicleType &ego_vehicle, const NeighborVehiclesType &vehicles)
{
    std::cout << "\t"
              << "\t"
              << "L\t"
              << "\t"
              << "C\t"
              << "\t"
              << "R\t" << std::endl;
    for (int i = 0; i <= static_cast<int>((VIEW_DISTANCE / DISTACE_STEP_SIZE) * 2); i++)
    {
        float current_position = VIEW_DISTANCE - (static_cast<float>(i) * DISTACE_STEP_SIZE);
        float d_left;
        float d_center;
        float d_right;
        char left_pos = ' ';
        char center_pos = ' ';
        char right_pos = ' ';
        for (int j = 0; j < VEHICLES_PER_LANE; j++)
        {
            d_left = roundf(vehicles.vehicles_left[j].distance / DISTACE_STEP_SIZE) * DISTACE_STEP_SIZE;
            d_center = roundf(vehicles.vehicles_center[j].distance / DISTACE_STEP_SIZE) * DISTACE_STEP_SIZE;
            d_right = roundf(vehicles.vehicles_right[j].distance / DISTACE_STEP_SIZE) * DISTACE_STEP_SIZE;
            if (d_left == current_position)
            {
                left_pos = 'V';
            }
            if (d_center == current_position)
            {
                center_pos = 'V';
            }
            if (d_right == current_position)
            {
                right_pos = 'V';
            }
        }


        if (current_position == 0)
        {
            if (ego_vehicle.lane == LaneAssociationType::CENTER_LANE)
            {
                center_pos = 'E';
            }
            if (ego_vehicle.lane == LaneAssociationType::LEFT_LANE)
            {
                left_pos = 'E';
            }
            if (ego_vehicle.lane == LaneAssociationType::RIGHT_LANE)
            {
                right_pos = 'E';
            }
        }

        std::cout << current_position << "\t"
                  << "|\t" << left_pos << "\t"
                  << "|\t" << center_pos << "\t"
                  << "|\t" << right_pos << "\t"
                  << "|\t" << std::endl;
    }
    std::cout.precision(3);
    std::cout << "E :(" << ego_vehicle.speed_mps << " mps)" << std::endl;
}


void compute_future_state(const VehicleType &ego_vehicle, NeighborVehiclesType &vehicles, const float seconds)
{

    vehicles.vehicles_left[0].distance += (vehicles.vehicles_left[0].speed_mps - ego_vehicle.speed_mps) * seconds;
    vehicles.vehicles_left[1].distance += (vehicles.vehicles_left[1].speed_mps - ego_vehicle.speed_mps) * seconds;
    vehicles.vehicles_center[0].distance += (vehicles.vehicles_center[0].speed_mps - ego_vehicle.speed_mps) * seconds;
    vehicles.vehicles_center[1].distance += (vehicles.vehicles_center[1].speed_mps - ego_vehicle.speed_mps) * seconds;
    vehicles.vehicles_right[0].distance += (vehicles.vehicles_right[0].speed_mps - ego_vehicle.speed_mps) * seconds;
    vehicles.vehicles_right[1].distance += (vehicles.vehicles_right[1].speed_mps - ego_vehicle.speed_mps) * seconds;
}


void decrease_speed(VehicleType &ego_vehicle)
{
    if (ego_vehicle.speed_mps >= 0)
    {
        const auto speed_reduction = ego_vehicle.speed_mps * LONGITUDINAL_DIFFERENCE_PERCENTAGE;
        ego_vehicle.speed_mps -= speed_reduction;
    }
}

void increase_speed(VehicleType &ego_vehicle)
{
    if (ego_vehicle.speed_mps >= 0)
    {
        const auto speed_reduction = ego_vehicle.speed_mps * LONGITUDINAL_DIFFERENCE_PERCENTAGE;
        ego_vehicle.speed_mps += speed_reduction;
    }
}


void longitudinal_control(const VehicleType &front_vehicle, VehicleType &ego_vehicle)
{
    const auto minimal_allowed_distance = mps_to_kph(ego_vehicle.speed_mps) / 2.0f;

    if (ego_vehicle.speed_mps < 0.0f)
    {
        return;
    }
    if (std::abs(front_vehicle.distance) < minimal_allowed_distance)
    {
        decrease_speed(ego_vehicle);
    }
    if (std::abs(front_vehicle.distance) > minimal_allowed_distance && ego_vehicle.speed_mps < kph_to_mps(160.0f))
    {
        increase_speed(ego_vehicle);
    }
}

const VehicleType *get_vehicle_array(const LaneAssociationType lane, const NeighborVehiclesType &vehicles)
{
    const VehicleType *lane_array = nullptr;
    switch (lane)
    {
    case LaneAssociationType::CENTER_LANE:
        lane_array = vehicles.vehicles_center;
        break;
    case LaneAssociationType::RIGHT_LANE:
        lane_array = vehicles.vehicles_right;
        break;
    case LaneAssociationType::LEFT_LANE:
        lane_array = vehicles.vehicles_left;
        break;
    default:
        break;
    }
    return lane_array;
}

LaneAssociationType get_lane_change_request(const VehicleType &ego_vehicle, const NeighborVehiclesType &vehicles)
{
    //If there is enough space on the right lane to make a lane change
    //Else if there is enough space on the left lane, send a request
    //Else, don't do anything
    const auto ego_lane_vehicles = get_vehicle_array(ego_vehicle.lane, vehicles);
    const auto target_distance_other_cars = mps_to_kph(ego_vehicle.speed_mps) / 5;

    bool lane_change_required = false;
    // check if lane change is required
    for (int i = 0; i < VEHICLES_PER_LANE; i++)
    {
        if (std::abs(ego_lane_vehicles[i].distance) < target_distance_other_cars)
        {
            lane_change_required = true;
        }
    }
    if (lane_change_required)
    {
        switch (ego_vehicle.lane)
        {
        case LaneAssociationType::CENTER_LANE:
        {
            const auto vehicles_right = get_vehicle_array(LaneAssociationType::RIGHT_LANE, vehicles);
            const auto vehicles_left = get_vehicle_array(LaneAssociationType::LEFT_LANE, vehicles);
            bool free_right = true;
            bool free_left = true;
            for (int i = 0; i < VEHICLES_PER_LANE; i++)
            {
                if (std::abs(vehicles_right[i].distance) < target_distance_other_cars && free_right)
                {
                    free_right = false;
                }
                if (std::abs(vehicles_left[i].distance) < target_distance_other_cars && free_left)
                {
                    free_left = false;
                }
            }
            if (free_right)
            {
                return LaneAssociationType::RIGHT_LANE;
            }
            else if (free_left)
            {
                return LaneAssociationType::LEFT_LANE;
            }
            else
            {
                return ego_vehicle.lane;
            }
            break;
        }

        case LaneAssociationType::LEFT_LANE:
        case LaneAssociationType::RIGHT_LANE:
        {
            const auto vehicles_center = get_vehicle_array(LaneAssociationType::CENTER_LANE, vehicles);
            bool free_center = true;
            for (int i = 0; i < VEHICLES_PER_LANE; i++)
            {
                if (std::abs(vehicles_center[i].distance) < target_distance_other_cars && free_center)
                {
                    free_center = false;
                }
            }
            if (free_center)
            {
                return LaneAssociationType::CENTER_LANE;
            }
            else
            {
                return ego_vehicle.lane;
            }
            break;
        }
        default:
            return ego_vehicle.lane;
            break;
        }
    }
    else
    {
        return ego_vehicle.lane;
    }
}

bool lateral_control(const LaneAssociationType lane_change_request, VehicleType &ego_vehicle)
{
    if (lane_change_request != ego_vehicle.lane)
    {
        ego_vehicle.lane = lane_change_request;
        return true;
    }

    return false;
}
