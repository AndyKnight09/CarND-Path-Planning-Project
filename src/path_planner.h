#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "tools.h"
#include "json.hpp"

struct MeasurementData
{
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the Planner
    nlohmann::basic_json<>::value_type previous_path_x;
    nlohmann::basic_json<>::value_type previous_path_y;
    
    // Previous path's end s and d values 
    double end_path_s;
    double end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    nlohmann::basic_json<>::value_type sensor_fusion;
};

class PathPlanner
{
public:
    PathPlanner(const Tools & tools);
    ~PathPlanner();

public:
    void UpdateTrajectory(MeasurementData & data, vector<double> & next_x_vals, vector<double> & next_y_vals);
    double CalculateLaneSpeed(int lane, const MeasurementData & data, int prev_size);
    bool CanMoveIntoLane(int lane, const MeasurementData & data, int prev_size);

private:
    const Tools & tools;

    int ref_lane;
    double ref_vel;
};

#endif
