#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "tools.h"
#include "json.hpp"

#define MPS_TO_MPH 2.23694  // Conversion between m/s and mph
#define MPH_TO_MPS (1 / MPS_TO_MPH)
#define DT 0.02 // Time step (s)

#define LEFT_LANE 0
#define MIDDLE_LANE 1
#define RIGHT_LANE 2

#define TARGET_TOP_SPEED 49.5 // mph
#define MAX_ACCELERATION 7.5 // m/s^2
#define LANE_WIDTH 4.0 // m
#define CAR_WIDTH 3.0 // m
#define MIN_KEEP_LANE_FOLLOW_DISTANCE 20.0 // m
#define MIN_LANE_CHANGE_FOLLOW_DISTANCE 10.0 // m
#define MIN_LANE_CHANGE_GAP 25.0 // m
#define NUM_TRAJECTORY_POINTS 50
#define MIN_LANE_CHANGE_SPEED_INCREASE 5.0 // m/s

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
    
    int prev_size() const { return previous_path_x.size(); }

    // Previous path's end s and d values 
    double end_path_s;
    double end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    nlohmann::basic_json<>::value_type sensor_fusion;
};

enum State
{
    KeepLane,
    LaneChangeLeft,
    LaneChangeRight,
    PrepareLaneChangeLeft,
    PrepareLaneChangeRight
};

class PathPlanner
{
public:
    PathPlanner(const Tools & tools);
    ~PathPlanner();

public:
    void UpdateTrajectory(MeasurementData & data, vector<double> & next_x_vals, vector<double> & next_y_vals);
    
private:
    void DetermineNextAction();
    bool ShouldChangeLane(double predictedLaneSpeed, double min_lane_change_speed_increase = MIN_LANE_CHANGE_SPEED_INCREASE);
    double CalculateLaneSpeed(int lane, double follow_distance, double gap_size);
    double CalculateCarSpeed(double d, double follow_distance, double gap_size);
    double CalculateDistanceToNextCar(int lane);
    double CalculateSpeedOfNextCar(int lane);
    bool CanMoveIntoLane(int lane, double gap_size);
    void CalculateTrajectory(vector<double> & next_x_vals, vector<double> & next_y_vals);
    double GetLaneD(int lane);
    void UpdateState(State newState, const char * format, ...);

private:
    const Tools & tools;

    State state;
    MeasurementData data;

    double target_vel;
    int ref_lane;
    double ref_vel;
};

#endif
