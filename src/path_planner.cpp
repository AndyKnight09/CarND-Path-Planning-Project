#include "path_planner.h"
#include "spline.h"

#include <float.h>
#include <cstdarg>

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
#define MIN_KEEP_LANE_FOLLOW_DISTANCE 15.0 // m
#define MIN_LANE_CHANGE_FOLLOW_DISTANCE 7.5 // m
#define MIN_LANE_CHANGE_GAP 20.0 // m
#define NUM_TRAJECTORY_POINTS 50
#define MIN_LANE_CHANGE_SPEED_INCREASE 5.0 // m/s

PathPlanner::PathPlanner(const Tools & tools)
    : tools(tools)
{
    // Start stationary in the middle lane
    UpdateState(KeepLane, "Initialisation");
    target_vel = TARGET_TOP_SPEED;
    ref_vel = 0;
    ref_lane = MIDDLE_LANE;
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::UpdateTrajectory(MeasurementData & latest_data, vector<double> & next_x_vals, vector<double> & next_y_vals)
{
    // Store latest measurement data
    data = latest_data;

    // Plan trajectory from the end of the current trajectory
    if (data.prev_size() > 0)
    {
        data.car_s = data.end_path_s;
    }

    // Behaviour Planning
    DetermineNextAction();

    // Trajectory generation
    CalculateTrajectory(next_x_vals, next_y_vals);
}

void PathPlanner::DetermineNextAction()
{
    // Check if we are too close to any cars (in our lane)
    target_vel = CalculateLaneSpeed(ref_lane, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_KEEP_LANE_FOLLOW_DISTANCE);

    // Different behaviour based on current state
    switch (state)
    {
        case KeepLane:
        {
            // See if we need to change lane
            if (target_vel < TARGET_TOP_SPEED)
            {
                // Check options
                if (ref_lane == LEFT_LANE)
                {
                    double middle_lane_speed = CalculateLaneSpeed(MIDDLE_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    double right_lane_speed = CalculateLaneSpeed(RIGHT_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    if (ShouldChangeLane(middle_lane_speed) || ShouldChangeLane(right_lane_speed))
                    {
                        if (CanMoveIntoLane(MIDDLE_LANE, MIN_LANE_CHANGE_GAP))
                        {
                            UpdateState(LaneChangeRight, "Current = %.1fm/s, Middle [OK] = %.1fm/s, Right = %.1fm/s", target_vel, middle_lane_speed, right_lane_speed);
                            ref_lane = MIDDLE_LANE;
                            target_vel = middle_lane_speed;
                        }
                        else
                        {
                            double next_middle_speed = CalculateSpeedOfNextCar(MIDDLE_LANE);
                            if (ShouldChangeLane(next_middle_speed))
                            {
                                UpdateState(PrepareLaneChangeRight, "Current = %.1fm/s, Middle [BUSY] = %.1fm/s / %.1fm/s, Right = %.1fm/s", target_vel, middle_lane_speed, next_middle_speed, right_lane_speed);
                            }
                        }
                    }
                }
                else if (ref_lane == RIGHT_LANE)
                {
                    double middle_lane_speed = CalculateLaneSpeed(MIDDLE_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    double left_lane_speed = CalculateLaneSpeed(LEFT_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    if (ShouldChangeLane(middle_lane_speed) || ShouldChangeLane(left_lane_speed))
                    {
                        if (CanMoveIntoLane(MIDDLE_LANE, MIN_LANE_CHANGE_GAP))
                        {
                            UpdateState(LaneChangeLeft, "Current = %.1fm/s, Middle [OK] = %.1fm/s, Left = %.1fm/s", target_vel, middle_lane_speed, left_lane_speed);
                            ref_lane = MIDDLE_LANE;
                            target_vel = middle_lane_speed;
                        }
                        else
                        {
                            double next_middle_speed = CalculateSpeedOfNextCar(MIDDLE_LANE);
                            if (ShouldChangeLane(CalculateSpeedOfNextCar(MIDDLE_LANE)))
                            {
                                UpdateState(PrepareLaneChangeLeft, "Current = %.1fm/s, Middle [BUSY] = %.1fm/s / %.1fm/s, Left = %.1fm/s", target_vel, middle_lane_speed, next_middle_speed, left_lane_speed);
                            }
                        }
                    }
                }
                else // Middle lane
                {
                    // Can move into either left or right lane
                    bool can_move_into_left_lane = CanMoveIntoLane(LEFT_LANE, MIN_LANE_CHANGE_GAP);
                    double left_lane_speed = CalculateLaneSpeed(LEFT_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    bool can_move_into_right_lane = CanMoveIntoLane(RIGHT_LANE, MIN_LANE_CHANGE_GAP);
                    double right_lane_speed = CalculateLaneSpeed(RIGHT_LANE, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP);
                    if (can_move_into_left_lane && can_move_into_right_lane)
                    {
                        if (left_lane_speed > right_lane_speed)
                        {
                            if (ShouldChangeLane(left_lane_speed))
                            {
                                UpdateState(LaneChangeLeft, "Current = %.1fm/s, Left [OK] = %.1fm/s, Right [OK] = %.1fm/s", target_vel, left_lane_speed, right_lane_speed);
                                ref_lane = LEFT_LANE;
                                target_vel = left_lane_speed;
                            }
                        }
                        else if (right_lane_speed > left_lane_speed)
                        {
                            if (ShouldChangeLane(right_lane_speed))
                            {
                                UpdateState(LaneChangeRight, "Current = %.1fm/s, Left [OK] = %.1fm/s, Right [OK] = %.1fm/s", target_vel, left_lane_speed, right_lane_speed);
                                ref_lane = RIGHT_LANE;
                                target_vel = right_lane_speed;
                            }
                        }
                        else // Both lane speeds are the same
                        {
                            // If either lane is better than the current one
                            if (ShouldChangeLane(left_lane_speed))
                            {
                                // Work out which lane has the furthest gap to the next car
                                double distance_to_left_car = CalculateDistanceToNextCar(LEFT_LANE);
                                double distance_to_right_car = CalculateDistanceToNextCar(RIGHT_LANE);
                                if (distance_to_left_car > distance_to_right_car)
                                {
                                    UpdateState(LaneChangeLeft, "Current = %.1fm/s, Left [OK] = %.1fm/s / %.1fm, Right [OK] = %.1fm/s / %.1fm", target_vel, left_lane_speed, distance_to_left_car, right_lane_speed, distance_to_right_car);
                                    ref_lane = LEFT_LANE;
                                    target_vel = left_lane_speed;
                                }
                                else
                                {
                                    UpdateState(LaneChangeRight, "Current = %.1fm/s, Left [OK] = %.1fm/s / %.1fm, Right [OK] = %.1fm/s / %.1fm", target_vel, left_lane_speed, distance_to_left_car, right_lane_speed, distance_to_right_car);
                                    ref_lane = RIGHT_LANE;
                                    target_vel = right_lane_speed;
                                }
                            }
                        }
                    }
                    else if (can_move_into_left_lane && ShouldChangeLane(left_lane_speed))
                    {
                        UpdateState(LaneChangeLeft, "Current = %.1fm/s, Left [OK] = %.1fm/s, Right [BUSY] = %.1fm/s", target_vel, left_lane_speed, right_lane_speed);
                        ref_lane = LEFT_LANE;
                        target_vel = left_lane_speed;
                    }
                    else if (can_move_into_right_lane && ShouldChangeLane(right_lane_speed))
                    {
                        UpdateState(LaneChangeRight, "Current = %.1fm/s, Left [BUSY] = %.1fm/s, Right [OK] = %.1fm/s", target_vel, left_lane_speed, right_lane_speed);
                        ref_lane = RIGHT_LANE;
                        target_vel = right_lane_speed;
                    }
                    else
                    {
                        // Stay where we are - no better options (in the short term at least)
                        UpdateState(KeepLane, "Current = %.1fm/s, Left [???] = %.1fm/s, Right [???] = %.1fm/s", target_vel, left_lane_speed, right_lane_speed);
                    }
                }
            }
        }
        break;

        case LaneChangeLeft:
        {
            // Wait for lane change to complete before starting next manoeuvre
            if(fabs(data.car_d - GetLaneD(ref_lane)) < LANE_WIDTH / 10)
            {
                // Lane change complete
                UpdateState(KeepLane, "Lane change complete");
            }
            else
            {
                // Adjust speed based on current lane position
                target_vel = min(target_vel, CalculateCarSpeed(data.car_d, MIN_LANE_CHANGE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_FOLLOW_DISTANCE));
            }
        }
        break;
        
        case LaneChangeRight:
        {
            // Wait for lane change to complete before starting next manoeuvre
            if (fabs(data.car_d - GetLaneD(ref_lane)) < LANE_WIDTH / 10)
            {
                // Lane change complete
                UpdateState(KeepLane, "Lane change complete");
            }
            else
            {
                // Adjust speed based on current lane position
                target_vel = min(target_vel, CalculateCarSpeed(data.car_d, MIN_LANE_CHANGE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_FOLLOW_DISTANCE));
            }
        }
        break;

        case PrepareLaneChangeLeft: 
        {
            // Slot in behind vehicle in lane to the left
            if (CanMoveIntoLane(ref_lane - 1, MIN_LANE_CHANGE_GAP))
            {
                // Switch lane
                UpdateState(LaneChangeLeft, "Can move into lane");
                ref_lane -= 1;
                target_vel = CalculateLaneSpeed(ref_lane, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_KEEP_LANE_FOLLOW_DISTANCE);
            }
            else
            {
                // Slow down to find a gap
                target_vel = min(target_vel, CalculateLaneSpeed(ref_lane - 1, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP) - 5);
            }
        }
        break;
    
        case PrepareLaneChangeRight: 
        {
            // Slot in behind vehicle in lane to the right
            if (CanMoveIntoLane(ref_lane + 1, MIN_LANE_CHANGE_GAP))
            {
                // Switch lane
                UpdateState(LaneChangeRight, "Can move into lane");
                ref_lane += 1;
                target_vel = CalculateLaneSpeed(ref_lane, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_KEEP_LANE_FOLLOW_DISTANCE);
            }
            else
            {
                // Slow down to find a gap
                target_vel = min(target_vel, CalculateLaneSpeed(ref_lane + 1, MIN_KEEP_LANE_FOLLOW_DISTANCE, MIN_LANE_CHANGE_GAP) - 5);
            }
        }
        break;
    }
}

bool PathPlanner::ShouldChangeLane(double predictedLaneSpeed)
{
    return predictedLaneSpeed > (target_vel + MIN_LANE_CHANGE_SPEED_INCREASE);
}

double PathPlanner::CalculateLaneSpeed(int lane, double follow_distance, double gap_size)
{
    return CalculateCarSpeed(LANE_WIDTH / 2 + LANE_WIDTH * lane, follow_distance, gap_size);
}

double PathPlanner::CalculateCarSpeed(double d, double follow_distance, double gap_size)
{
    double speed = TARGET_TOP_SPEED;
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float check_d = data.sensor_fusion[i][6];
        if ((d - CAR_WIDTH / 2) < check_d && check_d < (d + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)data.prev_size() * DT * check_car_speed);

            // Check if car is near to us
            double diff_s = check_car_s - data.car_s;
            if ((diff_s > (follow_distance - gap_size)) && (diff_s < follow_distance))
            {
                // See if this car is slower
                speed = fmin(speed, check_car_speed * MPS_TO_MPH);
            }
        }
    }
    return speed;
}

double PathPlanner::CalculateDistanceToNextCar(int lane)
{
    double min_distance = 10000.0;
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((LANE_WIDTH / 2 + LANE_WIDTH * lane - CAR_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * lane + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)data.prev_size() * DT * check_car_speed);

            // Check if this car is in front of us and closer than any other cars seen so far
            double distance_to_car = check_car_s - data.car_s;
            if (0 < distance_to_car && distance_to_car < min_distance)
            {
                min_distance = distance_to_car;
            }
        }
    }
    return min_distance;
}

double PathPlanner::CalculateSpeedOfNextCar(int lane)
{
    double min_distance = 10000.0;
    double speed = TARGET_TOP_SPEED;
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((LANE_WIDTH / 2 + LANE_WIDTH * lane - CAR_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * lane + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)data.prev_size() * DT * check_car_speed);

            // Check if this car is in front of us and closer than any other cars seen so far
            double distance_to_car = check_car_s - data.car_s;
            if (0 < distance_to_car && distance_to_car < min_distance)
            {
                min_distance = distance_to_car;
                speed = check_car_speed;
            }
        }
    }
    return speed;
}

bool PathPlanner::CanMoveIntoLane(int lane, double gap_size)
{
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((GetLaneD(lane) - CAR_WIDTH / 2) < d && d < (GetLaneD(lane) + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)data.prev_size() * DT * check_speed);

            // Check if car is within lane change gap
            double diff_s = check_car_s - data.car_s;
            if ((diff_s > (MIN_KEEP_LANE_FOLLOW_DISTANCE - gap_size)) && (diff_s < MIN_KEEP_LANE_FOLLOW_DISTANCE))
            {
                return false;
            }
        }
    }

    return true;
}

void PathPlanner::CalculateTrajectory(vector<double> & next_x_vals, vector<double> & next_y_vals)
{
    // Define sparse waypoints (30m apart) used to construct spline trajectory
    vector<double> ptsx, ptsy;

    // Reference starting point for car
    double ref_x, ref_y, ref_yaw;

    // Check if we have a previous trajectory to work from
    if (data.prev_size() < 2)
    {
        // Not enough points to work from so use current car state to define first two points for spline
        ref_x = data.car_x;
        ref_y = data.car_y;
        ref_yaw = tools.deg2rad(data.car_yaw);

        // Calculate previous position based on current heading of car
        double prev_car_x = data.car_x - cos(data.car_yaw);
        double prev_car_y = data.car_y - sin(data.car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(data.car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(data.car_y);
    }
    else
    {
        // Use existing trajectory to define first two points for spline
        ref_x = data.previous_path_x[data.prev_size() - 1];
        ref_y = data.previous_path_y[data.prev_size() - 1];

        double ref_x_prev = data.previous_path_x[data.prev_size() - 2];
        double ref_y_prev = data.previous_path_y[data.prev_size() - 2];

        // Calculate reference yaw based on last two points in trajectory
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Add 3 more points to spline, spaced 30m apart
    double dist_inc = 30;
    for (int i = 0; i < 3; i++)
    {
        // Use Frenet coordinates
        double next_s = data.car_s + (i + 1) * dist_inc;
        double next_d = GetLaneD(ref_lane);

        // Convert to cartesian coordinates
        vector<double> xy = tools.getXY(next_s, next_d);

        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);
    }

    // Convert all waypoints into vehicle frame of reference
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
        ptsy[i] = -shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw);
    }

    // Construct spline from waypoints
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // Start from existing trajectory
    for (int i = 0; i < data.prev_size(); i++)
    {
        next_x_vals.push_back(data.previous_path_x[i]);
        next_y_vals.push_back(data.previous_path_y[i]);
    }

    // Add new trajectory points using spline
    double x_addon = 0;
    for (int i = 0; i < NUM_TRAJECTORY_POINTS - data.prev_size(); i++)
    {
        // Choose acceleration based on whether we are too close to a car in front
        if (ref_vel > target_vel)
        {
            ref_vel -= MAX_ACCELERATION * DT * MPS_TO_MPH;
        }
        else if (ref_vel < target_vel)
        {
            ref_vel += MAX_ACCELERATION * DT * MPS_TO_MPH;
        }

        // Move along spline by calculated distance
        x_addon += ref_vel * MPH_TO_MPS * DT;

        double x_point = x_addon;
        double y_point = s(x_point);

        // Convert into world coordinate frame
        double x_point_world = ref_x + x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
        double y_point_world = ref_y + x_point * sin(ref_yaw) + y_point * cos(ref_yaw);

        next_x_vals.push_back(x_point_world);
        next_y_vals.push_back(y_point_world);
    }
}

double PathPlanner::GetLaneD(int lane)
{
    return LANE_WIDTH / 2 + LANE_WIDTH * lane;
}

void PathPlanner::UpdateState(State newState, const char * format, ...)
{
    if (state != newState)
    {
        state = newState;

        // Display state change
        switch (state)
        {
            case KeepLane: cout << "State -> KeepLane"; break;
            case LaneChangeLeft: cout << "State -> LaneChangeLeft"; break;
            case LaneChangeRight: cout << "State -> LaneChangeRight"; break;
            case PrepareLaneChangeLeft: cout << "State -> PrepareLaneCangeLeft"; break;
            case PrepareLaneChangeRight: cout << "State -> PrepareLaneCangeRight"; break;
        }

        // Display custom message
        va_list args;
        va_start(args, format);

        char message[1024];
        vsprintf(message, format, args);

        cout << " - " << message << endl;

        va_end(args);
    }
}
