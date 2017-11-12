#include "path_planner.h"
#include "spline.h"

#define MPS_TO_MPH 2.23694  // Conversion between m/s and mph
#define MPH_TO_MPS (1 / MPS_TO_MPH)

#define TARGET_SPEED 49.5 // mph
#define MAX_ACCELERATION 5.0 // m/s^2
#define DT 0.02 // Time step (s)
#define LANE_WIDTH 4.0 // m
#define CAR_WIDTH 3.0 // m
#define MIN_FOLLOW_DISTANCE 25.0 // m
#define MIN_LANE_CHANGE_GAP 25.0 // m
#define NUM_TRAJECTORY_POINTS 50

#define LEFT_LANE 0
#define MIDDLE_LANE 1
#define RIGHT_LANE 2

PathPlanner::PathPlanner(const Tools & tools)
    : tools(tools)
{
    // Start stationary in the middle lane
    ref_vel = 0;
    ref_lane = MIDDLE_LANE;
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::UpdateTrajectory(MeasurementData & data, vector<double> & next_x_vals, vector<double> & next_y_vals)
{
    int prev_size = data.previous_path_x.size();

    // Plan trajectory from the end of the current trajectory
    if (prev_size > 0)
    {
        data.car_s = data.end_path_s;
    }

    // Check if we are too close to any cars (in our lane)
    double max_speed = CalculateLaneSpeed(ref_lane, data, prev_size);

    // See if we need to change lane
    if (max_speed < TARGET_SPEED)
    {
        // Check options
        if (ref_lane == LEFT_LANE || ref_lane == RIGHT_LANE)
        {
            bool can_move_into_middle_lane = CanMoveIntoLane(MIDDLE_LANE, data, prev_size);
            double middle_lane_speed = CalculateLaneSpeed(MIDDLE_LANE, data, prev_size);
            if (can_move_into_middle_lane && middle_lane_speed > max_speed)
            {
                ref_lane = MIDDLE_LANE;
                max_speed = middle_lane_speed;
            } 
        }
        else
        {
            // Can move into either left or right lane
            bool can_move_into_left_lane = CanMoveIntoLane(LEFT_LANE, data, prev_size);
            double left_lane_speed = CalculateLaneSpeed(LEFT_LANE, data, prev_size);
            bool can_move_into_right_lane = CanMoveIntoLane(RIGHT_LANE, data, prev_size);
            double right_lane_speed = CalculateLaneSpeed(RIGHT_LANE, data, prev_size);
            if (can_move_into_left_lane && can_move_into_right_lane)
            {
                if (left_lane_speed > right_lane_speed)
                {
                    if (left_lane_speed > max_speed)
                    {
                        ref_lane = LEFT_LANE;
                        max_speed = left_lane_speed;
                    }
                }
                else
                {
                    if (right_lane_speed > max_speed)
                    {
                        ref_lane = RIGHT_LANE;
                        max_speed = right_lane_speed;
                    }
                }
            }
            else if (can_move_into_left_lane && left_lane_speed > max_speed)
            {
                ref_lane = LEFT_LANE;
                max_speed = left_lane_speed;
            }
            else if (can_move_into_right_lane && right_lane_speed > max_speed)
            {
                ref_lane = RIGHT_LANE;
                max_speed = right_lane_speed;
            }
            else
            {
                // Stay where we are - no better options (in the short term at least)
            }
        }
    }

    

    // Define sparse waypoints (30m apart) used to construct spline trajectory
    vector<double> ptsx, ptsy;

    // Reference starting point for car
    double ref_x, ref_y, ref_yaw;

    // Check if we have a previous trajectory to work from
    if (prev_size < 2)
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
        ref_x = data.previous_path_x[prev_size - 1];
        ref_y = data.previous_path_y[prev_size - 1];

        double ref_x_prev = data.previous_path_x[prev_size - 2];
        double ref_y_prev = data.previous_path_y[prev_size - 2];

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
        double next_d = LANE_WIDTH / 2 + LANE_WIDTH * ref_lane;

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
    for (int i = 0; i < prev_size; i++)
    {
        next_x_vals.push_back(data.previous_path_x[i]);
        next_y_vals.push_back(data.previous_path_y[i]);
    }

    // Add new trajectory points using spline
    double x_addon = 0;
    for (int i = 0; i < NUM_TRAJECTORY_POINTS - prev_size; i++)
    {
        // Choose acceleration based on whether we are too close to a car in front
        if (ref_vel > max_speed)
        {
            ref_vel -= MAX_ACCELERATION * DT * MPS_TO_MPH;
        }
        else if (ref_vel < max_speed)
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

double PathPlanner::CalculateLaneSpeed(int lane, const MeasurementData & data, int prev_size)
{
    double lane_speed = TARGET_SPEED;
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((LANE_WIDTH / 2 + LANE_WIDTH * lane - CAR_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * lane + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)prev_size * DT * check_speed);

            // Check if we will be too close to the car
            if ((check_car_s > data.car_s) && ((check_car_s - data.car_s) < MIN_FOLLOW_DISTANCE))
            {
                // Set flag to say that we need to slow down
                lane_speed = fmin(lane_speed, check_speed);
            }
        }
    }
    return lane_speed;
}

bool PathPlanner::CanMoveIntoLane(int lane, const MeasurementData & data, int prev_size)
{
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((LANE_WIDTH / 2 + LANE_WIDTH * lane - CAR_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * lane + CAR_WIDTH / 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)prev_size * DT * check_speed);

            // Check if we will be too close to the car
            if (((check_car_s - data.car_s) > (MIN_FOLLOW_DISTANCE - MIN_LANE_CHANGE_GAP)) && ((check_car_s - data.car_s) < MIN_FOLLOW_DISTANCE))
            {
                return false;
            }
        }
    }

    return true;
}
