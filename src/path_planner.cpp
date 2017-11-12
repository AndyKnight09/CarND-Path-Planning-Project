#include "path_planner.h"
#include "spline.h"

PathPlanner::PathPlanner(const Tools & tools)
    : tools(tools)
{
    lane = 1; // 0 = left, 1 = middle, 2 = right
    ref_vel = 0;
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
    double max_speed = 49.5;
    for (int i = 0; i < data.sensor_fusion.size(); i++)
    {
        // Check if this car is in our lane
        float d = data.sensor_fusion[i][6];
        if ((2 + 4 * lane - 2) < d && d < (2 + 4 * lane + 2))
        {
            // Calculate speed of other car
            double vx = data.sensor_fusion[i][3];
            double vy = data.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            // Predict position of car at point in time that we are planning from (in the future)
            double check_car_s = data.sensor_fusion[i][5];
            check_car_s += ((double)prev_size * 0.02 * check_speed);

            // Check if we will be too close to the car
            if ((check_car_s > data.car_s) && ((check_car_s - data.car_s) < 30))
            {
                // Set flag to say that we need to slow down
                max_speed = fmin(max_speed, check_speed);

                // Change into left lane (if we aren't there already)
                if (lane > 0)
                {
                    lane = 0;
                }
            }
        }
    }

    // Choose acceleration based on whether we are too close to a car in front
    if (ref_vel > max_speed)
    {
        ref_vel -= 0.224;
    }
    else if (ref_vel < max_speed)
    {
        ref_vel += 0.224;
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
    for (int i = 0; i<3; i++)
    {
        // Use Frenet coordinates
        double next_s = data.car_s + (i + 1) * dist_inc;
        double next_d = 2 + 4 * lane;

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

    // Keep existing trajectory
    for (int i = 0; i < prev_size; i++)
    {
        next_x_vals.push_back(data.previous_path_x[i]);
        next_y_vals.push_back(data.previous_path_y[i]);
    }

    // Add new trajectory points using spline
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    // Calculate distance between waypoints
    double N = target_dist / (0.02 * ref_vel / 2.24); // 2.24 m/s to mph

    double x_addon = 0;
    for (int i = 0; i < 50 - prev_size; i++)
    {
        // Move along spline by calculated distance
        x_addon += target_x / N;

        double x_point = x_addon;
        double y_point = s(x_point);

        // Convert into world coordinate frame
        double x_point_world = ref_x + x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
        double y_point_world = ref_y + x_point * sin(ref_yaw) + y_point * cos(ref_yaw);

        next_x_vals.push_back(x_point_world);
        next_y_vals.push_back(y_point_world);
    }
}
