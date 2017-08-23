#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "pathplanner.h"
#include "road.h"
#include <iostream>
#include "spline.h"
#include "helper.h"

using namespace std;

class PathPlanner {
	private:
		vector<double> map_waypoints_x;
		vector<double> map_waypoints_y;
		vector<double> map_waypoints_s;
		vector<double> map_waypoints_dx;
		vector<double> map_waypoints_dy;

		double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;

		int car_lane = 1;

		double car_ref_vel = 0.1;
		double max_speed;

		vector<vector<double>> sensor_fusion;
		vector<double> previous_path_x;
		vector<double> previous_path_y;
		int prev_size;
		vector<double> end_path_sd;

 public:
 	Road road;
	PathPlanner(int num_lanes,
		double car_max_vel,
		double min_car_distance,
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy);

	~PathPlanner();

	vector<vector<double>> GeneratePath(vector<double> car_data,
		vector<vector<double>> sensor_fusion,
		vector<vector<double>> path_data,
		vector<double> end_path_sd);
};

#endif /* PATH_PLANNER_H */