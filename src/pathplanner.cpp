#include "pathplanner.h"

PathPlanner::PathPlanner(int num_lanes,
	double _car_max_vel,
	vector<double> _map_waypoints_x,
	vector<double> _map_waypoints_y,
	vector<double> _map_waypoints_s,
	vector<double> _map_waypoints_dx,
	vector<double> _map_waypoints_dy):road(num_lanes, 1, 
		0, 
		_car_max_vel,
		3,
		50) {

	car_max_vel = _car_max_vel;
	map_waypoints_x = _map_waypoints_x;
	map_waypoints_y = _map_waypoints_y;
	map_waypoints_s = _map_waypoints_s;
	map_waypoints_dx = _map_waypoints_dx;
	map_waypoints_dy = _map_waypoints_dy;
}

PathPlanner::~PathPlanner() {}

vector<double> PathPlanner::GeneratePath(vector<double> car_data,
	vector<vector<double>> _sensor_fusion,
	vector<vector<double>> path_data,
	vector<double> _end_path_sd) {

	car_x = car_data[0];
	car_y = car_data[1];
	car_s = car_data[2];
	car_d = car_data[3];
	car_yaw = car_data[4];
	car_speed = car_data[5];

	sensor_fusion = _sensor_fusion;
	previous_path_x = path_data[0];
	previous_path_y = path_data[1];
	prev_size = previous_path_x.size();
	end_path_sd = _end_path_sd;

	if (prev_size > 0) 
		car_s = end_path_sd[0];

	road.populate_traffic(sensor_fusion);
	road.advance(car_s);

	vector<double> path;

	car_ref_vel = road.ego.v;
	car_lane = road.ego.lane;

	cout << "Ego: S = " << road.ego.s
		<< ", Lane = " << road.ego.lane
		<< ", Accel = " << road.ego.a
		<< ", Velocity = " << road.ego.v
		<< ", State = " << road.ego.state
		<< endl;

	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	if (prev_size < 2) {
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	} else {
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i++) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

	tk::spline s;
			
	s.set_points(ptsx, ptsy);

	for (int i = 0; i < previous_path_x.size(); i++) {
		path.push_back(previous_path_x[i]);
		path.push_back(previous_path_y[i]);
	}

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

	double x_add_on = 0;  
	
	for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
		double N = (target_dist / (.02 * car_ref_vel / 2.24));
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		path.push_back(x_point);
		path.push_back(y_point);
	}

	return path;
}