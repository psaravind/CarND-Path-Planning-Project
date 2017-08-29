#include "cost.h"
#include <math.h>
#include <map>
#include <string>
#include <iostream>
#include "vehicle.h"
#include "snapshot.h"
#include "helper.h"

Cost::Cost() {}
Cost::~Cost() {}

double Cost::calculate_cost(const Vehicle &ego, 
	string state,
	int num_lanes,
	vector<Snapshot> trajectories,
	vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions) {
	TrajectoryData trajectory_data = get_helper_data(ego,
		ego_predictions, 
		state,
		num_lanes,
		trajectories, 
		vehicle_predictions);
		
	double cost = 0.0;
	
	cost += inefficiency_cost(ego, trajectory_data);
	cost += collision_cost(ego, trajectory_data);
	cost += buffer_cost(ego, trajectory_data);
	if (state.compare("KL") != 0)
		cost += change_lane_cost(ego, trajectory_data);

	return cost;
}

TrajectoryData Cost::get_helper_data(Vehicle ego,
	vector<Prediction> ego_predictions, 
	string state,
	int num_lanes,
	vector<Snapshot> trajectories, 
	map<int, vector<Prediction>> predictions) {

	int min_car_distance = 120;
	TrajectoryData trajectory_data;
	Snapshot current_snapshot = trajectories[0];
	Snapshot first = trajectories[1];
	Snapshot last = trajectories.back();
	double dt = trajectories.size();
	vector<double> accels;

	double delta = 1;
	if (state.compare("KL") == 0)
		delta = 0;
	else if (state.compare("PLCL") == 0 || state.compare("LCL") == 0)
		delta = -1;

	trajectory_data.proposed_lane = ego.lane + delta;
	trajectory_data.proposed_lane = max(0, min(num_lanes - 1, trajectory_data.proposed_lane));
	trajectory_data.avg_speed = (last.s - current_snapshot.s) / dt;

	map<int, vector<Prediction>> filtered = filter_predictions_by_lane(predictions, 
		trajectory_data.proposed_lane);

	trajectory_data.c_front = false;
	trajectory_data.c_front_at = 0;
	vector<Prediction> in_front;

	for (auto val: filtered) {
		vector<Prediction> prediction = val.second;
		Prediction first_Prediction = prediction[1];
		
		if (first_Prediction.s > ego.s)
			in_front.push_back(first_Prediction);
	}
  
	if (in_front.size() > 0) {
		double min_s = min_car_distance;

		for (int i = 0; i < in_front.size(); i++) {
			if (in_front[i].s - ego_predictions[1].s < min_s) {
				trajectory_data.c_front = true;
				trajectory_data.c_front_at = in_front[i].s;
				min_s = in_front[i].s - ego_predictions[1].s;
			}
		}
	}

	trajectory_data.c_back = false;
	trajectory_data.c_back_at = 0;
	vector<Prediction> at_behind;
	
	if (state.compare("KL") != 0) {
		for (auto val: filtered) {
			vector<Prediction> prediction = val.second;
			Prediction first_Prediction = prediction[1];
			
			if (first_Prediction.s <= ego.s)
				at_behind.push_back(first_Prediction);
		}

		if (at_behind.size() > 0) {
			double max_s = -999999.0;

			for (int i = 0; i < at_behind.size(); i++) {
				Prediction prediction = at_behind[i];
				if (prediction.s > max_s)
					max_s = prediction.s;
			}
			if ( max_s > ego_predictions[0].s - min_car_distance) {
					trajectory_data.c_back = true;
					trajectory_data.c_back_at = max_s;
			}
		}
	}

	if (trajectory_data.c_front == true)
		trajectory_data.max_acceleration = -ego.max_acceleration;
	else {
		int num_accels = accels.size();
		trajectory_data.max_acceleration = 0;
		for (int i = 0; i < num_accels; i++) {
			if (abs(accels[i]) > trajectory_data.max_acceleration) 
				trajectory_data.max_acceleration = abs(accels[i]);
		}
	}
	return trajectory_data;
}

map<int, vector<Prediction>> Cost::filter_predictions_by_lane(map<int, vector<Prediction>> predictions, 
	int lane) {
  
	map<int, vector<Prediction>> filtered;

	for (auto val: predictions) {
		if (val.second[0].lane == lane) {
			filtered[val.first] = val.second;
		}
	}

	return filtered;
}

bool Cost::check_collision(Snapshot snapshot, 
	double s_previous, 
	double s_now) {

	double v_target = s_now - s_previous;

	if (s_previous < snapshot.s) 
		return (s_now >= snapshot.s);
  
	if (s_previous > snapshot.s) 
		return (s_now <= snapshot.s);

	if (s_previous == snapshot.s) 
		return (v_target <= snapshot.v);

	return false;
}

double Cost::inefficiency_cost(Vehicle ego, 
	TrajectoryData data) {

	double pct = (ego.max_speed - data.avg_speed) / ego.max_speed;
	double multiplier = pct * pct;
  
	return multiplier * EFFICIENCY;
}

double Cost::collision_cost(Vehicle ego,
	TrajectoryData data) {

	if (data.c_front) {
		double buffer = ego.s - data.c_front_at;
		double multiplier = exp(buffer);

		return multiplier * COLLISION;
	}

	return 0;
}

double Cost::buffer_cost(Vehicle ego,
	TrajectoryData data) {

	tk::spline buffer_spline;
	vector<double> buffer_x = {0, 22.5, 50};
	vector<double> buffer_y = {10, 23, 36};
	buffer_spline.set_points(buffer_x, buffer_y);
	int _DESIRED_BUFFER = buffer_spline(ego.v);

	if (!data.c_front)
		return 0;

	double buffer = data.c_front_at - ego.s;

	if (buffer == _DESIRED_BUFFER) 
		return 10 * DANGER;

	if (buffer > _DESIRED_BUFFER) 
		return 0;

	double multiplier = 1 - pow((buffer / _DESIRED_BUFFER), 2);

	return multiplier * DANGER;
}

double Cost::change_lane_cost(Vehicle ego,
	TrajectoryData data) {

	tk::spline buffer_spline;
	vector<double> buffer_x = {0, 22.5, 50};
	vector<double> buffer_y = {10, 17, 25};
	buffer_spline.set_points(buffer_x, buffer_y);
	int _DESIRED_BUFFER = buffer_spline(ego.v);

	if (!data.c_back)
		return 0;

	double buffer = ego.s - data.c_back_at;

	if (buffer > _DESIRED_BUFFER) 
		return 0;

	double multiplier = 1 - pow((buffer / _DESIRED_BUFFER), 2);

	return multiplier * DANGER;
}