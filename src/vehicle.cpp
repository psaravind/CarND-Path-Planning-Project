#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>

Vehicle::Vehicle(int _lane, 
	double _s, 
	double _v, 
	string _state) {
    
	lane = _lane;
	s = _s;
	v = _v;
	state = _state;

	a = 0;
	max_acceleration = -1;
}

Vehicle::Vehicle(int _lane, 
	double _s, 
	double _v, 
	double _a,
	double _max_speed,
	int _num_lanes,
	double _max_acceleration,
	double _min_car_distance,
	string _state) {
    
	lane = _lane;
	s = _s;
	v = _v;
	a = _a;
	max_speed = _max_speed;
	num_lanes = _num_lanes;
	max_acceleration = _max_acceleration;
	min_car_distance = _min_car_distance;
	state = _state;

	cost = Cost();
}

Vehicle::~Vehicle() {}

vector<Prediction> Vehicle::generate_predictions(int horizon) {
	vector<Prediction> predictions;
	
	for (int i = 0; i < horizon; i++) {
		Prediction prediction = state_at(i);

		predictions.push_back(prediction);
	}

	return predictions;
}

Prediction Vehicle::state_at(int t) {

	Prediction prediction;
	
	double dt = double(t);
	prediction.lane = lane;
	prediction.s = s + v * dt + a * dt * dt / 2;
	prediction.v = max(0.0, v + a * dt);
	prediction.a = a;

	return prediction;
}

void Vehicle::update_state(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions) {

	vector<string> next_states = {};

	if (state.compare("KL") == 0) {
		if (lane == 0)
			next_states = {"KL", "PLCR"};
		else if (lane == num_lanes - 1)
			next_states = {"KL", "PLCL"};
		else
			next_states = {"KL", "PLCL", "PLCR"};
	}

	if (state.compare("PLCL") == 0)
		next_states = {"KL", "PLCL", "LCL"};

	if (state.compare("PLCR") == 0)
		next_states = {"KL", "PLCR", "LCR"};

	if (state.compare("LCR") == 0)
		next_states = {"KL", "PLCR"};
	if (state.compare("LCL") == 0)
		next_states = {"KL", "PLCL"};

	map<string, double> new_costs;
	for (auto _state : next_states) {
		vector<Snapshot> trajectories = trajectories_for_state(_state, 
			ego_predictions, 
			vehicle_predictions, 
			50);
		new_costs[_state] = cost.calculate_cost(*this,
			_state,
			num_lanes,
			trajectories, 
			ego_predictions,
			vehicle_predictions);
	}

	double min = std::numeric_limits<double>::infinity();
	for (auto _state : next_states) {

		if (new_costs[_state] < min) {
			min = new_costs[_state];
			state = _state;
		}
	}

	if (state.compare("KL") == 0 || state.compare("PLCL") == 0) {
		if ((new_costs.count("KL") > 0) && (new_costs.count("PLCL") > 0)) {
			double f = fabs(new_costs["KL"] - new_costs["PLCL"]);
			if (f < std::numeric_limits<double>::epsilon())
				state = "KL";
		}
	}

	if (state.compare("KL") == 0 || state.compare("PLCR") == 0) {
		if ((new_costs.count("KL") > 0) && (new_costs.count("PLCR") > 0)) {
			double f = fabs(new_costs["KL"] - new_costs["PLCR"]);
			if (f < std::numeric_limits<double>::epsilon())
				state = "KL";
		}
	}

	if (state.compare("LCL") == 0 || state.compare("PLCL") == 0) {
		if ((new_costs.count("LCL") > 0) && (new_costs.count("PLCL") > 0)) {
			double f = fabs(new_costs["LCL"] - new_costs["PLCL"]);
			if (f < std::numeric_limits<double>::epsilon())
				state = "LCL";
		}
	}

	if (state.compare("LCR") == 0 || state.compare("PLCR") == 0) {
		if ((new_costs.count("LCR") > 0) && (new_costs.count("PLCR") > 0)) {
			double f = fabs(new_costs["LCR"] - new_costs["PLCR"]);
			if (f < std::numeric_limits<double>::epsilon())
				state = "LCR";
		}
	}
}

vector<Snapshot> Vehicle::trajectories_for_state(string _state, 
	vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions, 
	int horizon) {

	Snapshot snapshot = Snapshot(lane, s, v, a, state);
	vector<Snapshot> trajectories;

	state = _state;
	trajectories.push_back(snapshot);

	for (int i = 0; i < horizon; i++) {
		advance(1);

		trajectories.push_back(Snapshot(lane, s, v, a, state));
	}

	lane = snapshot.lane;
	s = snapshot.s;
	v = snapshot.v;
	a = snapshot.a;

	return trajectories;
}

void Vehicle::advance(int dt) {
	double ddt = double(dt) * 0.02;

	a = max(-max_acceleration, min(max_acceleration, a));

	s += v * ddt;  
	v += a * ddt * 50;

	v = max(0.0, min(max_speed, v));
}

void Vehicle::realize_state(vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions) {

	if (state.compare("CS") == 0)
		realize_constant_speed();
	else if (state.compare("KL") == 0)
		realize_keep_lane(ego_predictions,
			vehicle_predictions);
	else if (state.compare("LCL") == 0)
		realize_lane_change(ego_predictions,
			vehicle_predictions, "L");
	else if (state.compare("LCR") == 0)
		realize_lane_change(ego_predictions,
			vehicle_predictions, "R");
	else if (state.compare("PLCL") == 0)
		realize_prep_lane_change(ego_predictions,
			vehicle_predictions, "L");
	else if (state.compare("PLCR") == 0)
		realize_prep_lane_change(ego_predictions,
			vehicle_predictions, "R");
}

void Vehicle::realize_constant_speed() {
	a = 0;
}

void Vehicle::realize_keep_lane(vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions) {
	a = _max_accel_for_lane(lane,
		ego_predictions,
		vehicle_predictions);
}

void Vehicle::realize_lane_change(vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions, 
	string direction) {
		
	double delta = 1;
	if (direction.compare("L") == 0)
		delta = -1;

	lane += delta;

	lane = max(0, min(num_lanes - 1, lane));

	a = _max_accel_for_lane(lane,
		ego_predictions,
		vehicle_predictions);
}

double Vehicle::_max_accel_for_lane(int _lane, 
	vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions) {
	tk::spline buffer_spline;
	vector<double> buffer_x = {0, 22.5, 50};
	vector<double> buffer_y = {10, 24, 38};
	buffer_spline.set_points(buffer_x, buffer_y);

	tk::spline accel_spline;
	vector<double> accel_x = {0, 25.0, 50};
	vector<double> accel_y = {1.6, .9, 0.5};
	accel_spline.set_points(accel_x, accel_y);

	vector<Prediction> in_front;

	int preferred_buffer = buffer_spline(v);

	for (auto val: vehicle_predictions) {
		vector<Prediction> prediction = val.second;
		Prediction first_Prediction = prediction[0];
		
		if (first_Prediction.lane == _lane && first_Prediction.s > s)
			in_front.push_back(first_Prediction);
	}
 
	if (in_front.size() > 0) {
		double min_s = 9999999.0;

		for (int i = 0; i < in_front.size(); i++) {
			if (in_front[i].s < min_s)
				min_s = in_front[i].s;
		}

		double available_room = min_s - s;
		if (available_room < preferred_buffer)
			return -max_acceleration;
	}

	return accel_spline(v);
}

void Vehicle::realize_prep_lane_change(vector<Prediction> ego_predictions,
	map<int, vector<Prediction>> vehicle_predictions, 
	string direction) {

	int delta = 1;
	if (direction.compare("L") == 0)
		delta = -1;

	int _lane = lane + delta;

	vector<Prediction> at_behind;
	for (auto val: vehicle_predictions) {
		vector<Prediction> prediction = val.second;
		Prediction first_Prediction = prediction[0];

		if (first_Prediction.lane == _lane && first_Prediction.s <= s)
			at_behind.push_back(first_Prediction);
	}

	if (at_behind.size() > 0) {
		int max_s = s - min_car_distance;

		vector<Prediction> nearest_behind;

		for (int i = 0; i < at_behind.size(); i++) {
			Prediction prediction = at_behind[i];
			if (prediction.s > max_s) {
				max_s = prediction.s;
				nearest_behind.push_back(prediction);
			}
		}

		double delta_v = 0;
		double delta_s = s;
		if (nearest_behind.size() > 1) {
			delta_v = v - nearest_behind[1].v;
			delta_s = s - nearest_behind[1].s;
		} else if (nearest_behind.size() == 1) {
			delta_v = v - nearest_behind[0].v;
			delta_s = s - nearest_behind[0].s;
		}

		if (delta_v != 0) {
			double time = -2.0 * delta_s / delta_v;
			double aa;

			if (time == 0)
				aa = a;
			else
				aa = delta_v / time;

			a = min(-max_acceleration, max(aa, max_acceleration));

			return;
		}
	}

	a = _max_accel_for_lane(_lane, ego_predictions, vehicle_predictions);
}