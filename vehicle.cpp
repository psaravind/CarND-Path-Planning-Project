#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include "snapshot.h"
#include "cost.h"
#include "helper.h"

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
		cout << "state:" << _state;
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
		cout << _state << " " << new_costs[_state] << " ";
		if (new_costs[_state] < min) {
			min = new_costs[_state];
			state = _state;
		}
	}
	
	if ((new_costs.count("KL") > 0) && (new_costs.count("PLCL") > 0)) {
		double f = fabs(new_costs["KL"] - new_costs["PLCL"]);
		if (f < std::numeric_limits<double>::epsilon()) {
			state = "KL";
			cout << " fabs1:" << f << " ";
		}
	}

	if ((new_costs.count("KL") > 0) && (new_costs.count("PLCR") > 0)) {
		double f = fabs(new_costs["KL"] - new_costs["PLCR"]);
		if (f < std::numeric_limits<double>::epsilon()) {
			state = "KL";
			cout << " fabs2:" << f << " ";
		}
	}
	
	
	if ((new_costs.count("LCL") > 0) && (new_costs.count("PLCL") > 0)) {
		double f = fabs(new_costs["LCL"] - new_costs["PLCL"]);
		if (f < std::numeric_limits<double>::epsilon()) {
			state = "LCL";
			cout << " fabs1:" << f << " ";
		}
	}

	if ((new_costs.count("LCR") > 0) && (new_costs.count("PLCR") > 0)) {
		double f = fabs(new_costs["LCR"] - new_costs["PLCR"]);
		if (f < std::numeric_limits<double>::epsilon()) {
			state = "LCR";
			cout << " fabs4:" << f << " ";
		}
	}

	cout << "min state:" << state << " value:" << min << " s:" << s << endl;
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

	a = max(-max_acceleration, min(max_acceleration, a));//max(-.6, min(2.5, a)); //a = max(-.6, min(1.0, a));
	
	s += v * ddt;  
	v += a * ddt * 50;

	v = max(0.0, min(max_speed, v));
}

bool Vehicle::collides_with(Vehicle other, 
	int at_time) {

	Prediction check1 = state_at(at_time);
	Prediction check2 = other.state_at(at_time);

	return (check1.lane == check2.lane) && (abs(check1.s - check2.s) <= 1);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, 
	int timesteps) {
	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps + 1; t++) {
		if (collides_with(other, t)) {
			collider_temp.collision = true;
			collider_temp.time = t;
			return collider_temp;
		}
	}

	return collider_temp;
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

	double delta_v_til_target = max_speed - v;
	double max_acc = min(max_acceleration, delta_v_til_target);
	vector<Prediction> in_front;

	for (auto val: vehicle_predictions) {
		vector<Prediction> prediction = val.second;
		Prediction first_Prediction = prediction[0];
		
		if (first_Prediction.lane == _lane && first_Prediction.s > s) {
			in_front.push_back(first_Prediction);
		}
	}
  
	if (in_front.size() > 0) {
		double min_s = 9999999.0;

		for (int i = 0; i < in_front.size(); i++) {
			if (in_front[i].s < min_s)
				min_s = in_front[i].s;
		}

		double available_room = (min_s - s );
		//cout << " room:" << available_room << " " << " min_s:" << min_s << " ";		
		if (available_room < preferred_buffer) {
			max_acc = -max_acceleration;
			return max_acc;
		}

		max_acc = min(max_acc, available_room/preferred_buffer);
	}

	return max_acc;
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

			if (aa > max_acceleration)
				aa = max_acceleration;

			if (aa < -max_acceleration)
				aa = -max_acceleration;

			a = aa;
			return;
		}
	}
	a = _max_accel_for_lane(_lane, ego_predictions, vehicle_predictions);
}