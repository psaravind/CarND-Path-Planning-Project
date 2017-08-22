#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <algorithm>
#include "snapshot.h"
#include "cost.h"
#include "helper.h"

Vehicle::Vehicle(int _lane, 
	double _s, 
	double _v, 
	double _a) {
    
	lane = _lane;
	s = _s;
	v = _v;
	a = _a;
	
	state = "CS";
	max_acceleration = -1;

	cost = Cost();
}

Vehicle::~Vehicle() {}

void Vehicle::update(int _lane, 
	double _s, 
	double _v, 
	double _a) {
  
	lane = _lane;
	s = _s;
	v = _v;
	a = _a;
}

void Vehicle::configure(double car_max_vel,
	int num_lanes,
	double max_accel) {

	target_speed = car_max_vel;
	lanes_available = num_lanes;
	max_acceleration = max_accel;
}

vector<vector<double>> Vehicle::generate_predictions(int horizon) {
	vector<vector<double>> predictions;

	for (int i = 0; i < horizon; i++) {
		vector<double> check = state_at(i);
		vector<double> prediction = {check[0], check[1]};
		predictions.push_back(prediction);
	}

	return predictions;
}

vector<double> Vehicle::state_at(int t) {
	
	double dt = double(t) * 0.02;
	double _s = s + v * dt + a * dt * dt / 2;
	double _v = max(0.0, v + a * dt);

	return {double(lane), _s, _v, a};
}

void Vehicle::update_state(map<int, vector <vector<double>>> predictions) {

	vector<string> states = {"KL", "PLCR", "LCR", "PLCL", "LCL"};

	if (state.compare("KL") == 0) {
		if (lane == 0)
			states = {"KL", "PLCR"};
		else if (lane == lanes_available - 1)
			states = {"KL", "PLCL"};
		else
			states = {"KL", "PLCL", "PLCR"};
	}

	if (state.compare("PLCL") == 0)
		states = {"KL", "PLCL", "LCL"};

	if (state.compare("PLCR") == 0)
		states = {"KL", "PLCR", "LCR"};

	if (state.compare("LCR") == 0)
		states = {"KL"};//, "LCR"};

	if (state.compare("LCL") == 0)
		states = {"KL"};//, "LCL"};

	map<string, double> new_costs;
	for (auto _state : states) {
		//cout << "state:" << _state;
		vector<Snapshot> trajectories = trajectories_for_state(_state, predictions, 50);
		new_costs[_state] = cost.calculate_cost(*this, trajectories, predictions);
	}

	for(auto val: new_costs)
		cout << val.first << " " << double(val.second) << " ";
	
	double max = std::numeric_limits<double>::infinity();;
	for (auto val: new_costs) {
		if (val.second < max) {
			max = val.second;
			state = val.first;
		}
	}
	
	//pair<string, double> min = *min_element(new_costs.begin(), new_costs.end());
	//state = min.first;
	cout << "min state:" << state << " s:" << s;
}

vector<Snapshot> Vehicle::trajectories_for_state(string _state, 
	map<int, vector<vector<double>>> predictions, 
	int horizon) {

	Snapshot snapshot = Snapshot(lane, s, v, a, state);
	vector<Snapshot> trajectories;

	state = _state;
	trajectories.insert(trajectories.end(), snapshot);

	for (int i = 0; i < horizon; i++) {
		//lane = snapshot.lane;
		//s = snapshot.s;
		//v = snapshot.v;
		//a = snapshot.a;
		
		//state = _state;
		
		realize_state(predictions);
		ego_increment(1);

		trajectories.insert(trajectories.end(), Snapshot(lane, s, v, a, state));
	}

	lane = snapshot.lane;
	s = snapshot.s;
	v = snapshot.v;
	a = snapshot.a;
	state = snapshot.state;
	
	return trajectories;
}

/*
vector<Snapshot> Vehicle::trajectories_for_state(string _state, 
	map<int, vector<vector<double>>> predictions, 
	int horizon) {
	Snapshot snapshot = Snapshot(lane, s, v, a, state);
	vector<Snapshot> trajectories;

	state = _state;
	trajectories.insert(trajectories.end(), snapshot);
	realize_state(predictions);
	
	for (int i = 0; i < horizon; i++) {
		ego_increment(1);
	}
	trajectories.insert(trajectories.end(), Snapshot(lane, s, v, a, state));
	
	map<int, vector <vector<double>>>::iterator pr = predictions.begin();
	while (pr != predictions.end()) {
		int prediction_id = pr->first;
		vector<vector<double>> prediction = pr->second;
		prediction.erase(prediction.begin());
		predictions[prediction_id] = prediction;
		pr++;
	}
	lane = snapshot.lane;
	s = snapshot.s;
	v = snapshot.v;
	a = snapshot.a;
	state = snapshot.state;
	cout << "trajectories:size:" << trajectories.size() << endl;
	return trajectories;
}*/

void Vehicle::increment(int dt) {
	double ddt = double(dt) * 0.02;
  
	v += a * ddt;
	
	v = max(0.0, v);
	v = min(target_speed, v);
}

void Vehicle::ego_increment(int dt) {
	double ddt = double(dt) * 0.02;

	s += v * ddt;  
	//v += a * ddt;
	v += a * ddt * 50;

	v = max(0.0, v);
	v = min(target_speed, v);
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	vector<double> check1 = state_at(at_time);
	vector<double> check2 = other.state_at(at_time);

	return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= 1);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
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

void Vehicle::realize_state(map<int, vector<vector<double>>> predictions) {

	if (state.compare("CS") == 0)
		realize_constant_speed();
	else if (state.compare("KL") == 0)
		realize_keep_lane(predictions);
	else if (state.compare("LCL") == 0)
		realize_lane_change(predictions, "L");
	else if (state.compare("LCR") == 0)
		realize_lane_change(predictions, "R");
	else if (state.compare("PLCL") == 0)
		realize_prep_lane_change(predictions, "L");
	else if (state.compare("PLCR") == 0)
		realize_prep_lane_change(predictions, "R");
}

void Vehicle::realize_constant_speed() {
	a = 0;
}

void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
	a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, 
	string direction) {
	double delta = 1;

	if (direction.compare("L") == 0)
		delta = -1;

	lane += delta;

	lane = max(0, min(lanes_available, lane));

	a = _max_accel_for_lane(predictions, lane, s);
}

double Vehicle::_max_accel_for_lane(map<int, vector<vector<double>>> predictions, 
	int lane, 
	double ss) {

	double delta_v_til_target = target_speed - v;
	double max_acc = min(max_acceleration, delta_v_til_target);
	vector<vector<vector<double>>> in_front;

	for (auto val: predictions) {
		vector<vector<double>> vv = val.second;

		if ((vv[0][0] == lane) && (vv[0][1] > ss)) {
			in_front.push_back(vv);
		}
	}
  
	if (in_front.size() > 0) {
		double min_s = 200;

		vector<vector<double>> leading;

		for (int i = 0; i < in_front.size(); i++) {
			if ((in_front[i][0][1] - ss) < min_s) {
				min_s = (in_front[i][0][1] - ss);
				leading = in_front[i];
			}
		}

		double next_pos = leading.size() > 1 ? leading[1][1] : min_s;
		double my_next = ss + v * .02; // TODO CHECK
		double separation_next = next_pos - my_next;
		double available_room = separation_next - preferred_buffer;

		max_acc = min(max_acc, available_room);
		//cout << "in_front.size:" << in_front.size() << " separation_next:" << separation_next << "\n";
	}

	return max_acc;
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double>>> predictions, 
	string direction) {

	int delta = 1;
	if (direction.compare("R") == 0)  //  NOTE WAS L
		delta = -1;

	int _lane = lane + delta;

	vector<vector<vector<double>>> at_behind;
	for (auto val: predictions) {
		vector<vector<double>> v = val.second;
		if ((v[0][0] == _lane) && (v[0][1] <= s))
			at_behind.push_back(v);
	}

	if (at_behind.size() > 0) {
		int max_s = -1000;

		vector<vector<double>> nearest_behind;

		for (int i = 0; i < at_behind.size(); i++) {
			if ((at_behind[i][0][1]) > max_s) {
				max_s = at_behind[i][0][1];
				nearest_behind = at_behind[i];
			}
		}

		double target_vel;
		if (nearest_behind.size() > 1)
			target_vel = nearest_behind[1][1] - nearest_behind[0][1];
		else
			target_vel = 0;

		double delta_v = v - target_vel;
		double delta_s = s - nearest_behind[0][1];

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
		} else {
			double my_min_acc = max(-max_acceleration, -delta_s);

			a = my_min_acc;
		}
	}
}