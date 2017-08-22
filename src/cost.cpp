#include "cost.h"
#include <math.h>
#include <map>
#include <string>
#include "vehicle.h"
#include "snapshot.h"

Cost::Cost() {}
Cost::~Cost() {}

double Cost::calculate_cost(const Vehicle &vehicle, 
	vector<Snapshot> trajectories, 
	map<int, vector <vector<double>>> predictions) {
	TrajectoryData trajectory_data = get_helper_data(vehicle, trajectories, predictions);

	/*cout << " proposed_lane:" << trajectory_data.proposed_lane << 
		" avg_speed:" << trajectory_data.avg_speed <<
		" max_acceleration:" << trajectory_data.max_acceleration <<
		" closest_approach:" << trajectory_data.closest_approach <<
		" collides:" << trajectory_data.collides << 
		" collides_at:" << trajectory_data.collides_at << endl;*/
		
	double cost = 0.0;

	cost += inefficiency_cost(vehicle, trajectories, predictions, trajectory_data);
	cost += collision_cost(vehicle, trajectories, predictions, trajectory_data);
	cost += buffer_cost(vehicle, trajectories, predictions, trajectory_data);

	return cost;
}

TrajectoryData Cost::get_helper_data(Vehicle vehicle, 
	vector<Snapshot> trajectories, 
	map<int, vector <vector<double>>> predictions) {

	TrajectoryData trajectory_data;

	vector<Snapshot> t = trajectories;

	Snapshot current_snapshot = t[0];
	Snapshot first = t[1];
	Snapshot last = t.back();//[t.size() - 1];

	double dt = trajectories.size();

	trajectory_data.proposed_lane = first.lane;

	trajectory_data.avg_speed = ((last.s - current_snapshot.s) / dt) * 50; // TODO CHECK

	vector<double> accels;

	double closest_approach = 999999.0;
	bool collides = false;

	map<int, vector<vector<double>>> filtered = filter_predictions_by_lane(predictions, 
		trajectory_data.proposed_lane);

	trajectory_data.collides = false;
	trajectory_data.collides_at = 0;
	for (int i = 1; i < (PLANNING_HORIZON + 1); i++) {
		Snapshot snapshot = trajectories[i];

		accels.insert(accels.end(), snapshot.a);

		for (auto val: filtered) {
			int v_id = val.first;

			vector<vector<double>> v = val.second;
			vector<double> state = v[i];
			vector<double> last_state = v[i - 1];

			bool vehicle_collides = check_collision(snapshot, last_state[1], state[1]);
			if (vehicle_collides) {
				/*for(auto pred : predictions) {
					vector<vector<double>> values = pred.second;
					cout << "pred first:" << pred.first << " [0]:" << values[0].size() << " [1]:" << values[1].size() << endl; 
				}
				for(auto traj : trajectories)
					cout << "T<" << i << "> lane:" << traj.lane 
				         << " s:" << traj.s 
						 << " v:" << traj.v 
						 << " a:" << traj.a 
						 << " state:" << traj.state << endl;
				cout << "vehicle_collides:" << vehicle_collides << " at:" << i << " in lane:" << trajectory_data.proposed_lane << endl;*/
				trajectory_data.collides = true;
				trajectory_data.collides_at = double(i);
			}

			int dist = abs(state[1] - snapshot.s);
			if (dist < trajectory_data.closest_approach) 
				trajectory_data.closest_approach = dist;
		}
	}

	int num_accels = accels.size();
	trajectory_data.max_acceleration = 0;
	for (int i = 0; i < num_accels; i++) {
		if (abs(accels[i]) > trajectory_data.max_acceleration) 
			trajectory_data.max_acceleration = abs(accels[i]);
	}
	return trajectory_data;
}

map<int, vector<vector<double>>> Cost::filter_predictions_by_lane(map<int, vector <vector<double>>> predictions, 
	int lane) {
  
	map<int, vector<vector<double>>> filtered;

	for (auto val: predictions) {
		if (val.second[0][0] == lane && val.first != -1) {
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

double Cost::inefficiency_cost(Vehicle vehicle, 
	vector<Snapshot> trajectories, 
	map<int, vector<vector<double>>> predictions, 
	TrajectoryData data) {

	double speed = data.avg_speed;
	double target_speed = vehicle.target_speed;
	double diff = target_speed - speed;
	double pct = diff / target_speed;
	double multiplier = pct * pct;
  
	return multiplier * EFFICIENCY;
}

double Cost::collision_cost(Vehicle vehicle, 
	vector<Snapshot> trajectories, 
	map<int, vector<vector<double>>> predictions, 
	TrajectoryData data) {

	if (data.collides) {
		double time_til_collision = data.collides_at;
		double exponent = time_til_collision * time_til_collision;
		double multiplier = exp(-exponent);
		
		//cout << "Cost::collision_cost data.collides_at:" << data.collides_at << endl;
		return multiplier * COLLISION;
	}

	return 0;
}

double Cost::buffer_cost(Vehicle vehicle, 
	vector<Snapshot> trajectories, 
	map<int, vector<vector<double>>> predictions, 
	TrajectoryData data) {

	double closest = data.closest_approach;
	if (closest == 0) 
		return 10 * DANGER;

	double timesteps_away = closest / data.avg_speed;
	if (timesteps_away > DESIRED_BUFFER) 
		return 0;

	double multiplier = 1 - pow((timesteps_away / DESIRED_BUFFER), 2);

	return multiplier * DANGER;
}