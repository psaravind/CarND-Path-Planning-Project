#ifndef COST_H
#define COST_H
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "snapshot.h"

using namespace std;

struct TrajectoryData {
	int proposed_lane;
	double avg_speed;
	double max_acceleration;
	double closest_approach;
	bool collides;
	double collides_at;
};

// priority levels for costs
const double COLLISION  = pow(10.0, 6);
const double DANGER     = pow(10.0, 5);
const double EFFICIENCY = pow(10.0, 2);

const int PLANNING_HORIZON = 20;
const double DESIRED_BUFFER = 2;//10;

class Vehicle;

class Cost {
	public:
		Cost();

		virtual ~Cost();

		double calculate_cost(const Vehicle &vehicle, 
			vector<Snapshot> trajectories, 
			map<int, vector<vector<double>>> predictions);

		TrajectoryData get_helper_data(Vehicle vehicle, 
			vector<Snapshot> trajectories, 
			map<int, vector<vector<double>>> predictions);

		map<int, vector<vector<double>>> filter_predictions_by_lane(map<int, vector<vector<double>>> predictions, 
			int lane);

		bool check_collision(Snapshot snapshot, 
			double s_previous, 
			double s_now);

		double inefficiency_cost(Vehicle vehicle, 
			vector<Snapshot> trajectories, 
			map<int, vector<vector<double>>> predictions, 
			TrajectoryData data);

		double collision_cost(Vehicle vehicle, 
			vector<Snapshot> trajectories, 
			map<int, vector<vector<double>>> predictions, 
			TrajectoryData data);

		double buffer_cost(Vehicle vehicle, 
			vector<Snapshot> trajectories, 
			map<int, vector<vector<double>>> predictions, 
			TrajectoryData data);
};
#endif /* COST_H */