#ifndef COST_H
#define COST_H
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "snapshot.h"
#include "helper.h"

struct TrajectoryData {
	int proposed_lane;
	double avg_speed;
	double max_acceleration;
	bool c_front;
	double c_front_at;
	bool c_back;
	double c_back_at;
};

// priority levels for costs
const double COLLISION  = pow(10.0, 6);
const double DANGER     = pow(10.0, 5);
const double EFFICIENCY = pow(10.0, 3);

const double DESIRED_BUFFER = 20;

class Vehicle;

class Cost {
	public:
		Cost();

		virtual ~Cost();

		double calculate_cost(const Vehicle &vehicle, 
			string state,
			int num_lanes,
			vector<Snapshot> trajectories, 
			vector<Prediction> ego_predictions,
			map<int, vector<Prediction>> vehicle_predictions);

		TrajectoryData get_helper_data(Vehicle ego,
			vector<Prediction> ego_predictions,
			string state,
			int num_lanes,
			vector<Snapshot> trajectories, 
			map<int, vector<Prediction>> predictions);

		map<int, vector<Prediction>> filter_predictions_by_lane(map<int, vector<Prediction>> predictions, 
			int lane);

		bool check_collision(Snapshot snapshot, 
			double s_previous, 
			double s_now);

		double inefficiency_cost(Vehicle vehicle, 
			TrajectoryData data);

		double collision_cost(Vehicle vehicle, 
			TrajectoryData data);

		double buffer_cost(Vehicle vehicle,  
			TrajectoryData data);
		double change_lane_cost(Vehicle vehicle,
			TrajectoryData data);
};
#endif /* COST_H */