#ifndef VEHICLE_H
#define VEHICLE_H
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include "cost.h"
#include "snapshot.h"
#include "spline.h"

class Vehicle {
	private:
		Cost cost;

	public:
		struct collider {
			bool collision;
			int time;
		};

	struct snapshot {
		int lane;
		double s;
		double v;
		double a;
		string state;
	};

	int preferred_buffer = 38;

	int lane;
	int num_lanes;

	double s;
	double v;
	double a;
	double max_speed;
	double max_acceleration;
	double min_car_distance;

	string state;

	Vehicle(int lane, 
		double s, 
		double v, 
		string _state);

	Vehicle(int _lane, 
		double _s, 
		double _v, 
		double _a,
		double _max_speed,
		int _num_lanes,
		double _max_acceleration,
		double _min_car_distance,
		string _state);

	virtual ~Vehicle();

	void update_state(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> predictions);

	vector<Snapshot> trajectories_for_state(string state, 
		vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions, 
		int horizon = 5);

	void advance(int dt);

	Prediction state_at(int t);

	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions);

	void realize_constant_speed();

	double _max_accel_for_lane(int lane,
		vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions);

	void realize_keep_lane(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions);

	void realize_lane_change(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions, 
		string direction);

	void realize_prep_lane_change(vector<Prediction> ego_predictions,
		map<int, vector<Prediction>> vehicle_predictions, 
		string direction);

	vector<Prediction> generate_predictions(int horizon = 5);
};
#endif /* VEHICLE_H */