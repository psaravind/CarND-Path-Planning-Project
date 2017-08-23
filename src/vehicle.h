#ifndef VEHICLE_H
#define VEHICLE_H
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include "cost.h"
#include "snapshot.h"

using namespace std;

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

	int preferred_buffer = 20;

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
		double a = 0);

	virtual ~Vehicle();

	void update(int lane, 
		double s, 
		double v, 
		double a);

	void update_state(map<int, 
		vector <vector<double>>> predictions);

	vector<Snapshot> trajectories_for_state(string state, 
		map<int, vector <vector<double>>> predictions, 
		int horizon = 5);

	void configure(double _max_speed,
		int _num_lanes,
		double _max_acceleration,
		double _min_car_distance);

	string display();

	void increment(int dt);
	void ego_increment(int dt);	

	vector<double> state_at(int t);

	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(map<int, vector<vector<double>>> predictions);

	void realize_constant_speed();

	double _max_accel_for_lane(map<int, vector<vector<double>>> predictions);

	void realize_keep_lane(map<int, vector< vector<double>>> predictions);

	void realize_lane_change(map<int, vector<vector<double>>> predictions, 
		string direction);

	void realize_prep_lane_change(map<int, vector<vector<double>>> predictions, 
		string direction);

	vector<vector<double>> generate_predictions(int horizon = 5);
};
#endif /* VEHICLE_H */