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

	int preferred_buffer = 1;

	int lane;
	int lanes_available;

	double s;
	double v;
	double a;
	double target_speed;
	double max_acceleration;

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

	void configure(double car_max_vel,
		int num_lanes,
		double max_accel);

	string display();

	void increment(int dt, bool skip_s);

	vector<double> state_at(int t);

	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(map<int, vector<vector<double>>> predictions);

	void realize_constant_speed();

	double _max_accel_for_lane(map<int, vector<vector<double>>> predictions, int lane, double s);

	void realize_keep_lane(map<int, vector< vector<double>>> predictions);

	void realize_lane_change(map<int, vector<vector<double>>> predictions, string direction);

	void realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction);

	vector<vector<double>> generate_predictions(int horizon = 5);
};
#endif /* VEHICLE_H */