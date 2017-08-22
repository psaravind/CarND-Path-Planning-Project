#include "road.h"
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include "vehicle.h"
#include "helper.h"

Road::Road(int _num_lanes, 
	int lane_num, 
	double s, 
	double car_max_vel,
	int num_lanes,
	double max_accel) : ego(lane_num, s, 5, 1) {
	num_lanes = _num_lanes;

	ego.configure(car_max_vel, num_lanes, max_accel);

	ego.state = "KL";
}

Road::~Road() {}

void Road::populate_traffic(vector<vector<double>> sensor_fusion) {

	vehicles.clear();
 
	for (int i = 0; i < sensor_fusion.size(); i++) {
		int sensor_id = int((sensor_fusion[i][0]));

		double sensor_vx = sensor_fusion[i][3];
		double sensor_vy = sensor_fusion[i][4];
		double sensor_s = sensor_fusion[i][5];
		double sensor_d = sensor_fusion[i][6];

		int sensor_lane = getLane(sensor_d);

		double sensor_speed = sqrt(sensor_vx * sensor_vx + sensor_vy * sensor_vy);

		Vehicle vehicle = Vehicle(sensor_lane, sensor_s, sensor_speed);
		vehicle.state = "KL";
		vehicles.insert(std::pair<int, Vehicle>(sensor_id, vehicle));
	} 
}

void Road::advance(double car_s) {
//cout << " a:" << ego.a << " v:" << ego.v << endl;
	map<int, vector<vector<double>>> predictions;
	ego.s = car_s;
	predictions[-1] = ego.generate_predictions(50);

	for (auto val : vehicles)
		predictions[val.first] = val.second.generate_predictions(50);

	ego.update_state(predictions);
	ego.realize_state(predictions);
	ego.ego_increment(1);
	cout << " a:" << ego.a << " v:" << ego.v << endl;
	//for (auto val : vehicles)// TODO: is this needed
	//	val.second.increment(1);// TODO: is this needed
}

int Road::getLane(double d) {
	if (d >= 0 && d < LANE_WIDTH) 
		return 0;
	if (d >= LANE_WIDTH && d < LANE_WIDTH * 2) 
		return 1;
	if (d >= LANE_WIDTH * 2 && d < LANE_WIDTH * 3) 
		return 2;

	return -1;
}