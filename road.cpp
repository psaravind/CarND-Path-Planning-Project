#include "road.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include "vehicle.h"
#include "helper.h"

Road::Road(int num_lanes, 
	int start_lane, 
	double s,
	double v,
	double a,
	double max_speed,
	double max_acceleration,
	double min_car_distance):ego(start_lane, 
		s, 
		v, 
		a, 
		max_speed, 
		num_lanes, 
		max_acceleration, 
		min_car_distance, 
		"KL"){}

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

		Vehicle vehicle = Vehicle(sensor_lane, sensor_s, sensor_speed, "KL");
		vehicles.insert(std::pair<int, Vehicle>(sensor_id, vehicle));
	} 
}

void Road::advance(double car_s,
	double car_speed) {
	
	map<int, vector<Prediction>> vehicle_predictions;
	vector<Prediction> ego_predictions;
	
	ego.s = car_s;
	ego.v = car_speed;
	
	cout << "before lane:" << ego.lane << " a:" << ego.a << " v:" << ego.v << endl;

	ego_predictions = ego.generate_predictions(50);

	for (auto val : vehicles)
		vehicle_predictions[val.first] = val.second.generate_predictions(50);

	ego.update_state(ego_predictions, vehicle_predictions);
	ego.realize_state(ego_predictions, vehicle_predictions);

	ego.advance(1);
	cout << "after lane:" << ego.lane << " a:" << ego.a << " v:" << ego.v << endl << endl;
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