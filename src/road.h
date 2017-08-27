#ifndef ROAD_H
#define ROAD_H

#include "vehicle.h"

const int LANE_WIDTH = 4;

class Road {
	public:
		Vehicle ego;
		map<int, Vehicle> vehicles;

		Road(int num_lanes, 
			int start_lane, 
			double s, 
			double v,
			double a,
			double max_speed,
			double max_acceleration,
			double min_car_distance);
		~Road();

		void populate_traffic(vector<vector<double>> sensor_fusion);

		void advance(double car_s,
			double car_speed);
		int getLane(double d);
};

#endif /* ROAD_H */