#ifndef ROAD_H
#define ROAD_H

#include "vehicle.h"

using namespace std;

const int LANE_WIDTH = 4;
const int LANE_CENTR = LANE_WIDTH / 2;

class Road {
	public:
		Vehicle ego;
		int num_lanes;
		map<int, Vehicle> vehicles;

		Road(int _num_lanes, int lane_num, 
			double s, 
			double car_max_vel,
			int num_lanes,
			double max_accel);
		~Road();

		void populate_traffic(vector<vector<double>> sensor_fusion);

		void advance(double car_s);
		int getLane(double d);		
};
#endif /* ROAD_H */