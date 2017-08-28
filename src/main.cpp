#include <uWS/uWS.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "pathplanner.h"
#include "json.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

char* getCmdOption(char **begin, char** end, const std::string& option) {
	char **itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
		return *itr;
	
	return 0;
}

bool cmdOptionExists(char **begin, char** end, const std::string& option) {
	return std::find(begin, end, option) != end;
}

int main(int argc, char* argv[]) {
	uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	if (cmdOptionExists(argv, argv + argc, "-b")) {
		map_file_ = "../data/highway_map_bosch1.csv";
	}

	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
	bool debug = false;
	double max_speed = 49.75;
	double min_car_distance = 120.0;
	int num_lanes = 3;
	int start_lane = 1;
	double s;
	double v = 5;
	double a = 1.5;
	double max_acceleration = 1.5;

	if (cmdOptionExists(argv, argv + argc, "-d")) {
		debug = true;
	}

	if (cmdOptionExists(argv, argv + argc, "-s")) {
		max_speed = atof(getCmdOption(argv, argv + argc, "-s"));
	}

	if (cmdOptionExists(argv, argv + argc, "-a")) {
		a = atof(getCmdOption(argv, argv + argc, "-a"));
		max_acceleration = a;
	}

	if (cmdOptionExists(argv, argv + argc, "-m")) {
		min_car_distance = atof(getCmdOption(argv, argv + argc, "-m"));
	}

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;

		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
    
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	PathPlanner planner = PathPlanner(num_lanes,
		start_lane,
		s,
		v,
		a,
		max_speed,
		min_car_distance,
		max_acceleration,
		map_waypoints_x, 
		map_waypoints_y, 
		map_waypoints_s, 
		map_waypoints_dx, 
		map_waypoints_dy);

	h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		// auto sdata = string(data).substr(0, length);
		// cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {
			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					auto sensor_fusion = j[1]["sensor_fusion"];

					vector<double> car_data = {
						j[1]["x"],
						j[1]["y"],
						j[1]["s"],
						j[1]["d"],
						j[1]["yaw"],
						j[1]["speed"]
					};
					
					vector<vector<double>> path_data = {
						j[1]["previous_path_x"],
						j[1]["previous_path_y"]
					};

					vector<double> end_path_sd = {
						j[1]["end_path_s"],
						j[1]["end_path_d"]
					};

					vector<vector<double>> next_vals = planner.GeneratePath(car_data, 
						sensor_fusion, 
						path_data, 
						end_path_sd);
					
					json msgJson;

					msgJson["next_x"] = next_vals[0];
					msgJson["next_y"] = next_vals[1];

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
						size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
		  res->end(s.data(), s.length());
		} else {
		  // i guess this should be done more gracefully?
		  res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}