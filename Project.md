
# Path Planning Project

## Goals for the project
This project meets following goals as specified in the [Path Planning Project Github Readme file](https://github.com/udacity/CarND-Path-Planning-Project).

1. Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
2. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, other cars will try to change lanes as well.
3. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
4. The car should be able to make one complete loop around the 6946m highway. 
5. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
6. Car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Data and tools provided for the project.
1. Car's localization and sensor fusion data.
2. Sparse map list of waypoints around the highway.
3. Term3 Simulator that provides self driving car's localization data, previous path data and previous path's end s and d values.

## Path Planning Rubric

### 1. Compilation

#### 1.1 Code compiles correctly
CMakeLists.txt file has been modified to include following files along with the 'main.cpp'
1. road.cpp 
2. cost.cpp 
3. vehicle.cpp 
4. snapshot.cpp 
5. pathplanner.cpp 
6. helper.cpp

Third party libraries
1. Eigen-3.3 - Eigen C++ template library for linear algebra
2. spline.h - Cubic Spline interpolation in C++
3. json.hpp - JSON for modern C++

#### 1.2 Build Instructions

Compile: 
```shell 
$cmake .. && make
```

#### 1.3 Running Path Planning module

Run the Term3 simulator and run the compiled path planning module.

Run it:
```shell
$./path_planning
```

Path planning module lisents on port 4567 to communicate with the simulator. 

### 2. Valid Trajectories

#### 2.1 The car is able to drive at least 4.32 miles without incident

The self driving car is able to drive several laps around the track without any incident, I let it run for 3 laps without incident, self driving car is very conservative and does not cut other cars to change lanes.  During the 3 laps, current and best miles were the same, there were no acceleration, jerk, speed, collision or driving outside the lanes.  I did not see any incident listed on the simulator.

#### 2.2 The car drives according to the speed limit

Maximum speed limit that the self driving car could reach is 49.75mph, this value was chosen to keep the car below the speed limit.  Car was travelling 60% of the time around this maximum speed limit, during other them it was below the speed limit due to obstructed and congested traffic.  

#### 2.3 Max Acceleration and Jerk are not Exceeded

The self driving car never exceeded the total acceleration of 10 m/s^2 and a jerk of 10 m/s^3 during the 3 laps, this was achieved due to various optimization(described below in reflection) techniques employed to tune the acceleration parameters.

#### 2.4 Car does not have collisions

Self driving car maintains conservative driving and avoids collision at all costs.  It maintains enough driving distance with car in front and while change lanes by providing enough distance between cars in front and back.  Several optimization(described below in reflection) techniques were employed to avoid collisions.

#### 2.5 The car stays in its lane, except for the time between changing lanes

The self driving car stays in the lane by sticking to the center of the lane.  When the time comes to change lanes, transistion is done very smoothly with constant acceleration and reaches the target lane without coliding with other cars.  Car does not spend more 3 second length outside the lane during changing lanes.

#### 2.6 The car is able to change lanes

Self driving car smoothly changes lane when its behind a slower car, this lane change happens when the adjacent lane is clear and its safe to make the lane change without colliding with other cars.

#### 3. Reflection

Following are some of the methods and tuning steps I employed in the Path Planning project.  Project is based on code walkthru video and the python solutions provided for Lesson 4: Behavior Planning module, python code was converted to C++ classes and significant changes were made to achieve the goals for the project.  

#### 3.1 Driver module

main.cpp is the main driver for the project, this file was changed to call other classes to handle path planning, road, vehicle and cost functions.  This file has hard coded default values which could be changed in command line options.  Way points file './data/highway_map.csv' is read and stored as vectors or double.  These values along with following default values are passed to the path planner constructor.  

```C++
	double max_speed = 49.75;             // maximum speed for self driving car
	double min_car_distance = 120.0;      // minimum car distance to follow
	int num_lanes = 3;                    // number of lanes in the road
	int start_lane = 1;                   // starting lane for the self driving car
	double s;                             // s value of the self driving car
	double v = 5;                         // velocity of the self driving car
	double a = 1.6;                       // acceleration value for the self driving car
	double max_acceleration = 1.6;        // maximum acceleration for the self driving car
  ```

After this step, main module listens on the websocket for message events.  This websocket message provides both message and event, the json message provides data about the self driving car, previous path details and end path details.  These values are passed to the path planner 'GeneratePath()' method to get the path that the self driving car needs to follow.

#### 3.2 Path Planning

Path planning class is called in the main class with way points data along with default values for the road and self driving car.  Main logic for the self driving car is executed by the GeneratePath() method, this method takes in following parameters:
1. Self driving car's localization data, car's x, y, s, d, yaw and speed
2. Previous path data given to the simulator, this provides the path that has been processed since last time.
3. Sensor Fusion Data, a list of all other car's attributes on the same side of the road.

Using these parameters, is uses the road, vehicle and cost classes to get **reference velocity** and **lane** the self driving car need to follow on the road.  These parameters as used to generate path for the self driving car to follow.  The code for generating the path is based on the walk-through provided by Aaron Brown and David Silver in the [path planning overview](https://www.youtube.com/watch?v=7sI3VHFPP0w).

The new path for the self driving car starts by using 2 points from previous path data from the simulator.  Using this data, previous position of the car is calculated, these points describe a path tangent to the car or to the previous path's end point which is used to generate new path.  


```c++
	if (prev_size < 2) {
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	} else {
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}
```

3 (x, y) coordinates for 30, 60 and 90 meters ahead in the target lane are calculated using the map's waypoints.   Using these 5 (x, y) points provides a path that the self driving car needs to follow at that very movement.  

```c++
	vector<double> next_wp0 = getXY(car_s + 30, 
		(2 + 4 * car_lane), 
		map_waypoints_s, 
		map_waypoints_x, 
		map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + 60, 
		(2 + 4 * car_lane), 
		map_waypoints_s, 
		map_waypoints_x, 
		map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + 90, 
		(2 + 4 * car_lane), 
		map_waypoints_s, 
		map_waypoints_x, 
		map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);
```

These points are rotated to self driving car's local coordinate system.  
```c++
	for (int i = 0; i < ptsx.size(); i++) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}
```

In order to product smooth spacing of points without excessive acceleration and jerk, target distance of 30 meters is used to generate a spline spacing along the x values. 

```c++
	tk::spline s;
			
	s.set_points(ptsx, ptsy);

	vector<double> next_x_vals;
	vector<double> next_y_vals;
	
	for (int i = 0; i < previous_path_x.size(); i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

	double x_add_on = 0;  
```

Rest of the path points are calculated and then the coordinates are rotated and transformed to global coordinates.  These values are pushed into path vector which are returned to simulator.

```c++
	for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
		double N = (target_dist / (.02 * car_ref_vel / 2.24));
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	return {next_x_vals, next_y_vals};
}
```

#### 3.3 Populating vehicles and advancing self driving car in 'road' class

Road class simulates a road on the highway, storing the self driving car details along with the car details from the sensor fusion data.  Road class updates the self driving car's parameters, generates predictions, updates state, realizes the state based on predicitons and advances the car.

```c++
void Road::advance(double car_s,
	double car_speed) {
	
	map<int, vector<Prediction>> vehicle_predictions;
	vector<Prediction> ego_predictions;
	
	ego.s = car_s;
	ego.v = car_speed;

	ego_predictions = ego.generate_predictions(50);

	for (auto val : vehicles)
		vehicle_predictions[val.first] = val.second.generate_predictions(50);

	ego.update_state(ego_predictions, vehicle_predictions);
	ego.realize_state(ego_predictions, vehicle_predictions);

	ego.advance(1);
}
```

#### 3.4 State management in vehicle class

Vehicle class implements the state transition explained in the class with following states:
1. 'CS': constant speed, used by sensor fusion cars
2. 'KL': Keep Lane, cruise along current lane
3. 'PLCL': prepare for lane change left
4. 'PLCR': prepare for lane change right
5. 'LCL': lane change to left
6. "LCR': lane change to right

Following diagram shows the finite state machine implemented in the vehicle class.
![FSM](/image/fsm.jpg)

Following code implements the above state transition diagram

```c++
	if (state.compare("KL") == 0) {
		if (lane == 0)
			next_states = {"KL", "PLCR"};
		else if (lane == num_lanes - 1)
			next_states = {"KL", "PLCL"};
		else
			next_states = {"KL", "PLCL", "PLCR"};
	}

	if (state.compare("PLCL") == 0)
		next_states = {"KL", "PLCL", "LCL"};

	if (state.compare("PLCR") == 0)
		next_states = {"KL", "PLCR", "LCR"};

	if (state.compare("LCR") == 0)
		next_states = {"KL", "PLCR"};
	if (state.compare("LCL") == 0)
		next_states = {"KL", "PLCL"};
```

Vehicle class implements _max_accel_for lane() and realize_prep_lan_change() methods that provides the stability of the vehicle.  Maximum acceleration for the lane throttles the acceleration based on vehicle speed, also the preffered buffer to vehicle in front is also calculated on vehicles speed.  These two dynamic parameters atains the primary goal for the project, stable vehicle acceleration and reduce jerk.

#### 3.7 Cost Calculations in cost class

Cost class implements the cost function to determine the best state to transition:

In efficiency cost: calculates cost for not maining maximum speed for self driving car.
Collision cost: calculates cost for self driving car colliding with car in front based on location of car in front.
Buffer cost: calculates cost for maining safer buffer in front of the car, this buffer is calculated on speed of the car.
Change Lane cost:calculates cost for safely changing lane based on distance of car behind in adjacent lane and speed of the car.

```c++
	TrajectoryData trajectory_data = get_helper_data(ego,
		ego_predictions, 
		state,
		num_lanes,
		trajectories, 
		vehicle_predictions);
		
	double cost = 0.0;
	
	cost += inefficiency_cost(ego, trajectory_data);
	cost += collision_cost(ego, trajectory_data);
	cost += buffer_cost(ego, trajectory_data);
	if (state.compare("KL") != 0)
		cost += change_lane_cost(ego, trajectory_data);
```

### Reflection

Various improvements could be made like removing some of the dead code that are not really used in the final project.  Prepare to lane changes are not reducing the acceleration to match the car speed on the left or right, this reduction in speed could help transitioning to adjacent lane.  Currently it just waits for a open slot on adjacent lane, this could take a long time, so the car just reduces speed and stays in the lane.


