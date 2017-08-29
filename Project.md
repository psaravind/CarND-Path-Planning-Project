
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

Compile: ```shell 
$cmake .. && make
```

#### 1.3 Running Path Planning module

Run the Term3 simulator and run the compiled path planning module.

Run it:```shell
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

Folowing 

```C++
	double max_speed = 49.75;             // maximum speed for self driving car
	double min_car_distance = 120.0;
	int num_lanes = 3;
	int start_lane = 1;
	double s;
	double v = 5;
	double a = 1.6;
	double max_acceleration = 1.6;
  ```

#### 3.2 Path Planning

#### 3.3 Driver module

#### 3.4 Populating vehicles in road.cpp

#### 3.5 Generating predictions

#### 3.6 State management in vehicle.cpp

#### 3.7 Cost Calculations in cost.cpp

#### 3.8 Generating Paths

### Improvement and next steps
