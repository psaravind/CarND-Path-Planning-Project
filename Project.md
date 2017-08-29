
# Path Planning Project

This project meets following goals as specified in the Github Readme file.

1. Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
2. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, other cars will try to change lanes as well.
3. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
4. The car should be able to make one complete loop around the 6946m highway. 
5. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
6. Car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Data and tools provided for the project.
1.  Car's localization and sensor fusion data.
2.  Sparse map list of waypoints around the highway.
3.  Term3 Simulator that provides self driving car's localization data, previous path data and previous path's end s and d values.

### Path Planning Rubric

#### 1. Compilation

##### 1.1 The code compiles correctly
Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

#### 2. Valid Trajectories

##### 2.1 The car is able to drive at least 4.32 miles without incident
	
The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

##### 2.2 The car drives according to the speed limit
The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

##### 2.3 Max Acceleration and Jerk are not Exceeded
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3

##### 2.4 Car does not have collisions
The car must not come into contact with any of the other cars on the road

##### 2.5 The car stays in its lane, except for the time between changing lanes
The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

##### 2.6 The car is able to change lanes
The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

#### 3. Reflection

##### 3.1 There is a reflection on how to generate paths
The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".
