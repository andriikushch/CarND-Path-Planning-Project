# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
  
![road.gif](road.gif)

Link to [video](road.mp4)
 
### Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Model Documentation

#### Define constans and initial values for the vars
```c++
const double distance_for_a_next_waypoint = 30;
const double target_speed = 49.5;
const double distance_threshold = 30;
const double dt = 0.02;
const double max_acc = 0.224;
int lane = 1;
```
These constant will be used in implementation. 

#### Size of previous path for reuse
```
int prev_size = previous_path_x.size();
```

#### Define variables which will define possible behaviours
```
bool canChangeLeft = true;
bool canChangeRight = true;
bool carAhead = false;
bool shouldSlowDown = false;
```

#### Try to use end if existed path as current car position for trajectory prediction
```
if (prev_size > 0) {
	car_s = end_path_s;
}
```

#### Variables to find the closest cars in adjacent lines, to ensure that switching the line is safe
```
double minimal_distance_to_the_car_behind_on_the_left_side = 100000;
int index_of_closest_car_behind_left = -1;

double minimal_distance_to_the_car_behind_on_the_right_side = 100000;
int index_of_closest_car_behind_right = -1;
```

#### Disallow left turn if we are already on the most left lane
```      
if (lane <= 0) {
	canChangeLeft = false;
}
```

#### Disallow right turn if we are already on the most left lane
```
if (lane >= 2) {
	canChangeRight = false;
}
```
          
#### Predict cars position in the future using the trajectory
1. Looks for the cars in front in the future.
2. Check if changing the line is safe regarding the distance to the cars behind and their velocity
 
#### Use current sensor fusion data to prevent a collision with a car in front

If the situation on the road changed rapidly we want to react, by modifying

```
shouldSlowDown;
carAhead;
canChangeRight;
canChangeLeft;
```

#### Make decision about changing the lane or decreasing the speed
```
if (carAhead && canChangeLeft) {
    lane--;
} else if(carAhead && !canChangeLeft && canChangeRight) {
    lane++;
}
```

#### Change speed to avoid a collision but tries to get back to the target speed if possible
```
if(shouldSlowDown) {
	ref_vel -= max_acc;
} else if(!shouldSlowDown && ref_vel < target_speed) {
	ref_vel += max_acc;
}
```

### Build the trajectory
```c++
// points that will be used to build the spline
  vector<double> ptsx;
  vector<double> ptsy;

  // use sensor fusion data
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if there is no previous path, fake it by adding the augmented points using the known angle,
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // create a new point in Frenet coordinates, which will be used to generate smooth path with spline
  vector<double> next_wp0 = getXY(car_s + 1*distance_for_a_next_waypoint, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 2*distance_for_a_next_waypoint, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 3*distance_for_a_next_waypoint, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // transform points to local coordinates from map coordinates
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
    ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
  }

  // use spline to generate smooth path
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // this is real planner points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // reuse previous path points
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

  double x_add_on = 0;

  // number of intervals for the target distance, we need to properly distribute our trajectory points
  double N = (target_dist/(dt*ref_vel/2.24)); // 2.24 is koef for miles

  for (int i = 0; i <= 50 - prev_size; i++) {

    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // convert to map coordinates from local
    x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
```

--- 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```


---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


