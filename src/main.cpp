#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

void checkIfCanTurnBecauseOfCarsInFront(int lane, double distance_threshold, float d, double distance_to_car,
                                        bool &canChangeLeft, bool &canChangeRight);

void findClosestCarsBehindWhichCanCreateAnIssueForTheTurn(int car_id, int lane, double distance_threshold, float d,
                                                          double distance_to_car,
                                                          double &minimal_distance_to_the_car_behind_on_the_left_side,
                                                          int &index_of_closest_car_behind_left,
                                                          double &minimal_distance_to_the_car_behind_on_the_right_side,
                                                          int &index_of_closest_car_behind_right);

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int lane = 1;
  double ref_vel = 0.1;

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    // Define constans
    const double distance_for_a_next_waypoint = 30;
    const double target_speed = 49.5;
    const double distance_threshold = 30;
    const double dt = 0.02;
    const double max_acc = 0.224;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Size of previous path for reuse
          int prev_size = previous_path_x.size();

          // Define variables which will define possible behaviours
          bool canChangeLeft = true;
          bool canChangeRight = true;
          bool carAhead = false;

          // This variable has same values as carAhead, introduced to have more semantic in the context
          bool shouldSlowDown = false;

          // Try to use end if existed path as current car position for trajectory prediction
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // Variables to find the closest cars in adjacent lines, to ensure that switching the line is safe
          double minimal_distance_to_the_car_behind_on_the_left_side = 100000;
          int index_of_closest_car_behind_left = -1;

          double minimal_distance_to_the_car_behind_on_the_right_side = 100000;
          int index_of_closest_car_behind_right = -1;

          // disallow left turn if we are already on the most left lane
          if (lane <= 0) {
            canChangeLeft = false;
          }

          // disallow right turn if we are already on the most left lane
          if (lane >= 2) {
            canChangeRight = false;
          }

          for (auto & sf : sensor_fusion) {
            // get d coordinate of the car
            float d = sf[6];

            // check if car is on the our side of the highway
            if (d < 0 || d > 12) {
              continue;
            }

            double vx = sf[3];
            double vy = sf[4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sf[5];

            check_car_s += (double)prev_size * dt * check_speed;
            double distance_to_car = check_car_s - car_s;

            // looks for the cars in front in the future
            checkIfCanTurnBecauseOfCarsInFront(lane, distance_threshold, d, distance_to_car, canChangeLeft,
                                               canChangeRight);

            // check if there will be some cars behind with high speed and find closest
            findClosestCarsBehindWhichCanCreateAnIssueForTheTurn(sf[0], lane, distance_threshold, d, distance_to_car,
                                                                 minimal_distance_to_the_car_behind_on_the_left_side,
                                                                 index_of_closest_car_behind_left,
                                                                 minimal_distance_to_the_car_behind_on_the_right_side,
                                                                 index_of_closest_car_behind_right);
          }

          // check if changing the line is safe regarding the distance to the cars behind and their velocity
          for (auto & sf : sensor_fusion) {
            if (sf[0] == index_of_closest_car_behind_left ) {
              double vx = sf[3];
              double vy = sf[4];
              double check_speed = sqrt(vx*vx + vy*vy);

              canChangeLeft &= check_speed*1.3 < car_speed && abs(minimal_distance_to_the_car_behind_on_the_left_side) > distance_threshold*0.5; // use 0.5 fo safe turn
//              std::cout << "there is fast cars left " << index_of_closest_car_behind_left << " " << (check_speed < car_speed) << " check_speed " << check_speed << " car_speed " << car_speed << " dist " << minimal_distance_to_the_car_behind_on_the_left_side << "\n";
              continue;
            }

            if (sf[0] == index_of_closest_car_behind_right ) {
              double vx = sf[3];
              double vy = sf[4];
              double check_speed = sqrt(vx*vx + vy*vy);

              canChangeRight &= check_speed*1.3 < car_speed && abs(minimal_distance_to_the_car_behind_on_the_right_side) > distance_threshold*0.5; // use 0.5 fo safe turn
//              std::cout << "there is fast cars right " << index_of_closest_car_behind_right << " " << (check_speed < car_speed) << " check_speed " << check_speed << " car_speed " << car_speed << " dist " << minimal_distance_to_the_car_behind_on_the_right_side << "\n";
              continue;
            }
          }

          // use current sensor fusion data to prevent a collision with a car in front
          double car_measured_position = j[1]["s"];

          for (auto & sf : sensor_fusion) {
            float d = sf[6];

            double check_car_s = sf[5];

            double vx = sf[3];
            double vy = sf[4];
            double check_speed = sqrt(vx*vx + vy*vy);

            bool is_car_in_front = check_car_s > car_measured_position;
            double measured_distance_to_car = abs(check_car_s - car_measured_position);

            if (d < (4+4*lane) && d > (4*lane)) {
              if (is_car_in_front && measured_distance_to_car < distance_threshold) {
                  shouldSlowDown |= true;
                  carAhead |= true;
              }
            }

            if (d < (4+4*(lane+1)) && d > (4*(lane+1))) {
              if (!is_car_in_front && measured_distance_to_car < distance_threshold*3 && check_speed > car_speed*1.3) {
                canChangeRight = false;
              }
            }

            if (d < (4+4*(lane-1)) && d > (4*(lane-1))) {
              if (!is_car_in_front && measured_distance_to_car < distance_threshold*3 && check_speed > car_speed*1.3) {
                canChangeLeft = false;
              }
            }
          }


          // make decision about changing the lane or decreasing the speed
          // change the line if it makes sense
          if (carAhead && canChangeLeft) {
            lane--;
          } else if(carAhead && !canChangeLeft && canChangeRight) {
            lane++;
          }

          // change speed to avoid collision but tries to get back to the target speed if possible
          if(shouldSlowDown) {
            ref_vel -= max_acc;
          } else if(!shouldSlowDown && ref_vel < target_speed) {
            ref_vel += max_acc;
          }
// Debug
//          std::cout << "shouldSlowDown " << shouldSlowDown
//          << " canChangeLeft " << (canChangeLeft)
//          << " canChangeRight " << (canChangeRight)
//          << " lane " << lane
//          <<"\n";

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

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

void findClosestCarsBehindWhichCanCreateAnIssueForTheTurn(int car_id, int lane, const double distance_threshold, float d,
                                                          double distance_to_car,
                                                          double &minimal_distance_to_the_car_behind_on_the_left_side,
                                                          int &index_of_closest_car_behind_left,
                                                          double &minimal_distance_to_the_car_behind_on_the_right_side,
                                                          int &index_of_closest_car_behind_right) {
  if (distance_to_car < 0 && abs(distance_to_car) < distance_threshold * 0.5) { // use 0.5 for more aggressive driving
    // left lane
    if (d < (4+4*(lane-1)) && d > (4*(lane-1))) {
      if(minimal_distance_to_the_car_behind_on_the_left_side > distance_to_car) {
        minimal_distance_to_the_car_behind_on_the_left_side = distance_to_car;
        index_of_closest_car_behind_left = car_id;
      }
    }

    // right lane
    if (d < (4+4*(lane+1)) && d > (4*(lane+1))) {
      if(minimal_distance_to_the_car_behind_on_the_right_side > distance_to_car) {
        minimal_distance_to_the_car_behind_on_the_right_side = distance_to_car;
        index_of_closest_car_behind_right = car_id;
      }
    }
  }
}

void checkIfCanTurnBecauseOfCarsInFront(int lane, const double distance_threshold, float d, double distance_to_car,
                                        bool &canChangeLeft, bool &canChangeRight) {
  if (0 < distance_to_car && distance_to_car < distance_threshold) {
    if (d < (4+4*(lane-1)) && d > (4*(lane-1))) {
      // Debug
      // std::cout << "can't change left because of prediction "<< distance_to_car << " " << distance_threshold << "\n";
      canChangeLeft &= false;
    }

    if (d < (4+4*(lane+1)) && d > (4*(lane+1))) {
      // Debug
      // std::cout << "can't change right because of prediction \n";
      canChangeRight &= false;
    }
  }
}
