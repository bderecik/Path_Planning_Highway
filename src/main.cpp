#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
  // set the start lane
  int lane = 1;

  // reference velocity
  double ref_vel = 0.0; //mile per hour

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          /**
           * Defining a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();

          // collision avoidance! take surrounding cars' data from sensor fusion data
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          bool so_close = false;
          bool car_ahead_of = false;
          bool car_right_side = false;
          bool car_left_side = false;

          // find reference velocity ref_v to use
          for (int i=0; i<sensor_fusion.size(); ++i)
          {
            //other car lane information based on d value in franet coordinate.
            float d = sensor_fusion[i][6];
            int OtherCar_LANE = -1;
            if ((d > 0) && (d < 4))
            {
              OtherCar_LANE = 0;
            }
            else if ((d > 4) && (d < 8))
            {
              OtherCar_LANE = 1;
            }
            else if ((d > 8) && (d < 12))
            {
              OtherCar_LANE = 2;
            }
            if (OtherCar_LANE < 0)
            {
              continue;
            }
            //lateral and longitudinal velocity of other cars
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            //s value of other car
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size*0.02*check_speed); // if using previous points can project s calues outward
            
            //collision checks
            if (lane == OtherCar_LANE)
            {
              car_ahead_of |= (check_car_s>car_s) && ((check_car_s-car_s)<30);
            }
            else if (OtherCar_LANE - lane == 1)
            {
              car_right_side |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
            }
            else if (OtherCar_LANE - lane == -1)
            {
              car_left_side |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
            }

          }

          // behavior prediction of surrounding vehicles
          double  speed_diff = 0;
          if (car_ahead_of)
          {
            if (!car_left_side && lane > 0)
            { // left lane is available and there is no car in left lane
              lane--; // Change lane left
            }
            else if (!car_right_side && lane != 2)
            { // right lane is available and there is no car in right lane
              lane++; // Change lane right
            }
            else
            {
              speed_diff -= 0.224;
            }
            }
            else
            {
              if (lane != 1)
              { // if ego car is not on the center lane.
                if ((lane == 0 && !car_right_side) || (lane == 2 && !car_left_side))
                {
                  lane = 1; // Back to center.
                }
              }
              if (ref_vel < 49.8 )
              {
                speed_diff += 0.224;
              }
            }

          // create a list of (x, y) waypoints
          vector<double> pts_x;
          vector<double> pts_y;

          // reference x, y and yaw states
          // either use start points or previous paths and points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // use car states as the starting reference if the previous size is almost empty (size < 2)
          if (prev_size < 2)
          {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }
          // use previous path's end points as starting reference
          else
          {
            // define previous path's end point as starting reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }

          // In Frenet add evely 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          for (int i=0; i<pts_x.size(); ++i)
          {
            // shift car reference angle to 0 degrees
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            pts_y[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }

          // create a spline
          tk::spline s;
          // set (x, y) points to the spline
          s.set_points(pts_x, pts_y);

          // define the actual (x, y) to be used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all previous path points form last time
          for (int i=0; i<previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity
          double targ_x = 30.0;
          double targ_y = s(targ_x);
          double targ_distance = sqrt((targ_x*targ_x) + (targ_y*targ_y));
          double targ_addOn = 0;

          // fill up the rest of our path planner after filling it with previous points, always output 50 points
          for (int i=1; i<=50-previous_path_x.size(); ++i)
          {
            ref_vel += speed_diff;
            if (ref_vel > 49.8)
            {
              ref_vel = 49.8;
            }
            else if (ref_vel < 0.224)
            {
              ref_vel = 0.224;
            }
            double N = (targ_distance/(0.02*ref_vel/2.24));
            double x_point = targ_addOn + (targ_x)/N;
            double y_point = s(x_point);

            targ_addOn = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back ti normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
  		  json msgJson;
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