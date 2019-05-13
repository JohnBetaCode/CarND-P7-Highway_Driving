#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

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
  string map_file_ = "./data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Open and load csv to variable
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  // Read csv lines and create state vectors
  string line;
  std::cout << "Loading" << std::endl;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);

    double x;  // Cars x position
    double y;  // Cars y position
    float s;   // Cars longitudinal position
    float d_x; // Cars x speed
    float d_y; // Cars y speed

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;

    // Add map values for waypoint's x,y,s and d normalized 
    // normal vectors
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0.0; //[mph]

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        // get event
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"]; // Car's x axis global location
          double car_y = j[1]["y"]; // Car's y axis global location
          double car_s = j[1]["s"]; // Car's frenet coordinate s
          double car_d = j[1]["d"]; // Car's frenet coordinate d
          double car_yaw = j[1]["yaw"]; // Car's yaw
          double car_speed = j[1]["speed"]; // Car's speed

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          // of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // ----------------------------------------------------------------
          // Hyper parameters
          
          float SPACE_GAP = 40; // [m] Gap from car to obstacles
          float DECEL_CONTS = 0.524; // 0.224 is around 5m/s2 which is under 10m/s2 for jerk requirement
          float ACELE_CONTS = 0.424; // 0.224 is around 5m/s2 which is under 10m/s2 for jerk requirement
          float SPEED_LIMIT = 49.5; // Maximum speed limit
          float LANE_WIDTH = 4.0; // Lane width
          int OUTPUT_POINTS = 50; 

          // ----------------------------------------------------------------
          // Get the previous path size
          int prev_size = previous_path_x.size();

          // If having points to work with, change the cars s, representing the previous
          // path lap points s  
          if(prev_size > 0){
            car_s - end_path_s;
          }

          // Variables to check for close objects forward and backward in left center and right lanes
          bool close_obj_cf = false; // Close object forward in the current lane
          bool close_obj_l = false; // Close object forward in the left lane
          bool close_obj_r = false; // Close object forward in the right lane

          // If car is in left lane car can't turn to the left
          // Equivalent to an obstacle ahead
          if(lane==0){ 
            close_obj_l = true;
          }
          // If car is in right lane car can't turn to the right
          // Equivalent to an obstacle ahead
          else if(lane==2){
            close_obj_r = true;
          }

          // Initialization for obstacles
          float close_obj_ref_vel = 0;
          float close_obj_cf_s = 1000000;
          
          // Find ref_v to use
          for(int i=0; i<sensor_fusion.size(); i++){

            // Get Cars properties
            int obj_id = sensor_fusion[i][0]; // car's identifier
            float obj_x = sensor_fusion[i][1]; // car's x axis global coordinate
            float obj_y = sensor_fusion[i][2]; // car's y axis global coordinate
            double obj_vx = sensor_fusion[i][3]; // car's x velocity component
            double obj_vy = sensor_fusion[i][4]; // car's y velocity component
            double obj_s = sensor_fusion[i][5]; // car's s frenet coordinate
            float obj_d = sensor_fusion[i][6]; // car's d frenet coordinate

            // Print obstacle information
            // std::cout<<"|car_id:"<<obj_id<<"\t| x:"<<obj_x<<"\t| y:"<<obj_y<<"\t| vx:" 
            //   <<obj_vx<<"\t| vy:"<< obj_vy<<"\t| s:"<<obj_s<<"\t| d:"<<obj_d<<"\t|"<< std::endl;

            // If using previous points can project s value out
            // See where the car is going to be in the future
            double check_speed = sqrt(obj_vx*obj_vx+obj_vy*obj_vy);
            obj_s += ((double)prev_size*.02*check_speed);

            // Check for a close objects ahead
            if((obj_s>car_s)&&((obj_s-car_s)<SPACE_GAP)){

              // Check if obstacle car is in the current lane
              if(obj_d < (LANE_WIDTH*.5+LANE_WIDTH*lane+LANE_WIDTH*.5) 
              && obj_d > (LANE_WIDTH*.5+LANE_WIDTH*lane-LANE_WIDTH*.5)){
                if(obj_s<close_obj_cf_s){
                    close_obj_cf = true;
                    close_obj_cf_s = obj_s;
                    close_obj_ref_vel = check_speed*2.24; // 2.24 because mph
                }
              }
            
            // Check for obstacles in the left and the right lanes
            if(abs(obj_s-car_s)<SPACE_GAP){

                // Check obstacles to the right
                if(obj_d < (LANE_WIDTH*.5+LANE_WIDTH*(lane+1)+LANE_WIDTH*.5) 
                && obj_d > (LANE_WIDTH*.5+LANE_WIDTH*(lane+1)-LANE_WIDTH*.5)
                && lane!=2){
                  close_obj_r = true;
                }
                // Check obstacles to the left
                else if(obj_d < (LANE_WIDTH*.5+LANE_WIDTH*(lane-1)+LANE_WIDTH*.5) 
                && obj_d > (LANE_WIDTH*.5+LANE_WIDTH*(lane-1)-LANE_WIDTH*.5)
                && lane!=0){
                  close_obj_l = true;
                }
              }
              
            }             
          }

          // Print turning information
          std::cout<<car_d<<"Left Turn: "<<!close_obj_l<<"| Right Turn: "<<!close_obj_r<< std::endl;

          // ----------------------------------------------------------------
          // Acceleration/Deceleration heuristic

          // If there's an obstacle ahead
          if(close_obj_cf){
            if(ref_vel > close_obj_ref_vel){

               // slow down gradually to avoid future collision
              ref_vel -= DECEL_CONTS*(1.-SPACE_GAP/close_obj_cf_s);

              // brake completely immediately to avoid collision
              if(close_obj_cf_s-car_s<SPACE_GAP*0.1){
                ref_vel = 0;
              }
              // brake sharply immediately for the same speed of the car ahead
              else if(close_obj_cf_s-car_s<SPACE_GAP*0.3){
                ref_vel = close_obj_ref_vel;
              }
              // slow down even more is object is very close
              else if(close_obj_cf_s-car_s<SPACE_GAP*0.5){
                ref_vel -= (DECEL_CONTS*(1.-SPACE_GAP/close_obj_cf_s))*0.6;
              }
            }

            // Change lane heuristic
            if(!close_obj_l){
              lane -= 1;
            }
            else if(!close_obj_r){
              lane += 1;
            }

          }
          // If there's an obstacle ahead
          else if(ref_vel < SPEED_LIMIT){
            ref_vel += ACELE_CONTS;
          }

          // ----------------------------------------------------------------
          // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with 
          // more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y yaw states
          // either we will reference the starting point as where the car is or at the 
          // previous paths end points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty, use the car as starting reference
          if(prev_size < 2){
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else{
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In frenet and evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+60, (LANE_WIDTH*.5+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+120, (LANE_WIDTH*.5+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s+180, (LANE_WIDTH*.5+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i=0; i < ptsx.size(); i++){

            // Shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
 
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }

          // Create a spline
          tk::spline s;

          // Set (x, y) points to the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x, y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for(int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points,
          // here we will always output 50 points
          for(int i=1; i <= OUTPUT_POINTS-previous_path_x.size(); i++){

            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point; 

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // ----------------------------------------------------------------
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