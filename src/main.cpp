#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "../include/Eigen-3.3/Eigen/Core"
#include "../include/Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "way_points.h"
#include "vehicle.h"

#include <glog/logging.h>
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  WayPoints map;
  Vehicle vehicle(map);

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

//  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
//
//  string line;
//  while (getline(in_map_, line)) {
//    std::istringstream iss(line);
//    double x;
//    double y;
//    float s;
//    float d_x;
//    float d_y;
//    iss >> x;
//    iss >> y;
//    iss >> s;
//    iss >> d_x;
//    iss >> d_y;
//    map_waypoints_x.push_back(x);
//    map_waypoints_y.push_back(y);
//    map_waypoints_s.push_back(s);
//    map_waypoints_dx.push_back(d_x);
//    map_waypoints_dy.push_back(d_y);
//  }

  h.onMessage([&vehicle]
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
          CarLocalizationData car_loc_data;
          car_loc_data.x_ = j[1]["x"];
          car_loc_data.y_ = j[1]["y"];
          car_loc_data.s_ = j[1]["s"];
          car_loc_data.d_ = j[1]["d"];
          car_loc_data.yaw_ = j[1]["yaw"];
          car_loc_data.speed_ = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
//          double end_path_s = j[1]["end_path_s"];
//          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<SensorFusionData>& sf_data = vehicle.SensorFusionStorage();
          sf_data.resize(sensor_fusion.size());
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            auto& sfd = sf_data[i];
            sfd.id_ = sensor_fusion[i][0];
            sfd.x_ = sensor_fusion[i][1];
            sfd.y_ = sensor_fusion[i][2];
            sfd.vx_ = sensor_fusion[i][3];
            sfd.vy_ = sensor_fusion[i][4];
            sfd.s_ = sensor_fusion[i][5];
            sfd.d_ = sensor_fusion[i][6];
          }
          car_loc_data.d_ = -car_loc_data.d_;

          vehicle.UpdateTrajectory(car_loc_data, previous_path_x, previous_path_y);

          json msgJson;

          msgJson["next_x"] = vehicle.GetNextXValues();
          msgJson["next_y"] = vehicle.GetNextYValues();

          std::cout<<vehicle.GetNextYValues().size()<<std::endl;
          auto msg = "42[\"control\"," + msgJson.dump() + "]";

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