#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "path_planner.h"
#include "road.h"
#include "utility.h"

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

// Converts the json sensor_fusion data to a list of vehicle objects
void toVehicles(Road& road, json& sensor_fusion, vector<Vehicle>& out) {
  for (const auto& data: sensor_fusion) {
    Vehicle vehicle;
    vehicle.id_ = data[0];
    vehicle.x_ = data[1];
    vehicle.y_ = data[2];
    double vx = data[3]; // Already in m/s
    double vy = data[4]; // Already in m/s
    vehicle.s_ = data[5];
    vehicle.d_ = data[6];
    vehicle.v_ = sqrt(vx*vx + vy*vy);
    vehicle.a_ = 0; // Ignore acceleration
    vehicle.yaw_ = atan2(vx, vy);
    vehicle.lane_ = road.getLaneId(vehicle.s_, vehicle.d_);
    out.push_back(vehicle);
  }
}

void loadWaypoints(Road& road) {
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

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
    road.map_waypoints_x_.push_back(x);
    road.map_waypoints_y_.push_back(y);
    road.map_waypoints_s_.push_back(s);
    road.map_waypoints_dx_.push_back(d_x);
    road.map_waypoints_dy_.push_back(d_y);
  }
}

int main() {
  uWS::Hub h;

  Road road;
  Vehicle vehicle;
  PathPlanner pathPlanner(road, vehicle);

  loadWaypoints(road);

  h.onMessage([&road,&pathPlanner](uWS::WebSocket<uWS::SERVER> ws, char *data,
      size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	car_yaw = deg2rad(car_yaw); // deg to radian
          	double car_speed = j[1]["speed"]; // mph to m/s conversion
          	car_speed *= MPH2MS;

            Vehicle ego;
            ego.x_ = car_x;
            ego.y_ = car_y;
            ego.s_ = car_s;
            ego.d_ = car_d;
            ego.yaw_ = car_yaw;
            ego.v_ = car_speed;
            ego.lane_ = road.getLaneId(car_s, car_d);

            // Update current vehicle information
            pathPlanner.setEgoVehicleData(ego);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"].get<vector<double>>();
          	auto previous_path_y = j[1]["previous_path_y"].get<vector<double>>();
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // Convert sensor_fusion to vehicles objects
            vector<Vehicle> vehicles;
            toVehicles(road, sensor_fusion, vehicles);

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // Generate new path
            pathPlanner.update(vehicles, previous_path_x, previous_path_y,
              end_path_s, end_path_d, next_x_vals, next_y_vals);

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
