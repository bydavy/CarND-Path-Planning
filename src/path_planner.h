#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "vehicle.h"
#include "road.h"
#include <vector>

using namespace std;

enum State {
  KEEP_LANE,
  PREPARE_LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT,
};

class PathPlanner {
private:
  Road& road_;
  Vehicle& ego_;
  State state_;
  // Updates the state if the internal machine state
  void updateState(vector<Vehicle> others);
  void realizeKeepLane(vector<Vehicle> others,
    vector<double>& prev_x_vals, vector<double>& prev_y_vals,
    vector<double>& next_x_vals, vector<double>& next_y_vals);
  double maxAccelForLane(vector<Vehicle> others);

public:
  PathPlanner(Road& road, Vehicle& vehicle);

  void setEgoVehicleData(double car_x, double car_y, double car_s, double car_d,
    double car_yaw, double car_speed);
  void update(vector<Vehicle> others,
    vector<double>& prev_x_vals, vector<double>& prev_y_vals,
    vector<double>& next_x_vals, vector<double>& next_y_vals);
};

#endif // PATH_PLANNER_H
