#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "vehicle.h"
#include "road.h"
#include <vector>
#include <time.h>

using namespace std;

class PathPlanner {
private:
  Road& road_;
  Vehicle& ego_;
  int targetLaneId_;
  time_t lastLaneChange_;
  bool canChangeLane(vector<Vehicle> others, Vehicle::PredictionState state,
    double delta_t, int target_lane);

public:
  PathPlanner(Road& road, Vehicle& vehicle);

  void setEgoVehicleData(double car_x, double car_y, double car_s, double car_d,
    double car_yaw, double car_speed);
  void update(vector<Vehicle> others,
    vector<double>& prev_x_vals, vector<double>& prev_y_vals, double end_path_s,
    double end_path_d, vector<double>& next_x_vals,
    vector<double>& next_y_vals);
};

#endif // PATH_PLANNER_H
