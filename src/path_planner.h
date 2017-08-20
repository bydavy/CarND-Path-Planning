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
  bool canChangeLane(Vehicle ego, vector<Vehicle> others, double delta_t,
    int target_lane);

public:
  PathPlanner(Road& road, Vehicle& vehicle);

  void setEgoVehicleData(Vehicle v);
  void update(vector<Vehicle> others,
    vector<double>& prev_x_vals, vector<double>& prev_y_vals, double end_path_s,
    double end_path_d, vector<double>& next_x_vals,
    vector<double>& next_y_vals);
};

#endif // PATH_PLANNER_H
