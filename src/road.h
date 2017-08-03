#ifndef ROAD_H
#define ROAD_H

#include <vector>

using namespace std;

class Road {
public:
  const static int LANE_NOT_FOUND = -1;

  // Max s in frenet coordinates in meters
  // Ideally, this should not be hardcoded
  const double MAX_S = 6945.554;

  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;

  // Returns the lane id given frenet coordinates
  int getLaneId(double s, double d);
  // Get lane center d unit in frenet coordinates
  double getLaneCenter(int laneId, double s);
  // Get lane width
  double getLaneWidth(int laneId, double s);
  // Get lane left to current lane
  int getLaneLeftTo(int laneId, double s);
  // Get lane right to current lane
  int getLaneRightTo(int laneId, double s);
  // Returns the speed limit in m/s
  double getSpeedLimit(double s, double d);
};

#endif // ROAD_H
