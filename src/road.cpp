#include "road.h"

// During simulation we know we've 3 lanes everywhere in the track
const int LANE_COUNT = 3;
// Lane width in meter
const double LANE_WIDTH = 4;
// Lane ids
const int LANE_IDS[] = {
  0, // Left lane
  1, // Center lane
  2 // Right lane
};

int Road::getLaneId(double s, double d) {
  double start = 0;
  for (int i = 0; i < LANE_COUNT; i++) {
    double end = start + LANE_WIDTH;
    if (d >= start && d <= end) {
      return LANE_IDS[i];
    }
    start = end;
  }

  return Road::LANE_NOT_FOUND;
}

// Get lane center d in frenet coordinates;
double Road::getLaneCenter(int laneId, double s) {
  if (laneId == Road::LANE_NOT_FOUND) {
    return Road::LANE_NOT_FOUND;
  }

  double laneWidth = getLaneWidth(laneId, s);
  return laneId * laneWidth + laneWidth/2;
}

double Road::getLaneWidth(int laneId, double s) {
  return LANE_WIDTH;
}

int Road::getLaneLeftTo(int laneId, double s) {
  return (laneId >= 1 && laneId <= 2) ? laneId - 1 : Road::LANE_NOT_FOUND;
}

int Road::getLaneRightTo(int laneId, double s){
  return (laneId >= 0 && laneId <= 1) ? laneId + 1 : Road::LANE_NOT_FOUND;
}

double Road::getSpeedLimit(double s, double d) {
  // The speed limit is hardcoded to 50 mph
  return 22.352; // in m/s
}
