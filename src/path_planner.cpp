#include "path_planner.h"
#include <iostream>
#include <math.h>
#include "utility.h"
#include "spline.h"

using namespace std;

// Timestep in seconds
const double PREDICTION_TIMESTEP = 0.02; // 20ms
const double PREDICTIONS_PER_SECOND = 1/PREDICTION_TIMESTEP;
// How many steps in the future to predict the car behavior
const int PREDICTION_STEPS = 45;
// in meters
const double PREDICTION_DISTANCE = 30;
// in km/h
const double LEGAL_VELOCITY_PADDING = 2;

PathPlanner::PathPlanner(Road& road, Vehicle& vehicle):
  road_(road),ego_(vehicle),
  targetLaneId_(Road::LANE_NOT_FOUND) {}

void PathPlanner::setEgoVehicleData(Vehicle vehicle) {
  ego_ = vehicle;
}

bool PathPlanner::canChangeLane(Vehicle ego, vector<Vehicle> others,
    double delta_t, int target_lane) {
  if (target_lane == Road::LANE_NOT_FOUND) {
    return false;
  }

  for(Vehicle const& v: others) {
    if(v.lane_ != target_lane) { // Ignore if not same lane
      continue;
    }

    const double s = v.stateAt(delta_t).s_;
    const double dist = fabs(s - ego.s_);
    bool inFront = ego.s_ < s && dist < 20;
    bool behind = ego.s_ > s && dist < 10;
    if (inFront || behind) {
        return false;
    }
  }

  return true;
}

void PathPlanner::update(vector<Vehicle> others,
    vector<double>& prev_x_vals, vector<double>& prev_y_vals,
    double end_path_s, double end_path_d,
    vector<double>& next_x_vals, vector<double>& next_y_vals) {
  // Initialize target lane
  if (targetLaneId_ == Road::LANE_NOT_FOUND) {
    targetLaneId_ = road_.getLaneId(ego_.s_, ego_.d_);
    if (targetLaneId_ == Road::LANE_NOT_FOUND) {
      targetLaneId_ = 0; // Fallback to line 0
    }
  }

  const int prev_size = prev_x_vals.size();

  vector<double> ptsx;
  vector<double> ptsy;

  double prev_car_x;
  double prev_car_y;

  // Reference car position used to complete the path
  // This might be the current car position (bootstrap), or a prediction
  double ref_x;
  double ref_y;
  double ref_s;
  double ref_d;
  double ref_yaw;
  double ref_v;
  double ref_t = prev_size * PREDICTION_TIMESTEP;

  if (prev_size < 2) {
    // Fake previous position one second ago based on yaw
    prev_car_x = ego_.x_ - cos(ego_.yaw_);
    prev_car_y = ego_.y_ - sin(ego_.yaw_);

    // Current position
    ref_x = ego_.x_;
    ref_y = ego_.y_;
    ref_s = ego_.s_;
    ref_d = ego_.d_;
    ref_yaw = ego_.yaw_;
    ref_v = ego_.v_;
  } else {
    // Copy previous path
    // The more entries we copy the more our car will take time to react
    // to environment changes, as this path is based on predictions.
    for (int i = 0; i < prev_size; i++) {
      next_x_vals.push_back(prev_x_vals[i]);
      next_y_vals.push_back(prev_y_vals[i]);
    }

    // Previous position (second to last entry in previous path)
    prev_car_x = prev_x_vals[prev_size-2];
    prev_car_y = prev_y_vals[prev_size-2];

    // Current position (last entry in previous path)
    ref_x = prev_x_vals[prev_size-1];
    ref_y = prev_y_vals[prev_size-1];
    ref_s = end_path_s;
    ref_d = end_path_d;
    ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
    ref_v = distance(prev_car_x, prev_car_y, ref_x, ref_y) / PREDICTION_TIMESTEP;
  }

  // Detect if we will collide a car soon
  Vehicle* inFront = NULL;
  for(auto const& v: others) {
    if(v.lane_ != ego_.lane_ && v.lane_ != targetLaneId_) { // Ignore if not same lane
      continue;
    }

    const double s = v.stateAt(ref_t).s_;
    if (s > ref_s && (s - ref_s < 30)) { // A car in front of us
      if (inFront == NULL || inFront->stateAt(ref_t).s_ > s) {
        inFront = new Vehicle(v);
      }
    }
  }

  // Should we change lane?
  // FIXME Move to a state machine and cost to elect next state
  time_t now = time(NULL);
  if (inFront != NULL && ego_.lane_ == targetLaneId_ &&
      difftime(now, lastLaneChange_) > 5 /* seconds */) {
    Vehicle pred_v = Vehicle(ego_);
    pred_v.s_ = ref_s;
    pred_v.d_ = ref_d;
    pred_v.v_ = ref_v;

    const int left_lane = road_.getLaneLeftTo(ego_.lane_, ref_s);
    const int right_lane = road_.getLaneRightTo(ego_.lane_, ref_s);
    if (canChangeLane(pred_v, others, ref_t, left_lane)) {
      targetLaneId_ = left_lane;
      lastLaneChange_ = time(NULL);
    } else if (canChangeLane(pred_v, others, ref_t, right_lane)) {
      targetLaneId_ = right_lane;
      lastLaneChange_ = time(NULL);
    }
  }

  // Target speed
  double target_v = road_.getSpeedLimit(ref_s, ref_d);
  if (inFront != NULL) { // Collision! Adapt speed to vehicle in front of us
    target_v = min(target_v, inFront->v_);
  }

  // Previous position
  ptsx.push_back(prev_car_x);
  ptsy.push_back(prev_car_y);

  // Current position
  ptsx.push_back(ref_x);
  ptsy.push_back(ref_y);

  // Add 3 waypoints spread out, to estimate the road curvature with spline
  for (int i = 1; i < 4; i++) {
    double s = ref_s + PREDICTION_DISTANCE * i;
    double d = road_.getLaneCenter(targetLaneId_, s);
    vector<double> next_waypoint = getXY(s, d, road_.map_waypoints_s_,
      road_.map_waypoints_x_, road_.map_waypoints_y_);

    ptsx.push_back(next_waypoint[0]);
    ptsy.push_back(next_waypoint[1]);
  }

  // Convert from map to car coordinates system
  // This will solve a problem with spline and a vertical curvature generating
  // mutiple y for a single x.
  for (int i = 0; i < ptsx.size(); i++) {
    const double shift_x = ptsx[i] - ref_x;
    const double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
    ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  const double target_x = PREDICTION_DISTANCE;
  const double target_y = s(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);

  // Position relative to reference car position {ref_x, ref_y}
  double x = 0;
  double v = ref_v;

  // Starting delta time for prediction
  const int pred_steps = max(PREDICTION_STEPS - prev_size, 0);
  for (int i = 0; i < pred_steps; i++) {
    // Update velocity (update only if too far from target_v)
    if (v < target_v - LEGAL_VELOCITY_PADDING || v >= target_v) {
      double a = ego_.max_acceleration_ * PREDICTION_TIMESTEP;
      if (v > target_v) { // Decelerate
        a = -a;
      }
      v += a;
    }
    const double n = target_dist / (v*PREDICTION_TIMESTEP);
    x += target_x / n;
    const double y = s(x);

    // Convert from car to map coordinates system
    const double x_map = ref_x + (x * cos(ref_yaw-0) - y * sin(ref_yaw-0));
    const double y_map = ref_y + (x * sin(ref_yaw-0) + y * cos(ref_yaw-0));

    next_x_vals.push_back(x_map);
    next_y_vals.push_back(y_map);
  }

  if (inFront != NULL) {
    free(inFront);
    inFront = NULL;
  }
}
