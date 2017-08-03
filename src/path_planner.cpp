#include "path_planner.h"
#include <iostream>
#include <math.h>
#include "utility.h"

using namespace std;

// Timestep in seconds
const double PREDICTION_TIMESTEP = 0.02; // 20ms
const double PREDICTIONS_PER_SECOND = 1/PREDICTION_TIMESTEP;
// How many steps in the future to predict the car behavior
const int PREDICTION_STEPS = PREDICTIONS_PER_SECOND;

PathPlanner::PathPlanner(Road& road, Vehicle& vehicle):
  state_(State::KEEP_LANE),road_(road),ego_(vehicle) {}

void PathPlanner::setEgoVehicleData(double car_x, double car_y, double car_s,
  double car_d, double car_yaw, double car_speed) {
    ego_.x_ = car_x;
    ego_.y_ = car_y;
    ego_.s_ = car_s;
    ego_.d_ = car_d;
    ego_.yaw_ = car_yaw;
    ego_.v_ = car_speed;
    ego_.lane_ = road_.getLaneId(car_s, car_d);
}

void PathPlanner::updateState(vector<Vehicle> others) {
  // TODO Implement the state selection logic
  state_ = State::KEEP_LANE;
}

void PathPlanner::update(vector<Vehicle> others,
  vector<double>& prev_x_vals, vector<double>& prev_y_vals,
  double end_path_s, double end_path_d,
  vector<double>& next_x_vals, vector<double>& next_y_vals) {
  // Initialize target lane
  if (targetLaneId_ == Road::LANE_NOT_FOUND) {
    targetLaneId_ = road_.getLaneId(ego_.s_, ego_.d_);
    if (targetLaneId_ == Road::LANE_NOT_FOUND) {
      targetLaneId_ = 0;
    }
  }

  updateState(others);

  if (State::KEEP_LANE == state_) {
      realizeKeepLane(others, prev_x_vals, prev_y_vals, end_path_s, end_path_d,
        next_x_vals, next_y_vals);
  } else {
    cout << "Unknown state: " << state_ << endl;
  }
}

double PathPlanner::maxAccelForLane(vector<Vehicle> others) {
  vector<Vehicle> in_front;
  for(auto const& v: others) {
    if((v.lane_ == ego_.lane_) && (v.s_ > ego_.s_)) {
      in_front.push_back(v);
    }
  }

  double delta_v_til_target = road_.getSpeedLimit(ego_.s_, ego_.d_) - ego_.v_;
  double max_acc = fmin(ego_.max_acceleration_, delta_v_til_target);
  cout << "Max acc: " << max_acc << endl;

  if (in_front.size() > 0) { // Avoid collision
    Vehicle vehicle = in_front[0];
    Vehicle::Collider collider = ego_.willCollideWith(vehicle, 50, 0.02);
    if (collider.collision) {
      // We should slow down
      long distance = vehicle.s_ - ego_.s_;
      long delta_v_til_collision = vehicle.v_ - ego_.v_;
    }
  }

  return max_acc;
}

void PathPlanner::realizeKeepLane(vector<Vehicle> others,
  vector<double>& prev_x_vals, vector<double>& prev_y_vals,
  double end_path_s, double end_path_d,
  vector<double>& next_x_vals, vector<double>& next_y_vals) {
  int prev_size = prev_x_vals.size();

  double s = ego_.s_;
  double d = 0;
  double v = ego_.v_;
  double a = 20;

  if (prev_size > 3) {
    // Copy previous path
    for (int i = 0; i < prev_size; i++) {
      next_x_vals.push_back(prev_x_vals[i]);
      next_y_vals.push_back(prev_y_vals[i]);
    }

    // Update starting vehicle state
    double x_0 = next_x_vals[prev_size-3];
    double y_0 = next_y_vals[prev_size-3];
    double x_1 = next_x_vals[prev_size-2];
    double y_1 = next_y_vals[prev_size-2];
    double x_2 = next_x_vals[prev_size-1];
    double y_2 = next_y_vals[prev_size-1];
    double vx_1 = (x_1 - x_0) * PREDICTIONS_PER_SECOND;
    double vy_1 = (y_1 - y_0) * PREDICTIONS_PER_SECOND;
    double v_1 = sqrt(vx_1*vx_1 + vy_1*vy_1);

    double vx_2 = (x_2 - x_1) * PREDICTIONS_PER_SECOND;
    double vy_2 = (y_2 - y_1) * PREDICTIONS_PER_SECOND;
    double v_2 = sqrt(vx_2*vx_2 + vy_2*vy_2);

    s = end_path_s;
    //d = end_path_d;
    v = v_2;
    a = (v_2 - v_1) / PREDICTION_TIMESTEP;
  }

  // Starting delta time for prediction
  double t = prev_size * PREDICTION_TIMESTEP;
  int pred_steps = min(PREDICTION_STEPS - prev_size, 0);

  for (int i = 0; i < pred_steps; i++) {
    // Solve s
    a = 20;
    double v1 = v + a * PREDICTION_TIMESTEP;
    if (v1 > 20) {
      v1 = 20;
    }
    double delta_s = (v1 + v) / 2 * PREDICTION_TIMESTEP;

    // Update states
    s += delta_s;
    v = v1;

    // Solve d
    double target_d = road_.getLaneCenter(s, targetLaneId_);

    vector<double> pts = getXY(s, target_d, road_.map_waypoints_s_,
      road_.map_waypoints_x_, road_.map_waypoints_y_);
    next_x_vals.push_back(pts[0]);
    next_y_vals.push_back(pts[1]);

    t += PREDICTION_TIMESTEP;
  }
}
