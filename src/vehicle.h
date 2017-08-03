#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <vector>
#include "road.h"

using namespace std;

class Vehicle {
private:
public:
  struct Collider {
    bool collision ; // is there a collision?
    double time; // time collision happens in seconds (relative to current time)
  };

  struct PredictionState {
    int lane;
    double s;
    double d;
    double v;
    double a;
  };

  // Unique id
  int id_ = -1;
  // x position in global map coordinates in meters
  double x_;
  // y position in global map coordinates in meters
  double y_;
  // x velocity in m/s
  double vx_;
  // y velocity in m/s
  double vy_;
  // s position in frenet coordinates in meters
  double s_;
  // d position in frenet coordinates in meters
  double d_;
  // vecolity in m/s
  double v_;
  // acceleration in m/s^2
  double a_;
  // d velocity in m/s
  double d_dot_;
  // d velocity in m/s^2
  double d_dot_dot_;
  // yaw in radian
  double yaw_;
  // lane id
  int lane_ = Road::LANE_NOT_FOUND;
  // Vehicle length in meters
  double length_ = 4.5;
  // Max acceleration in m/s^2
  double max_acceleration_ = 20;

  Vehicle();

  Vehicle(const Vehicle &other);

  // Return the predicated state of the car for the given relative time
  // t is in seconds
  PredictionState stateAt(double t);

  // Tests collision between this vehicle and another one at a given time in seconds
  bool collidesWith(Vehicle other, int at_time);

  // Tests collision between this cehicle and another one for a given time interval in seconds
  Collider willCollideWith(Vehicle other, int timesteps, double time_increment);

  // Print out human readable representation of the Vehicle
  void display();
};

#endif // VEHICLE_H
