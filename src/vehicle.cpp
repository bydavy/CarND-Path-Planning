#include "vehicle.h"
#include <iostream>

using namespace std;

Vehicle::Vehicle() {}

Vehicle::Vehicle(const Vehicle &other) {
  id_ = other.id_;
  x_ = other.x_;
  y_ = other.y_;
  vx_ = other.vx_;
  vy_ = other.vy_;
  v_ = other.v_;
  a_ = other.a_;
  s_ = other.s_;
  d_ = other.d_;
  d_dot_ = other.d_dot_;
  d_dot_dot_ = other.d_dot_dot_;
  lane_ = other.lane_;
  length_ = other.length_;
  yaw_ = other.yaw_;
  max_acceleration_ = other.max_acceleration_;
}

Vehicle::PredictionState Vehicle::stateAt(double t) const {
  // We assume the vehicle stays in its current lane
  PredictionState prediction;
  prediction.lane = lane_;
  prediction.s = s_ + v_ * t + a_ * t * t / 2;
  prediction.d = d_;
  prediction.v = v_ + a_ * t;
  prediction.a = a_;
  return prediction;
}

bool Vehicle::collidesWith(Vehicle other, int at_time) const {
    PredictionState p1 = stateAt(at_time);
    PredictionState p2 = other.stateAt(at_time);
    return (p1.lane == p2.lane) && (fabs(p1.s-p2.s) <= length_);
}

Vehicle::Collider Vehicle::willCollideWith(Vehicle other, int timesteps, double time_increment) const {
	Vehicle::Collider colliderTemp;
	colliderTemp.collision = false;
	colliderTemp.time = -1;

	for (double t = 0; t <= time_increment; t+=time_increment) {
    if(collidesWith(other, t)) {
		    colliderTemp.collision = true;
        colliderTemp.time = t;
        return colliderTemp;
    }
  }

	return colliderTemp;
}

void Vehicle::display() const {
  cout << "*** Vehicle ***" << endl;
  cout << "id:\t\t" << id_ << endl;
  cout << "x:\t\t" << x_ << endl;
  cout << "y:\t\t" << y_ << endl;
  cout << "vx:\t\t" << vx_ << endl;
  cout << "vy:\t\t" << vy_ << endl;
  cout << "a:\t\t" << a_ << endl;
  cout << "s:\t\t" << s_ << endl;
  cout << "s_dot:\t\t" << v_ << endl;
  cout << "d:\t\t" << d_ << endl;
  cout << "d_dot:\t\t" << d_dot_ << endl;
  cout << "d_dot_dot:\t" << d_dot_dot_ << endl;
  cout << "lane:\t\t" << lane_ << endl;
  cout << endl;
}
