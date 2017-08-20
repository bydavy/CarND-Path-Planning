#include "vehicle.h"
#include <iostream>

using namespace std;

Vehicle::Vehicle() {}

Vehicle::Vehicle(const Vehicle &other) {
  id_ = other.id_;
  x_ = other.x_;
  y_ = other.y_;
  s_ = other.s_;
  d_ = other.d_;
  v_ = other.v_;
  a_ = other.a_;
  length_ = other.length_;
  max_acceleration_ = other.max_acceleration_;
  lane_ = other.lane_;
}

Vehicle Vehicle::stateAt(double t) const {
  Vehicle vehicle(*this);
  vehicle.x_ = 0; // Ignore map coordinates
  vehicle.y_ = 0; // Ignore map coordinates
  vehicle.s_ = s_ + v_ * t + a_ * t * t;
  vehicle.v_ = v_ + a_ * t;
  return vehicle;
}

bool Vehicle::collidesWith(Vehicle other, int at_time) const {
    Vehicle v1 = stateAt(at_time);
    Vehicle v2 = other.stateAt(at_time);
    return v1.lane_ == v2.lane_ && fabs(v1.s_ - v2.s_) <= length_/2;
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
  cout << "s:\t\t" << s_ << endl;
  cout << "d:\t\t" << d_ << endl;
  cout << "v:\t\t" << v_ << endl;
  cout << "a:\t\t" << a_ << endl;
  cout << "length:\t\t" << length_ << endl;
  cout << "max_acceleration_:\t\t" << max_acceleration_ << endl;
  cout << endl;
}
