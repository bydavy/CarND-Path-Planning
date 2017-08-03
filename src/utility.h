#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <math.h>

using namespace std;

#define MPH2MS 0.44704

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int closestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int nextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

#endif // UTILITY_H
