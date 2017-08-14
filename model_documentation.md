# CarND-Path-Planning

[![Demo running on simulator](https://img.youtube.com/vi/lS_J9u7eqvQ/0.jpg)](https://youtu.be/lS_J9u7eqvQ "Demo")

## Path generation

The path is solved in the map coordinate system. An approximate path is defined first and it's composed of 5 points, the previous car position, the current car position and 3 positions in the future at 30, 60 and 90 meters ahead (this are generated thanks to known map waypoints).
Spline is used to generate smooth car positions between those 5 points, at the desired interval in order to regulate the car speed.

## Car speed

The car speed is either the speed limit or the speed of the car in front of it. This makes sure the car will not collide other cars in the same lane.

## Lane change

The car will attempt to change lane if the car in front of it doesn't drive at the speed limit. In such even, the change lane will be initiated if and only if there is no car in the close proximity behind and ahead in the target lane.

## Improvements

We could introduce the notion of states (keep lane, change lane left, change lane right, etc). And use a cost function to determine which state if the most appropriate.
