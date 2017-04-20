# Unscented Kalman Filter in C++

An Unscented Kalman Filter that uses a Constant Turn Rate and Velocity (CTRV) motion model in C++. This UKF fuses LIDAR and RADAR sensor readings.

**WIP**
---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`


## FYI:

*Even though linear approximation using a Jacobian matrix stinks ;)*
---
1. Check out my implementation of an [Extended Kalman Filter in C++](https://github.com/mez/extended_kalman_filter_cpp)
