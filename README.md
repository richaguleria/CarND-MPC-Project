# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Model Predictive Control project provides understanding of how to optimize the control inputs/actuators – steering angle and throttle using MPC that is based on Kinematics Vehicle model.

MPC Inputs are vehicle states: position_x, position_y, velocity, vehicle orientation, cross track error and orientation error.

Kinemetic Vehicle Motion equations to update vehicles states are,

x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt

y​t+1​​=y​t​​+v​t​​∗sin(ψ​t​​)∗dt

ψ​t+1​​=ψ​t​​+​L​f​​​​v​t​​​​∗δ∗dt

v​t+1​​=v​t​​+a​t​​∗dt

cte​t+1​​=f(x​t​​)−y​t​​+(v​t​​∗sin(eψ​t​​)∗dt)

eψ​t+1​​=eψ​t​​+​L​f​​​​v​t​​​​∗δ​t​​∗dt

To plot the reference line, subtract all the waypoints from the vehicle's current position. This helps in plotting a polynomial to the waypoints by bringing the vehicles cordinates at the origin with zero orientation angle. This would ensure car stays in the same horizonal line as the reference line.

For this project, the actuators are predicted for 1 second in future. Horizon of 1 second is covered in N = 10 iterations with each iteration predicting control inputs for the next 100ms(dt). Any smaller value of dt gives better accuracy but will require larger value of N for the horizon of 1 second. Increase in N would lead to larger computational time which effectively increase the latency. Hence the choice of N=10 and dt=0.1 second.

Actuators constraint is set to -25 to +25 for steering angle and -1 to +1 for throttle.

Next, a cost function is defined to minimize the cross track error and orientation error of the vehicle with respect to a line. The approach to minimize the error in control inputs is by tuning the cost function that affects steering value and throttle.

To factor in real-world latency main function sleeps for 100ms before sending control inputs to the vehicle. To contain this latency, I predicted the vehicle state 100ms in future before passing the vehicle state and line coefficients to MPC implementation. This resolved the issue of erratic behavior of the vehicle.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.

* Ipopt and CppAD

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


