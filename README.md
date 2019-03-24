# CarND-PID-Control-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./Images/Model_predictive_control_setup.JPG "MPC"

## Overwiew

In this project we'll implement the Model Predictive Control to drive the car around the track. 
This time however we're not given the cross track error, we'll have to calculate that ourselfs! 
Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Project Rubic points

### Your code should compile.

The code compiled without errors. No modification were made on the initial setup.

### The Model

Describing the JSON object send back from the simulator command server:

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.


### `psi` and `psi_unity` representations

`psi`

```
//            90
//
//  180                   0/360
//
//            270
```


`psi_unity`

```
//           0/360
//
//  270                   90
//
//            180
```

The Kinematic model setup:

![alt text][image1] 

* x, y  : Vehicle position.
* psi   : Vehicle orientation angle.
* v     : Vehicle velocity.
* cte   : Cross-track error.
* epsi  : Orientation error.
* a 	: Vehicle acceleration (throttle).
* delta : Steering angle.
* Lf	: The distance between the car of mass and the front wheels.

### Timestep Length and Elapsed Duration (N & dt)

For the prediction horizon I chose the N (number of timesteps) and dt (elapsed duration between timesteps) hyperparameter to 10 and 0,1. 
I tried to use more steps at the begining of the tuning process, but it turned out the car became unstable in the turns.

### Polynomial Fitting and MPC Preprocessing

I transformed the points from the simulator's global coordinates into the vehicle's coordinates at first. 
Then, I used the polyfit function to make a third-degree polynomial line that fits for the transformed points. 
After that, I calculated the cross-track error with the polyeval function and also calculated the error psi value (epsi) which is the difference between the psi and the path angle.
These values are used by the MPC solver to calculate the steering and throttle values.

### Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. 
A realistic delay was introduced in this project with 100 milliseconds.
At low speed (below 25 mph) this delay is not very noticable, but at higher speed it can result undesired behavior.
This latency was calculated into the state parameter and passed to the state vector so the MPC Solver calculate the steering and throttle values with this latency.

### The vehicle must successfully drive a lap around the track.
The car was successfully drove around the track.