# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. run `./clean.sh`
2. run `./build.sh`
2. run `./run.sh`

## Model

The kinetic model is used in this project (following the quiz solution provided by Udacity).

```
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Timestep Length and Elapsed Duration (N & dt)

N & dt are set in MPC.h.
Eventually N was set to 20 and dt was set to 0.05 which means a planning time 20*0.05 = 1sec was considered.

A short planning time as 1s makes sense because the roadway condition is rapidly changing while driving. A finer dt such as 0.025 was also tried. While changing the dt, all other weights in the cost funciton (such as the weight for the change in throttle needs to be adjusted accordingly). To reach the same planning time, a larger N needs to be set while reducing dt, and as a result the computational loads increase. So eventually dt = 0.05 was chosen (following the quiz solution provided by Udacity) and N = 20 was chosen to reach the planning time = 1sec.

## Polynomial Fitting and MPC Preprocessing

All the points was first converted to and then calculated under the vehicle coordinate system.
The following equations were used to convert points from global coordinate system (x,y) to the vehicle coordinate system (x',y'):

```
x'=cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
y'=-sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
```

where ptsx,ptsy and psi are the vechile position and direction in the global coordinate system.

Then based on the fitted poly as object and the vehicle state as input, the solver is called to minimize the cost function over the planning time.

The the cost function is consist of not only the amplitude of cte, difference between velocity and its desired value, steering angle and throttle value, but also the changing rate of steering angle and throttle value.

The weights of each component in the cost function is critical to the final optimal soluation. to make the vehicle driving smoother, a large weight needs to be assigned to the changing rate of steering angle.

Eventually the weights of all components in the cost function were set in MPC.h as follows:

```
const double cte_weight_in_cost = 1;
const double epsi_weight_in_cost = 1;
const double v_weight_in_cost = 1;
const double steer_weight_in_cost = 1;
const double throttle_weight_in_cost = 10;
const double steerchange_weight_in_cost = 1000;
const double throttlechange_weight_in_cost = 1;
```

To make the driving easier, the desired driving speed was set to 50. This can be changed to a higher value but the weights may need to be adjusted to handle the more fragile system.
`const double ref_v = 50;`

## Model Predictive Control with Latency

To handle the latency, the MPC process assumes the steering angle and throttle in the first several steps with the latency time cannot change by setting the constrain values. Then the steering angle and throttle value are free to change right after the latency time. After the MPC process, the first free to change steering angle and throttle value were sent to the simulator. In this way, the MPC take the latency into consideration and provide the actuation values the simulator supposed to use after the lantency time.

## simulating result

After compiling, the vechiele can run smoothing at around 44-45 mph in the MAC simulator with screen resolution = 640*480 and graphics quality = fastest.



