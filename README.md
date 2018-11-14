# Self-Driving Car Engineer Nanodegree Extended Kalman Filter Project

## Project Overview
The goal of this project is to implement the Extended Kalman Filter (EKF) in C++, and use the Kalman Filter with given lidar and radar measurements to track a moving object of interest that travels around the vehicle. To pass the project, calculated RMSE values should be less than or equal to the tolerance provided in the project rubric.

## Source Code Structure
The source code files can be found in the src folder at root level of the Git repository. 
* main.cpp/.h is the entry point of the executable, which communicates with the Term 2 Simulator to obtain inputs via uWebSocketIO, pass the inputs to FusionEKF for making prediction and state update, calculate RMSE values, then pass the calculated outputs back to the simulator.
* FusionEKF.cpp/.h implements the measurement processing logic, including initialization of kalman filter state and covariance matrices, and invoke Predict and Update/UpdateEKF methods based on sensor type using the instance of KalmanFilter
* kalman\_filter.cpp/.h implements the low-level details of prediction calculation and state updates. The mathematical formulas can be found in the [reference](https://s3.amazonaws.com/video.udacity-data.com/topher/2018/June/5b327c11_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf) provided by the course.
* tools.cpp/.h provides the method for calculating RMSE values
* measurement_package.h defines the data structure representing sensor measurement provided by the simulator


## Project Dependencies

* [Udacity Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) v1.45
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) The setup instructions can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/edf28735-efc1-4b99-8fbb-ba9c432239c8/modules/49d8fda9-69c7-4f10-aa18-dc3a2d790cbe/lessons/3feb3671-6252-4c25-adf0-e963af4d9d4a/concepts/7dedf53a-324a-4998-aaf4-e30a3f2cef1d).
* cmake >= 3.5
  * [Installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/) 

## Compilation and Execution Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Launch Simulator, and select "EKF and UKF"
5. Run the compiled program: `./ExtendedKF `

## Measurement Processing Workflow

An illustration of the data flow for sensor measurement processing is shown below:

![Alt text](./images/EKF General Flow.png?raw=true "General Flow of Extended Kalman Filter")

For each sensor measurement, the following attributes are provided:
*Laser*

| type | px | py | timestamp | actual x | actual y | actual vx | actual vy | yaw  | yawrate |
|:---|---|---|---|---|---|---|---|---|---:|
|L	|1.173848 | 0.4810729 | 1477010443100000 | 1.119984 | 0.6002246 | 5.199429 | 0.005389957 | 	0.001036644	| 0.02072960 |

*Radar*

| type | $\rho$ | $\phi$ | $\dot{\rho}$ | timestamp | actual x | actual y | actual vx | actual vy | yaw  | yawrate |
|:---|---|---|---|---|---|---|---|---|---|---:|
|R	|1.047505 |0.3892401 | 4.511325 |1477010443100000 | 1.379955 | 0.6006288 | 5.198979 | 0.01077814 | 	0.002073124	| 0.02763437 |

The difference between two consecutive measurements, $\Delta$t is used to update transition matrix F and process noise covariance matrix Q. With updated F and Q, state prediction can be obtained by using the formula below:

![Alt text](./images/Prediction.png?raw=true "Prediction")

For laser measurement, the raw measurement in each measurement package contains position coordinates px and py, and is passed to method KalmanFilter::Update(). For radar measurement, it is represented in polar coordinates of range, bearing, and range rate, and is passed to method KalmanFilter::UpdateEKF(). Both methods calculate the updated states using the formulas below, with different inputs y, H, and R:

![Alt text](./images/Lidar Update.png?raw=true "Measurement Update for Lidar")

\begin{equation*}
  S = H P' H^T + R
\end{equation*}
\begin{equation*}
  K = P' H^T S^{-1}
\end{equation*}
\begin{equation*}
  x = x' + K y
\end{equation*}
\begin{equation*}
  P = (I - K H) P'
\end{equation*}

In measurement update for laser sensor, the 4D predicted state vector is mapped to the 2D measurement space of lidar sensor, $y = z - Hx'$. In measurement update for radar sensor, the prediction error y is calculated by mapping predicted state vector from cartesian to polar coordinates, then subtract the result from radar measurement: 

$y = z - h(x')$, 
where 
![Alt text](./images/hx for radar.png?raw=true "Measurement Update for Radar")

$h(x') = \left(\begin{array}{cc} \sqrt{p'_{x}^2+p'_{y}^2} \\arctan(\frac{p'_y}{p'_x}) \\ \sqrt{px^2+py^2} \end{array} \right)$

The actual values of position and velocity are used to calculate RMSE values as measurement of the Kalman Filter performance.





INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.




