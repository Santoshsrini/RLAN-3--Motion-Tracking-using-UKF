# RLAN-3--Motion-Tracking-using-UKF

In this project, we develop an Unscented Kalman Filter (UKF) to fuse inertial data from an IMU with vision-based pose and velocity estimates for improved state estimation and motion tracking of a quadrotor. The project is divided into two parts: 1) Fusing IMU data with visual pose estimates expressed in the world frame, and 2) Fusing IMU data with velocity estimates from optical flow expressed in the camera frame. The UKF is expected to better capture system nonlinearities compared to the Extended Kalman Filter, at the potential cost of increased runtime. The algorithm was tested on two datasets, with state estimation performance evaluated against ground truth data from a Vicon motion capture system. 	

## Part 1:

Part 1 involves fusing IMU data with visual pose estimates by implementing the UKF. For each time step, the state is predicted using the IMU-driven motion model and the UKF prediction step and then updated using the visual pose measurements data.

### Prediction:

Step1: Compute sigma points with state augmentation. 
Step 2: Propagate sigma points through the nonlinear function f
Step 3: Compute the mean and covariance

### Update:

Once the predicted mean and co-variance is computed, the update using the pose measurement from the camera is used for the state update. Since the measurement pose from the camera is already in the world frame, the measurement model is linear. Thus the update equation is the same as that of EKF. 

### Part 1 Results - Dataset 1:

<img width="1222" alt="image" src="https://github.com/Santoshsrini/RLAN-3--Motion-Tracking-using-UKF/assets/28926309/d9902f82-4330-4ce6-9ba6-6ca20a8337b3">


For position and orientation X, Y, and Z, the UKF seems to have performed well, with the predicted values closely following the actual values. The velocity predictions are reasonable but show some deviation, especially noticeable in the X-axis. There's a significant spike in bias gyroscope X and Y axes and a noticeable drift in the Z-axis, which suggests that there may be unmodeled dynamics or sensor errors affecting the gyroscope readings.

### Part 1 Results - Dataset 4:

<img width="1228" alt="image" src="https://github.com/Santoshsrini/RLAN-3--Motion-Tracking-using-UKF/assets/28926309/703efb78-eba1-44a1-b7ef-476bdc5433f8">

The position and orientations predictions  are fairly accurate with the predicted values closely following the actual values across all axes. Velocity in the X and Z axes follows the actual values quite well, while there are some discrepancies in the Y-axis.The bias of the gyroscope shows more noise compared to Dataset 1 but less dramatic spikes. There is a clear upward trend in the bias on the Y-axis, which could point to a systematic error in the gyroscope or a change in the sensor's operating conditions.



