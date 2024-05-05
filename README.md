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

### Part 1 Results - Dataset 4:

<img width="1228" alt="image" src="https://github.com/Santoshsrini/RLAN-3--Motion-Tracking-using-UKF/assets/28926309/703efb78-eba1-44a1-b7ef-476bdc5433f8">


## Part 2:

### Prediction: 

Exactly same as part 1.

### Update:

Since measurement model is non linear because of transformation of frame from world to camera as the actual measurement is in camera frame,  update is done using Unscented transform

Step1: Compute sigma points
Step 2: Propagate sigma points through the nonlinear function g:
Step 3: Compute the mean and covariance
Step 4: Update using Kalman Gain: 

### Part 2 Results - Dataset 1:

<img width="1200" alt="image" src="https://github.com/Santoshsrini/RLAN-3--Motion-Tracking-using-UKF/assets/28926309/5bcedf69-724f-4857-a20d-423dd9f16849">


### Part 2 Results - Dataset 2:

<img width="1218" alt="image" src="https://github.com/Santoshsrini/RLAN-3--Motion-Tracking-using-UKF/assets/28926309/afeabf7d-5ed5-448c-afa7-3d5e1e5d1391">






