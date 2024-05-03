# RLAN-3--Motion-Tracking-using-UKF

In this project, we develop an Unscented Kalman Filter (UKF) to fuse inertial data from an IMU with vision-based pose and velocity estimates for improved state estimation and motion tracking of a quadrotor. The project is divided into two parts: 1) Fusing IMU data with visual pose estimates expressed in the world frame, and 2) Fusing IMU data with velocity estimates from optical flow expressed in the camera frame. The UKF is expected to better capture system nonlinearities compared to the Extended Kalman Filter, at the potential cost of increased runtime. The algorithm was tested on two datasets, with state estimation performance evaluated against ground truth data from a Vicon motion capture system. 	
