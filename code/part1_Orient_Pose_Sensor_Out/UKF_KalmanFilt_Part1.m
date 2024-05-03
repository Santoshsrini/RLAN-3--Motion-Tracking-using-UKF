clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum);

Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.1*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
pos = proj2Data.position;
pose = proj2Data.angle;
for i = 1:length(sampledTime)
    %% Fill in the FOR LOOP

    %disp(i);% Display the current iteration number

    % Extract angular velocity and acceleration from the sampled data IMU
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    
    % Calculate the time elapsed since the last measurement
    dt = sampledTime(i)- prevTime; 

    % Extract the current measurement of position and orientation from
    % camera data of project 2
    z_t = [pos(i,:)';pose(i,:)'];

    % Prediction step: estimate the current state and covariance
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt); 

    % Update step: refine the estimate using the current measurement
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);

    % Save the current state
    savedStates(:,i) = uCurr;

    % Update the variables for the next iteration

    prevTime = sampledTime(i);
    uPrev = uCurr;
    covarPrev= covar_curr;

end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);