clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;
%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP

    disp(i);% Display the current iteration number

    % Extract angular velocity and acceleration from the sampled data IMU
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    
    % Calculate the time elapsed since the last measurement
    dt = sampledTime(i)- prevTime; 

    % Extract the current measurement of velocity and angular velocity from
    % project 2 camera data
    z_t = [vel(i,:)';angVel2(i,:)'];


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

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);