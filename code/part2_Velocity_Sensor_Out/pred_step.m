function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 
    

    % Determine the length of the previous state vector
    n = length(uPrev);

    % Define the size for quaternion and noise (3 for each quaternion component)
    nq = 4*3;

    % Define the augmented state dimension (state vector + noise)
    n_aug = n+nq;

    % Initialize the zero vector of length nq
    zeroVector = zeros(nq, 1);

    % Append the zero vector to the existing vector uPrev
    uPrev_aug = [uPrev; zeroVector];

    % Define the process noise covariance matrix 'Q'
    % and scale it by the time step 'dt' to get 'Qd'
    Qt = 0.004*eye(nq);
    Qd = Qt * dt;

    % Create a zero matrix of appropriate sizes to fill the off-diagonal blocks
    zeroMatrixTop = zeros(n, nq);  % This will be on the top-right
    zeroMatrixBottom = zeros(nq, n);  % This will be on the bottom-left
    
    % Construct the augmented covaraince matrix
    Paug = [covarPrev, zeroMatrixTop;
            zeroMatrixBottom, Qd];

    % Calculate the square root of the augmented covariance matrix using Cholesky decomposition
    Paug_sqrt = chol(Paug);

    % Scaling parameters for Unscented Transform
    alpha = 0.001;
    k=1;
    beta = 2;

    % Compute lambda for scaling the sigma points
    lambda = (alpha^2) * (n_aug + k) - n_aug;

    % Initialize matrix for sigma points
    Xaug_prev = zeros(n_aug,2*n_aug+1);

    % Generate sigma points around the mean
    for i = 1:n_aug

        Xaug_prev(:,i) = uPrev_aug+sqrt(n_aug+lambda)*Paug_sqrt(:,i);
        Xaug_prev(:,i+n_aug) = uPrev_aug-sqrt(n_aug+lambda)*Paug_sqrt(:,i);

    end

    Xaug_prev(:, 2:2*n_aug+1) = Xaug_prev(:, 1:2*n_aug);  
    Xaug_prev(:, 1) = uPrev_aug;

    % Define matrix to hold the output of propagated sigma points
    Xaug = zeros(n,2*n_aug+1);

    % Propagate each sigma point through the nonlinear system dynamics
    for i=1:2*n_aug+1

        % Decompose the state vector into meaningful components
        x1 = Xaug_prev(1:3, i);
        x2 = Xaug_prev(4:6, i);
        x3 = Xaug_prev(7:9, i);
        x4 = Xaug_prev(10:12, i);
        x5 = Xaug_prev(13:15, i);
        ng = Xaug_prev(16:18, i);
        na = Xaug_prev(19:21, i);
        nbg = Xaug_prev(22:24, i);
        nba = Xaug_prev(25:27, i);



        phi = x2(3);
        theta = x2(2);
        psi = x2(1);

        % Compute the rotation matrix R using the ZYX Euler angles
        Rz = [cos(phi), -sin(phi), 0;
              sin(phi), cos(phi),  0;
              0,        0,         1];

        Ry = [cos(theta), 0, sin(theta);
              0,          1, 0;
              -sin(theta), 0, cos(theta)];

        Rx = [1, 0,         0;
              0, cos(psi), -sin(psi);
              0, sin(psi), cos(psi)];

        R = Rz * Ry * Rx;

        % Define the matrix T symbolically that relates angular veclocity to euler
        % angle derivatives
        T = [cos(theta)*cos(phi), -sin(phi), 0;
             cos(theta)*sin(phi), cos(phi),  0;
             -sin(theta),         0,         1];

        % Compute G = R' * T, where R' is the transpose of R
        G = R' * T;

        %disp(inv(G));

        % Define the gravity vector
        g = [0;0;-9.8];

        % Define the state transition function f using symbolic vectors

        f1 = x3;
        f2 = G\(angVel-x4-ng);
        f3 = g+R*(acc-x5-na);
        f4 = nbg;
        f5 = nba; 

        % Perform euler integration and find the estimated state vectors
        Xaug(:,i) = [x1 + (dt * f1); 
            x2 + (dt * f2);
            x3 + (dt * f3);
            x4 + (f4);
            x5 + (f5)];


    end

    % Compute weights for mean and covariance update
    w0_m = lambda/(lambda+n_aug);
    wi_m = 1/(2*(lambda+n_aug));


    % Compute the predicted state mean
    u_pred = zeros(n,1);

    for i=1:2*n_aug+1

            if i==1
                u_pred = u_pred+  w0_m*Xaug(:,i);
            else
                u_pred = u_pred + wi_m*Xaug(:,i);
              
            end

    end

    % Compute weights for covariance update
    w0_c = (lambda/(lambda+n_aug))+(1-alpha^2+beta);
    wi_c = 1/(2*(lambda+n_aug));

    % Compute the predicted state covariance
    covar_pred = zeros(n,n);

    for i=1:2*n_aug+1

            if i==1
                covar_pred = covar_pred+  w0_c*(Xaug(:,i)-u_pred)*(Xaug(:,i)-u_pred)';
            else
               covar_pred = covar_pred+  wi_c*(Xaug(:,i)-u_pred)*(Xaug(:,i)-u_pred)';
              
            end

    end

    % Return the estimated covariance and state
    covarEst = covar_pred;
    uEst = u_pred;

    
end