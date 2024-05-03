function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)


%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state

    % Measurement noise covariance matrix can be tuned
    R =  diag([0.002;0.03; 0.0009]);

    %R = 0.0002 * eye(3);

    % Simulate measurement noise (for each of three measurement components)
    vt = normrnd(0,(0.0005),[3,1]);

    % Get the number of state variables
    n = length(uEst);

    % Scaling parameters for Unscented Transform
    alpha = 0.001;
    k=1;
    beta = 2;
    lambda = (alpha^2) * (n+ k) - n;

    % Calculate square root of the covariance estimate
    P_sqrt = sqrtm(covarEst);

    % Initialize matrix for sigma points
    Xt = zeros(n,2*n+1);

    % Generate sigma points
    Xt(:,1) = uEst;

    for i = 1:n

        Xt(:,2*i) = uEst + sqrt(n+lambda) * P_sqrt(:,i);
        Xt(:,2*i+1) = uEst - sqrt(n+lambda) * P_sqrt(:,i);

    end


    % Find the Homogenous transform to go from camera frame to imu frame
    Rc2b = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1]; % eul2rotm([-pi/4,pi,0])
    Pc2b = [0.0283; -0.0283; 0.0300]; % eul2rotm([-pi/4,pi,0]) * [-0.04, 0.0, -0.03]';

    %Pc2b = Rc2b*(-Pc2b);

    % Calculate skew-symmetric matrix of Pc2b for cross product in matrix form
    Skew_Pb2c_in_body = [0,-Pc2b(3),Pc2b(2);Pc2b(3),0,-Pc2b(1);-Pc2b(2),Pc2b(1),0];

    % Initialize matrix for transformed sigma points in measurement space
    Zt = zeros(3,2*n+1);

    % Rotation matrix from Euler angles of the estimated state
    Rb2w = eul2rotm(uEst(4:6)');

    % Transform sigma points to measurement space
    for i=1:2*n+1

        %Rb2w = eul2rotm(Xt(4:6,i)');
        
        l_x2_x3 = Rb2w' * Xt(7:9,i);

        Zt(:,i) = Rc2b'*l_x2_x3 - Rc2b' * Skew_Pb2c_in_body * Rc2b * z_t(4:6,1) +vt;

    end

    % Weights for mean of the measurement prediction
    Wm_0 = lambda / (n + lambda);
    Wm_1 = 1 / (2*(n + lambda)); 

    % Calculate the estimated measurement mean
    Zut = zeros(3,1);
    for i = 1:((2*n)+1)

        if i==1
            Zut = Zut + (Wm_0 * Zt(:,i));
        else
            Zut = Zut + (Wm_1 * Zt(:,i));

        end
        
    end

    % Weights for the covariance of the measurement prediction

    Wc_0 = ((lambda) / (n + lambda)) + (1 - (alpha^2) + beta);
    Wc_1 = 1 / (2*(n + lambda)); 


    % Initialise covariance and cross-covariance matrices
    Ct = zeros(15,3);
    St = zeros(3,3);

    % Compute cross-covariance and covariance of the output of sigma points

    for i = 1:((2*n)+1)

        if i==1

            Ct = Ct + (Wc_0 * (Xt(:,i) - uEst) * (Zt(:,i) - Zut)');
            St = St + (Wc_0 * (Zt(:,i) - Zut) * (Zt(:,i) - Zut)');
        else
            Ct = Ct + (Wc_1 * (Xt(:,i) - uEst) * (Zt(:,i) - Zut)');
            St = St + (Wc_1 * (Zt(:,i) - Zut) * (Zt(:,i) - Zut)');


        end
        
    end

    % Add measurement noise covariance
    St = St + R;

    % Compute Kalman Gain
    K = Ct / St;

    % update current mean
    uCurr = uEst + K * (z_t(1:3) - Zut);
    % update current covariance
    covar_curr = covarEst - (K * St * K');

    
    
    

    
end

