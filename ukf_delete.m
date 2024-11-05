function [traj_max, traj_mean, xl_max, xl_mean, P_max, P_mean, traj_sample_iwmax] = ...
    ukf(dynModel, measModel, measurements, x0_nonLin, P0_lin, Q, R, dt, groundTruth)
% UKF - Run Unscented Kalman Filter
%
% Syntax:
%   [traj_max, traj_mean, xl_max, xl_mean, P_max, P_mean, traj_sample_iwmax] = 
%       ukf(dynModel, measModel, measurements, ...
%           x0_nonLin, P0_lin, Q, R, dt, groundTruth)
%
% In:
%   dynModel    - Dynamical model function handle
%   measModel   - Measurement model function handle
%   measurements - Observations [N_T x n_y]
%   x0_nonLin   - Initial non-linear state [nNonLin x 1]
%   P0_lin      - Initial state covariance [nNonLin x nNonLin]
%   Q           - Process noise covariance [nw x nw]
%   R           - Measurement noise covariance [n_y x n_y]
%   dt          - Time step [N_T x 1] or scalar
%
% Out:
%   traj_max    - Highest-weight trajectory (nonlinear)
%   traj_mean   - Weighted-mean trajectory (nonlinear) 
%   xl_max      - Map of highest weight particle
%   xl_mean     - Weighted mean map
%   P_max       - Covariance of highest weight particle
%   P_mean      - Weighted mean covariance
%   traj_sample_iwmax - Trajectory of particle with highest weight at last time instance

nNonLin = size(x0_nonLin, 1);
N_T = size(measurements, 1);

% Initialize states and covariance
x = x0_nonLin;
P = P0_lin;

traj_max = nan(nNonLin, N_T);
traj_mean = nan(nNonLin, N_T);

% UKF loop
for t = 1:N_T
    if (mod(t, round(N_T / 10)) == 0)
        disp(['t is ', num2str(t)]);
    end

    % Time Update (Prediction)
    [x, P] = ukf_predict(dynModel, x, P, Q, dt(t));

    % Measurement Update
    y = measurements(t, :)'; % Current measurement
    [x, P] = ukf_update(measModel, x, P, y, R);
    
    % Store trajectories
    traj_max(:, t) = x;  % Store current state
    traj_mean(:, t) = x; % For UKF, mean is the state itself
end

xl_max = x;  % Final state
xl_mean = x; % For UKF, mean is the state itself
P_max = P;   % Final covariance
P_mean = P;  % For UKF, mean covariance is the state covariance
traj_sample_iwmax = traj_max(:, N_T);  % Last state

end

function [x, P] = ukf_predict(dynModel, x, P, Q, dt)
    % Implement the UKF prediction step
    % Calculate sigma points, propagate through dynModel, and compute the mean and covariance
    % (Implementation details required)
    
    % Example (replace with actual UKF prediction logic):
    x = dynModel(x, dt, Q);
    P = P + Q; % Simplified update; adjust for actual model
end

function [x, P] = ukf_update(measModel, x, P, y, R)
    % Implement the UKF measurement update step
    % Calculate sigma points, propagate through measModel, compute the innovations, 
    % and update the state and covariance
    % (Implementation details required)
    
    % Example (replace with actual UKF update logic):
    yhat = measModel(x); % Predicted measurement
    innovation = y - yhat;
    P = P + R; % Update covariance; adjust for actual model
    x = x + innovation; % Simplified state update; adjust for actual model
end
