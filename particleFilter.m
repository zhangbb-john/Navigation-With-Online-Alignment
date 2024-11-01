function [traj_max,traj_mean,xl_max,xl_mean,P_max,P_mean,traj_sample_iwmax] = ...
    particleFilter(dynModel,measModel,measurements,...
    x0_nonLin,Q,R,N_P,dt, groundTruth)
% N_P = 100 particle number
% measurements is 192-time measurements
% PARTICLEFILTER - Run Rao-Blackwellized particle filter
%
% Syntax:
%   [traj_max,traj_mean,xl_max,xl_mean,P_max,P_mean,traj_sample,xn_traj] = 
%       particleFilter(dynModel,measModel,measurements,...
%           x0_nonLin,x0_lin,P0_lin,Q,R,N_P,dt,sparseFeatures,makePlots)
%
% In:
%   dynModel    - Dynamical model function handle, xn = @(xn,dx,dt,Q)
%   measModel   - Measurement model function handle, measurements = @(xn,xl) + r
%   measurements           - Observations [N_T x n_y]
%   x0_nonLin   - Initial non-linear state [nNonLin x 1]
%   Q           - Process noise cov [nw x nw x N_T] or [nw x nw]
%   R           - Measurement noise cov [n_y x n_y]
%   N_P         - Number of particles
%   dt          - Time between two time steps [N_T x 1] or scalar
% Out:
%   traj_max    - Highest-weight trajectory (nonlinear)
%   traj_mean   - Weighted-mean trajectory (nonlinear) 
%   traj_sample_iwmax - Trajectory of particle with highest weight at t = N_T
%   xn_traj     - Trajectories of nonlinear states over time
%
% Description:
%   Run particle filter for a conditionally linear or conditionally
%   linearized state space model. See [1] for details.
%
% References:
%
%   [1] Manon Kok, Arno Solin, and Thomas B. Schon. Rao-Blackwellized 
%       Particle Smoothing for Simultaneous Localization and Mapping.
%       pre-print: https://arxiv.org/abs/2306.03953
%
% See also:
%   particleSmoother
%
% Copyright:
%   2023-   Manon Kok and Arno Solin
iPos = 1 : 3;
iQuat = 4 : 6;
iVel = 7 : 9;
iOffset = 10 : 12;
iBeacon = 13 : 15;
iMeasEuler = 1 : 3;
iMeasVel = 4 : 6;
iMeasDoa = 7 : 8;
iMeasDoppler = 9 : 10;
%% Initialise weights, states and covariance matrices

% Initial weights
w = 1/N_P * ones(1,N_P);
logw = log(w);
  
% Initial states (nonlinear) and initial means (linear states)
xn = repmat(x0_nonLin,1,N_P); % Nonlinear states  
% Initial covariance matrices for linear states

%% Parameters and settings
% Extract some parameters
nNonLin = size(x0_nonLin,1);
N_T = size(measurements,1);
  
% Allow for both time-varying and constant Q
if size(Q,3) == 1 
    Q = repmat(Q,[1 1 N_T-1]);
end
  
% Allow for both time-varying and constant time step
if length(dt) == 1 
    dt = dt * ones(N_T-1,1);
end
  

% Jitter to use if Cholesky decomposition fails due to numerical instability
jitter = 1e-3; 
  
%% Preallocate trajectories
traj_max = nan(nNonLin,N_T); % Maximum-weight trajectory
traj_mean = nan(nNonLin,N_T); % Weighted-mean trajectory
yhattraj = nan(size(measurements,2), N_T); % Predicted measurement by the maximum-weight particle

ai = zeros(N_P,1); % Sampled ancestors
    
%% Filter recursion
for t=1:N_T
    % Particle filter prediction
    xn_ = xn; % Copy old nonlinear states
    if t ~= 1 % Don't do a prediction at the very first time instance
        for i = 1:N_P
            % Draw ancestor index...
            ai(i) = sample(w); 
            % ... and propagate that nonlinear state through dynamics
			Qi = Q(:,:,t-1);
			Qi(iMeasVel, iMeasVel) = Qi(iMeasVel, iMeasVel);
            xn(:,i) = dynModel(xn_(:,ai(i)),dt(t-1),Q(:,:,t-1) ); 
% 			xn(iQuat, i) = groundTruth.gt(iQuat, t);
% 			xn(iOffset, i) = groundTruth.gt(iOffset, t);
% 			xn(iVel, i) = groundTruth.gt(iVel, t);
			xn(iOffset, i) = groundTruth.gt(iOffset, t);
			xn(iBeacon, i) = groundTruth.gt(iBeacon, t);
			
			if (mod(i, round(N_P / 2)) == 0 && mod(t, round(N_T / 20)) == 0)
% 				figure(20);
% 				subplot(2, 1, 1);
% 				plot(t, xn(iQuat(3),i), 'r.'); hold on;
% 				plot(t, groundTruth.gt(iQuat(3), t), 'g+'); hold on;
% 				title('particle yaw');
			end
		end
        % Save trajectory with shuffled ancestor indices, e.g. to visualise
    end
    
    % Compute the importance weights
    yt = measurements(t,:); % Measurements at time t
% 	if (mod(t, round(N_T / 20)) == 0)
% 		figure(20);
% 		subplot(2, 1, 2);
% 		plot(t, yt(iMeasEuler(3)), 'bo'); hold on;
% 		title('Yaw measurement');
% 	end
    for i=1:N_P 
		% Linearize measurement model
		Qmeas = eye(size(R, 1)) * 1e-20;
		[yhat] = measModel(xn(:,i), Qmeas);
		% Compute innovations and their covariances
		e = yt' - yhat;
		SS = R;
		% Strip away those that are not observed
		ind = ~isnan(yt);
		e = e(ind);
		NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
		e(iMeasEuler) = NormalizeAngle(e(iMeasEuler));
		if (length(e) > iMeasDoa(end))
			e(iMeasDoa) = NormalizeAngle(e(iMeasDoa));
		end
% 		figure(22);
% 		plot(e(1 : 8));
% 		title('error of measurement vs predicted');
		SS = SS(ind,ind);
        % Compute the log weights
        [cS,flag] = chol(SS,'lower');
        if flag>0
            cS = chol(SS+jitter*eye(size(SS,1)),'lower');
        end
        v = cS\e;
        logw(i) = -sum(log(diag(cS))) - .5*(v'*v) - .5*numel(e)*log(2*pi);
    end

    % Normalize by log-sum-exp trick
    c = max(logw);
    lse = c + log(sum(exp(logw - c)));
    w = exp(logw - lse);  
    
    % Store trajectories
    [~,iw_max] = max(w);
    traj_max(:, t) = xn(:, iw_max);   % Store maximum-weight particle
    traj_mean(:, t) = sum(xn.*w, 2);  % Store weighted-mean particle

end
xl_max = 0; xl_mean=0; P_max =0; P_mean =0;
%% Extract final map and trajectory
% Map of highest weight particle and its covariance
% xl_max = xl(:,iw_max);
% P_max = P(:,:,iw_max);
  
% Weighted mean map and covariance
% xl_mean = sum(xl.*w,2);
% P_mean = zeros(length(xl_mean));
% for i = 1:N_P
%     P_mean = w(i) * (P(:,:,i) + (xl_mean - xl(:,i)) * (xl_mean - xl(:,i))');
% end
  
% Trajectory of particle with highest weight at last time instance
traj_sample_iwmax = traj_max(:,t);

end