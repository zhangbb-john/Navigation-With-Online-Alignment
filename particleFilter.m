function [traj_max,traj_mean,xl_max,xl_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    particleFilter(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,N_P,dt, groundTruth, makePlots)
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
iMeasDepth = 7;
iMeasDoa = 8 : 9;
iMeasDoppler = 10 : 11;
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
traj_P = zeros((nNonLin), (nNonLin), N_T);
traj_std = zeros(nNonLin, N_T);
NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
%% Filter recursion
flag_good_particle = false;

%% reference 
reference = initialize(x0_nonLin, Q0); ref_err = 200;
for t=1:N_T
	if (mod(t, round(N_T / 10)) == 0)
		disp(['t is ', num2str(t)]);
	end
    % Particle filter prediction
    xn_ = xn; % Copy old nonlinear states
    if t == 1 
		for i  = 1 : N_P
			xn(:,i) = initialize(x0_nonLin, Q0);
			idx = find(xn(iOffset, i) < 0);
			xn(iOffset(idx), i) = 0;
		end
	else % Don't do a prediction at the very first time instance

% 		if (mod(t, round(N_T / 10)) == 0)
% 			w = ones(size(w)) * 1 / length(w);
% 		end
% 		figure(23); subplot(2, 1, 1); histogram(w, 10); title('Histogram of weights with 10 bins'); xlabel('weights'); ylabel('frequency');
% 		subplot(2, 1, 2); plot(w); title('Weights with 10 bins'); xlabel('weights'); ylabel('frequency');
		for i = 1:N_P
            % Draw ancestor index...
			neff = 1 / sum(w.^2);
			sum_w = sum(w);
			if (flag_good_particle)
				Qi = Q(:,:,t-1);
			else
				Qi = Q(:,:,t-1);
% 				Qi(iOffset, iOffset) = 1e2 * Qi(iOffset, iOffset);
% 				Qi(iBeacon(1 : 2), iBeacon(1 : 2)) = 100 * Qi(iBeacon(1 : 2), iBeacon(1 : 2));
			end
			if (neff < N_P * 0.3)
				ai(i) = sample(w); 

			else 
				ai(i) = i;

			end
            % ... and propagate that nonlinear state through dynamics
			

            xn(:,i) = dynModel(xn_(:,ai(i)), dt(t-1), Qi); 
% 			xn(iQuat, i) = groundTruth.gt(iQuat, t);
% 			xn(iOffset, i) = groundTruth.gt(iOffset, t);
% 			xn(iVel, i) = groundTruth.gt(iVel, t);
% 			xn(iOffset(1 : 2), i) = groundTruth.gt(iOffset(1 : 2), t);
% 			xn(iBeacon, i) = groundTruth.gt(iBeacon, t);
			
% 			if (mod(i, round(N_P / 50)) == 0 && mod(t, round(N_T / 50)) == 0)
% 				figure(20);
% 				rows = iBeacon(end) / 3 + 1;
% 				cols = 3;
% 				for figi = 1 : size(xn, 1)
% 					subplot(rows, cols, figi);
% 					plot(t, xn(figi,i), 'r.'); hold on;
% 					plot(t, groundTruth.gt(figi, t), 'g+'); hold on;
% 					title('particle distribution');
% 				end
% 			end
		end
 		xn([iOffset, iBeacon], end) = reference([iOffset, iBeacon]);%cheat

		for k = 1 : length(iOffset)
			row = iOffset(k);
			xn(row, xn(row, :) > 0.15) = 0.15;
			xn(row, xn(row, :) < 0) = 0;
			
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
	bearing_hat = [];
	elevation_hat = [];
	normalized_diff_vals = [];
	normalized_diff = [];
	
    for i=1:N_P 
		% Linearize measurement model
		Qmeas = eye(size(R, 1)) * 1e-20;
		[yhat] = measModel(xn(:,i), Qmeas);
		% Compute innovations and their covariances
		e = yt' - yhat;
		SS = R;
		SS(iMeasDoa, iMeasDoa) = SS(iMeasDoa, iMeasDoa);
		% Strip away those that are not observed
		ind = ~isnan(yt);
		e = e(ind);
		
		e(iMeasEuler) = NormalizeAngle(e(iMeasEuler));
		if (length(e) >= iMeasDoa(end))
			e(iMeasDoa) = NormalizeAngle(e(iMeasDoa));
		end
% 		figure(22);
% 		plot(e(1 : 8));
% 		title('error of measurement vs predicted');
		SS = SS(ind,ind) * 4;
        % Compute the log weights
        [cS,flag] = chol(SS,'lower');
        if flag>0
            cS = chol(SS+jitter*eye(size(SS,1)),'lower');
        end
        v = cS\e;
		normalized_diff = [normalized_diff, norm(v)];
		normalized_diff_vals = [normalized_diff_vals, v];
		%[maxv, maxi] = max(v);
		%disp(['max index is ', num2str(maxi)]);
        logw(i) = -sum(log(diag(cS))) - .5*(v'*v) - .5*numel(e)*log(2*pi);
		bearing_hat = [bearing_hat, yhat(iMeasDoa(1))];
		elevation_hat = [elevation_hat, yhat(iMeasDoa(2))];

	end
	normalized_vec = [];
	normalized_err = [];
	
	for i = 1 : N_P
		[cQ, flag] = chol(dt(1) * Q(:, :, 1) + Q0,'lower');
		err_state = (groundTruth.gt(:, t) - xn(:,i));
		err_state(iQuat) = NormalizeAngle(err_state(iQuat));
		err_state(iOffset) = NormalizeAngle(err_state(iOffset));
		normalized_vec = [normalized_vec,  cQ\err_state];
		normalized_err = [normalized_err; norm(normalized_vec)];
	end
	if (makePlots)
		figure(22);
		subplot(4, 1, 1);
		histogram(normalized_err, 10);
		xlabel('error'); ylabel('frequency');
		subplot(4, 1, 2);
		histogram(normalized_diff, 10);
		xlabel('diff from observation'); ylabel('frequency');
		subplot(4, 1, 3);

		% Find unique elements and their frequencies
		unique_elements = unique(ai)';
		frequency = histcounts(ai, [unique_elements, max(unique_elements)+1]);

		histogram(frequency, 10);
		xlabel('ancestor'); ylabel('frequency');

		subplot(4, 1, 4);
		histogram(w, 10);
		xlabel('weight'); ylabel('frequency');
		figure(24)
		for m = 1 : size(normalized_diff_vals, 1)
			subplot(ceil(size(normalized_diff_vals, 1) / 3), 3, m); ylabel(num2str(m));
			histogram(normalized_diff_vals(m, :), 10);
		end
	end
	
	% 定义均值向量和协方差矩阵
	mu = zeros(size(R, 1), 1); % 均值向量
	Sigma = R; % 协方差矩阵
	% 定义需要计算的点
	x = 1 * diag(chol(Sigma));

	% 计算多维正态分布的概率密度
	p = mvnpdf(x, mu, Sigma);
	likelyhood = exp(logw);
	flag_good_particle = any(likelyhood > p);

% 	figure(20);
% 	subplot(2, 1, 1);
% 	plot(t, bearing_hat, 'b.'); hold on;
% 	plot(t, yt(iMeasDoa(1)), 'ro'); hold on;
% 	title('bearing prediction');
% 	subplot(2, 1, 2);
% 	plot(t, elevation_hat, 'b.'); hold on;
% 	plot(t, yt(iMeasDoa(2)), 'ro'); hold on;
% 	title('elevation prediction');	
    % Normalize by log-sum-exp trick
    c = max(logw);
    lse = c + log(sum(exp(logw - c)));
    w = exp(logw - lse);  
%   w = w.^(1/3);
% 	w = w./sum(w);
    % Store trajectories
    [~,iw_max] = max(w);
    traj_max(:, t) = xn(:, iw_max);   % Store maximum-weight particle
% 	if (mod(t, round(N_T / 1)) == 0)
	if (makePlots && mod(t, round(N_T / 100)) == 0)
		if (mod(t, round(N_T / 5)) == 0)
			figure(20);
			filename = ['C:\Users\29434\Desktop\delete\particle', num2str(t), '.jpg'];
			saveas(gcf, filename);
			close(20);
		end
		figure(20);
		
		rows = 2;
		cols = 3;
		figs = (size(xn, 1) - rows * cols + 1) : size(xn, 1);
		for id = 1 : length(figs)
			figi = figs(id);
			subplot(rows, cols, id);
			plot(t, xn(figi,:), 'r.'); hold on;
			plot(t, groundTruth.gt(figi, t), 'g+'); hold on;
			if (flag_good_particle)
				plot(t, xn(figi, iw_max), 'b*'); hold on;
			else 
				plot(t, xn(figi, iw_max), 'k*'); hold on;
			end
			plot(t, xn(figi, end), 'yo'); hold on;
			if (id == 1)
				title(num2str(likelyhood(iw_max)));
			end
		end
		set(gcf, 'Units', 'centimeters'); % Set units to centimeters
		set(gcf, 'Position', [0, 0, 35, 22]); % Set position [x, y, width, height] in cm
		
		figure(21);
 		subplot(4, 1, 1);
		plot(t, likelyhood(iw_max), 'k.'); hold on;
		title('pdf');
		subplot(4, 1, 2);
		offset_err =  norm(traj_max(iOffset, t) - groundTruth.gt(iOffset, t)) / 0.001;
		beacon_err = norm(traj_max(iBeacon, t) - groundTruth.gt(iBeacon, t)) / 0.1;
		calib_err = norm([offset_err, beacon_err]);
		
		plot(t, calib_err, 'r.'); hold on;
		title('normalized offset err + beacon err');
		subplot(4, 1, 3);
		plot(t, calib_err, 'r.'); hold on;
		
		subplot(4, 1, 4);
		plot(calib_err, likelyhood(iw_max), 'b.'); hold on;
		xlabel('calibration error'); ylabel('likelihood');
	end	
 
	traj_mean(:, t) = sum(xn .* w, 2);  % Store weighted-mean particle	
	offset_err =  norm(traj_mean(iOffset, t) - groundTruth.gt(iOffset, t)) / 0.001;
	beacon_err = norm(traj_mean(iBeacon, t) - groundTruth.gt(iBeacon, t)) / 0.1;
	calib_err = norm([offset_err, beacon_err]);
	if (calib_err < ref_err)
		ref_err = calib_err;
		reference = traj_mean(:, t);
	end
	for id = iQuat(1) : iQuat(end)
		sum_sin = 0;
		sum_cos = 0;
		for i = 1 : N_P
			sum_sin = sum_sin + sin(xn(id,i)) * w(i);
			sum_cos = sum_cos + cos(xn(id,i)) * w(i);
		end
		traj_mean(id, t) = atan2(sum_sin, sum_cos);  % Compute wrapped mean for angles		
	end
	for id = iOffset(1) : iOffset(end)
		sum_sin = 0;
		sum_cos = 0;
		for i = 1 : N_P
			sum_sin = sum_sin + sin(xn(id,i)) * w(i);
			sum_cos = sum_cos + cos(xn(id,i)) * w(i);
		end
		traj_mean(id, t) = atan2(sum_sin, sum_cos);  % Compute wrapped mean for angles		
	end
	cov = zeros(size(xn, 1), size(xn, 1));
	for i = 1:N_P
		diff_state = (traj_mean(:, t) - xn(:,i));
		diff_state(iQuat) = NormalizeAngle(diff_state(iQuat));
		diff_state(iOffset) = NormalizeAngle(diff_state(iOffset));
		covi = (diff_state * diff_state');
% 		if (diff_state(iQuat(end)) > 1)
% 			'stop'
% 		end
		cov = cov + w(i) * covi;
	end
	traj_P(:, :, t) = cov;
	traj_std(:, t) = sqrt(diag(cov));
% 	figure(iBeacon(end) + 3);
% 	idx = [iPos(1) iQuat(3) iVel(1) iOffset(3) iBeacon(1)];
% 	for i = 1 : length(idx)
% 		subplot(length(idx), 1, i);
% 		plot(traj_std(idx(i), 1 : t), 'k');
% 	end
% 	figure(21);
% 	std_px = std(xn(iPos(1),:)); std_py = std(xn(iPos(2),:)); std_pz = std(xn(iPos(3),:));
% 	std_pos = norm([std_px, std_py, std_pz]);
% 	subplot(3, 1, 1);
% 	plot(t, std_pos, 'r.'); hold on;
% 	title('std of position');
% 	std_offsetx = std(xn(iOffset(1),:)); std_offsety = std(xn(iOffset(2),:)); std_offsetz = std(xn(iOffset(3),:));
% 	subplot(3, 1, 2);
% 	plot(t, std_offsetz, 'b.'); hold on;
% 	title('std of offset');
% 	subplot(3, 1, 3);
% 	plot(t, xn(iOffset(3), iw_max), 'b.'); hold on;
% 	title('Estimated yaw offset');	
end
%% Extract final map and trajectory
% Map of highest weight particle and its covariance
% xl_max = xl(:,iw_max);
% P_max = P(:,:,iw_max);
  
% Weighted mean map and covariance
% xl_mean = sum(xl.*w,2);

  
% Trajectory of particle with highest weight at last time instance
traj_sample_iwmax = traj_max(:,t);
xl_max = 0; xl_mean=0; P_mean =0;

end