
function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    lsUkf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,dt, groundTruth)
disp('Performing LS-UKF ...');
iPos = 1 : 3;
iQuat = 4 : 6;
iVel = 7 : 9;
iOffset = 10 : 12;
iBeacon = 13 : 15;
iMeasEuler = 1 : 3;
iMeasVel = 4 : 6;
iMeasDoa = 7 : 8;
iMeasDoppler = 9 : 10;
freq_acoustic = 2;
nNonLin = size(x0_nonLin,1);
N_T = size(measurements,1);% original Y is measurement * state_num

% Reserve space for estimates.
traj_max = zeros(nNonLin, N_T);
U_PP = zeros(nNonLin, nNonLin, N_T);

nls_data = [];
% Estimate with UKF
flag_nls = true;
for k=1:N_T
	if (mod(k, round(N_T / 10)) == 0)
		disp(['time step k is ', num2str(k)])
	end
	if (k == 1)
		M = initialize(x0_nonLin, Q0 * 1e-10);
		P = Q0;
	else
		if (flag_nls)
			[M, P] = ukf_predict1(M,P,dynModel,Q, dt, 1, 2, 0, 0, [iQuat, iOffset]);
			R1 = R; idx = [iMeasDoa, iMeasDoppler]; R1(idx, idx) = R1(idx, idx) * 1e6;
			[M, P] = ukf_update1(M, P, measurements(k,:)', measModel, R1, [], 1, 2, 0, 0, [iQuat, iOffset], [iMeasEuler, iMeasDoa]);
			M([iOffset, iBeacon]) = x0_nonLin([iOffset, iBeacon]);
			P([iOffset, iBeacon], [iOffset, iBeacon]) = Q0([iOffset, iBeacon], [iOffset, iBeacon]);
			eigenvalues = eig(P);
			isNotPositiveDefinite = any(eigenvalues <= 1e-15); % Not PD if any eigenvalue is <= 0
			if (isNotPositiveDefinite)
				P = Q0;
			end 
			if (mod(k * dt, 1 / freq_acoustic) < 0.01)
				nls_data = [nls_data, [M(iPos(1): iVel(end)); measurements(k, [iMeasDoa, iMeasDoppler])']];
			end
		
			if (size(nls_data, 2) / freq_acoustic > 300)
				dr_error = nls_data(iPos, end) - groundTruth.gt(iPos, k);
				nls_pos_err = norm(dr_error);
				disp(['nls final pos err of dead reckoning is ', num2str(nls_pos_err), ' meter']);	
				figure(iQuat(2) * 10);
				subplot(3, 2, 1); plot(1 : (k-1), traj_max(iQuat(1), 1 : (k-1)), 'r.'); hold on;
				subplot(3, 2, 3); plot(1 : (k-1), traj_max(iQuat(2), 1 : (k-1)), 'r.'); hold on;
				subplot(3, 2, 5); plot(1 : (k-1), traj_max(iQuat(3), 1 : (k-1)), 'r.'); hold on;
				NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
				subplot(3, 2, 2); plot(1 : (k-1), NormalizeAngle(traj_max(iQuat(1), 1 : (k-1)) - groundTruth.gt(iQuat(1), 1 : (k-1))), 'r.'); hold on;
				subplot(3, 2, 4); plot(1 : (k-1), NormalizeAngle(traj_max(iQuat(2), 1 : (k-1)) - groundTruth.gt(iQuat(2), 1 : (k-1))), 'r.'); hold on;
				subplot(3, 2, 6); plot(1 : (k-1), NormalizeAngle(traj_max(iQuat(3), 1 : (k-1)) - groundTruth.gt(iQuat(3), 1 : (k-1))), 'r.'); hold on;	
				sgtitle('euler of nls stage');
				figure(iVel(2) * 10);
				subplot(3, 2, 1); plot(1 : (k-1), traj_max(iVel(1), 1 : (k-1)), 'r.'); hold on;
				subplot(3, 2, 3); plot(1 : (k-1), traj_max(iVel(2), 1 : (k-1)), 'r.'); hold on;
				subplot(3, 2, 5); plot(1 : (k-1), traj_max(iVel(3), 1 : (k-1)), 'r.'); hold on;
				subplot(3, 2, 2); plot(1 : (k-1), (traj_max(iVel(1), 1 : (k-1)) - groundTruth.gt(iVel(1), 1 : (k-1))), 'r.'); hold on;
				subplot(3, 2, 4); plot(1 : (k-1), (traj_max(iVel(2), 1 : (k-1)) - groundTruth.gt(iVel(2), 1 : (k-1))), 'r.'); hold on;
				subplot(3, 2, 6); plot(1 : (k-1), (traj_max(iVel(3), 1 : (k-1)) - groundTruth.gt(iVel(3), 1 : (k-1))), 'r.'); hold on;	
				sgtitle('velocity of nls stage');
				figure(iPos(2)* 10);
				plot(nls_data(iPos(1), :), nls_data(iPos(2), :), 'r.'); hold on; 
				plot(-50, 20, 'b+'); hold on;
				plot(groundTruth.gt(iPos(1), 1:(k-1)), groundTruth.gt(iPos(2), 1 : (k-1)), 'k.'); hold on;
				title('Trajectory');
				param = nlsSolver(nls_data);
				disp('estimated offset is '); 
				disp(param(1: 3)')
				disp('estimated beacon is '); 
				disp(param(4 : 6)')
				flag_nls = false;
				M(iOffset) = param(1: 3);
				M(iBeacon) = param(4 : 6);
			end			
		else
			[M, P] = ukf_predict1(M,P,dynModel,Q, dt, 1, 2, 0, 0, [iQuat, iOffset]);
		
			eigenvalues = eig(P);
			isNotPositiveDefinite = any(eigenvalues <= 1e-15); % Not PD if any eigenvalue is <= 0
			if (isNotPositiveDefinite)
				P = Q0;
			end 
			if (mod(k * dt, 1 / freq_acoustic) > 0.01)
				R1 = R; idx = [iMeasDoa, iMeasDoppler]; R1(idx, idx) = R1(idx, idx) * 1e6;
			else
				R1 = R;
			end
				
			[M,P] = ukf_update1(M,P,measurements(k,:)',measModel,R1,[], 1, 2, 0, 0, [iQuat, iOffset], [iMeasEuler, iMeasDoa]);
			eigenvalues = eig(P);
			isNotPositiveDefinite = any(eigenvalues <= 1e-15); % Not PD if any eigenvalue is <= 0
			if (isNotPositiveDefinite)
				figure(iBeacon(end) + 1);
				idx = [iPos(1) iQuat(3) iVel(1) iOffset(3) iBeacon(1)];
				for i = 1 : length(idx)
					subplot(length(idx), 1, i);
					plot(traj_std(idx(i), 1 : (k - 1)), 'k');
				end
				figure(50);
				plot(traj_max(iPos(1), 1 : (k - 1)), 'r.'); hold on;
				plot(traj_max(iPos(2), 1 : (k - 1)), 'g.'); hold on;	
				plot(traj_max(iPos(3), 1 : (k - 1)), 'b.'); hold on;
				for kk = 1 : 5
					min_diag_value = 1e-6; % Minimum desired value for diagonals
					diag(max(min_diag_value - diag(P), 0));
					P = P + diag(max(min_diag_value - diag(P), 0));
				end
	% 			P = Q0;
			end 
	% 		figure(51)
	% 		plot3(traj_max(iPos(1), 1 : (k-1)), traj_max(iPos(2), 1 : (k-1)), traj_max(iPos(3), 1 : (k-1)), 'r-');
		end
	end
	traj_max(:,k)   = M;
	U_PP(:,:,k) = P;
	traj_std(:, k) = sqrt(diag(P));

end
traj_mean = traj_max;
P_mean = [] ;
traj_sample_iwmax = [];
function param = nlsSolver(nls_data)
% Set options to use Levenberg-Marquardt algorithm
% Set options to use Levenberg-Marquardt algorithm

objective = @(param) residualModel(param, nls_data);

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
param0 = [0 0 0 0 0 0]';
% Run the optimization
[param, resnorm] = lsqnonlin(objective, param0, [], [], options); 

end

function residuals = residualModel(params, data)
	iPos = 1 : 3;
	iQuat = 4 : 6;
	iVel = 7 : 9;
	iOffset = 10 : 12;
	iBeacon = 13 : 15;
	iMeasEuler = 1 : 3;
	iMeasVel = 4 : 6;
	iMeasDoa = 7 : 8;
	iMeasDoppler = 9 : 10;	 
	residuals  = [];
	for j = 1 : size(data, 2)
		xn = [data([iPos, iQuat, iVel], j); params];
		euler = xn(iQuat);
		World2Base  = euler2rot(euler);
		base2beaconInworld = xn(iBeacon) - xn(iPos);
		base2beaconInbase = World2Base' * base2beaconInworld;
		offset = xn(iOffset);
		Base2USBL = euler2rot(offset);
		base2beaconInUSBL = Base2USBL' * base2beaconInbase;   
		velocity = xn(iVel);
		measurement = data((iVel(end) + 1): (iVel(end) + 4), j);

		pred_doa = [atan2(base2beaconInUSBL(2), base2beaconInUSBL(1)); asin(base2beaconInUSBL(3) / norm(base2beaconInUSBL))];	
		pred_doppler = [base2beaconInbase' * velocity / norm(base2beaconInbase); xn(iBeacon(end))]; 
		pred = [pred_doa; pred_doppler];
		res = pred - measurement;
		NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
		res(1 : 2) = NormalizeAngle(res(1 : 2));
		res = diag([0.01, 0.01, 0.1, 1]) \ res;
		residuals  = [residuals, res];
	end
end


end