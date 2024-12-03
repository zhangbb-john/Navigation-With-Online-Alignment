
function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    ekf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,dt, groundTruth, makeplots)
disp('Performing EKF ...');
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
freq_acoustic = 0.2;
freq_dvl = 1;
nNonLin = size(x0_nonLin,1);
N_T = size(measurements,1);% original Y is measurement * state_num

% Reserve space for estimates.
traj_max = zeros(nNonLin, N_T);
U_PP = zeros(nNonLin, nNonLin, N_T);

% Estimate with UKF
for k=1:N_T
	if (mod(k, round(N_T / 10)) == 0)
		disp(['time step k is ', num2str(k)])
	end
	if (k == 1)
		M = initialize(x0_nonLin, Q0 * 1e-10);
		M(iQuat) = measurements(k, iMeasEuler)';
		M(iVel) = measurements(k, iMeasVel)';
		P = Q0;
		% Call Jac_func with x_values and dt_value
		jac_pred_func = ekf_alignment_df_dx(@easyDynModel);
		jac_meas_func = ekf_alignment_dh_dx(@easyMeasModel);	
		if (makeplots)
			figure(iBeacon(end) + 3);
			idx = [iPos(1) iQuat(3) iVel(1) iOffset(3) iBeacon(1)];
			diag_std = sqrt(diag(P));
			for i = 1 : length(idx)
				subplot(length(idx), 1, i);
				plot(k - 0.7, diag_std(idx(i)), 'r*'); hold on;
			end
		end
	else
		A = jac_pred_func(M, dt);
% 		if (k < 3)
% 			Q1 = Q;
% 			Q1(iOffset, iOffset) = Q1(iOffset, iOffset) * 1e2;
% 			Q1(iBeacon, iBeacon) = Q1(iBeacon, iBeacon) * 1e2;
% 			[M, P] = ekf_predict1(M,P,A,Q1, @easyDynModel,dt);
% 		else
		[M, P] = ekf_predict1(M,P,A,Q, @easyDynModel,dt);
% 		end
		if (makeplots)
			figure(iBeacon(end) + 3);
			diag_std = sqrt(diag(P));
			for i = 1 : length(idx)
				subplot(length(idx), 1, i);
				plot(k - 0.3, diag_std(idx(i)), 'r*'); hold on;
			end
		end
		H = jac_meas_func(M);
		idx = [];
		if (mod(k * dt, 1 / freq_dvl) > 0.01)
			idx = [idx, iMeasVel]; 
		end
		if (mod(k * dt, 1 / freq_acoustic) > 0.01)
			idx = [idx, iMeasDoa, iMeasDoppler]; 
		end 
		R1 = R; R1(idx, idx) = R1(idx, idx) * 1e6;
		[M, P] = ekf_update1(M,P,measurements(k,:)', H, R1, @easyMeasModel, [iQuat, iOffset], [iMeasEuler, iMeasDoa]);
		if (makeplots)
			figure(iBeacon(end) + 3);
			diag_std = sqrt(diag(P));
			for i = 1 : length(idx)
				subplot(length(idx), 1, i); plot(k + 0.2, diag_std(idx(i)), 'r*'); hold on;
			end
			figure(iQuat(3));
			subplot(4, 1, 1); 
			plot_est_yaw = plot(k, M(iQuat(1), :), 'r.'); hold on;
			plot(k, measurements(k, iMeasEuler(1)), 'k+');
			legend('estimated roll', 'Measured roll'); 	title('Estimated roll ');
			subplot(4, 1, 2);
			plot_est_yaw = plot(k, M(iQuat(2), :), 'r.'); hold on;
			plot(k, measurements(k, iMeasEuler(2)), 'k+');
			legend('estimated pitch', 'Measured pitch'); title('Estimated pitch ');
			subplot(4, 1, 3);
			plot_est_yaw = plot(k, M(iQuat(3), :), 'r.'); hold on;
			plot(k, measurements(k, iMeasEuler(3)), 'k+');
			legend('estimated yaw', 'Measured yaw'); title('Estimated yaw ');
			subplot(4, 1, 4);
			NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
			plot(k, NormalizeAngle(M(iQuat(3)) - measurements(k, iMeasEuler(3))), 'ro'); hold on;
			title('Yaw error for each step');
		end
		eigenvalues = eig(P);
		isNotPositiveDefinite = any(eigenvalues <= 1e-15); % Not PD if any eigenvalue is <= 0
		if (isNotPositiveDefinite)
			P = Q0;
		end 
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
% 		plot3(traj_max(iPos(1), 1 : (k-1)), traj_max(iPos(2), 1 : (k-1)), traj_max(iPos(3), 1 : (k-1)), 'r-'); hold on;
% 		plot3(groundTruth.gt(iPos(1), 1 : (k-1)), groundTruth.gt(iPos(2), 1 : (k-1)), groundTruth.gt(iPos(3), 1 : (k-1)), 'g-');
	end
	traj_max(:,k)   = M;
	U_PP(:,:,k) = P;
	traj_std(:, k) = sqrt(diag(P));

end
traj_mean = traj_max;
P_mean = [] ;
traj_sample_iwmax = [];
function [xpred] = easyDynModel(xn, dt)
	iPos = 1 : 3;
	iQuat = 4 : 6;
	iVel = 7 : 9;
	iOffset = 10 : 12;
	iBeacon = 13 : 15;	
    % Predict through dynamic model. Also optionally output dQuat for
    % generating odometry data
	Rot = euler2rot(xn(iQuat));
%     xpred_pos = xn(iPos) + xn(iVel); 
	xpred_pos = xn(iPos) + Rot * dt * xn(iVel);
	xpred_vel = xn(iVel);

	xpred_attitude = xn(iQuat);
	xpred_offset= xn(iOffset);
	xpred_beacon = xn(iBeacon);		
    xpred = [xpred_pos; xpred_attitude; xpred_vel; xpred_offset; xpred_beacon]; 
end

function measurement = easyMeasModel(xn)
	iPos = 1 : 3;
	iQuat = 4 : 6;
	iVel = 7 : 9;
	iOffset = 10 : 12;
	iBeacon = 13 : 15;	
 	euler = xn(iQuat);
	World2Base  = euler2rot(euler);
	base2beaconInworld = xn(iBeacon) - xn(iPos);
	base2beaconInbase = World2Base' * base2beaconInworld;
	offset = xn(iOffset);
	Base2USBL = euler2rot(offset);
    base2beaconInUSBL = Base2USBL' * base2beaconInbase;   
    velocity = xn(iVel);
	measEuler = [euler(1); euler(2); euler(3)];

	measVel = [velocity(1); velocity(2); velocity(3)];
	measDepth = xn(iPos(end));
	measDoa = [atan2(base2beaconInUSBL(2), base2beaconInUSBL(1)); asin(base2beaconInUSBL(3) / norm(base2beaconInUSBL))];	
	measDoppler = [base2beaconInbase' * velocity / norm(base2beaconInbase); xn(iBeacon(end))]; 
	measurement = [measEuler; measVel; measDepth; measDoa; measDoppler];
end

end