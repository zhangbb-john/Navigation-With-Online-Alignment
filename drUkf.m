
function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    drUkf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,Rs,dt, groundTruth)
disp('Performing drUKF ...');
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
		P = Q0;
	else
		[M, P] = ukf_predict1(M,P,dynModel,Q, dt, 1, 2, 0, 0, [iQuat, iOffset]);
		
		eigenvalues = eig(P);
		isNotPositiveDefinite = any(eigenvalues <= 1e-15); % Not PD if any eigenvalue is <= 0
		if (isNotPositiveDefinite)
			P = Q0;
		end 
		R1 = Rs(:, :, k); 
		idx = find(diag(R1) > 100);
		[M,P] = ukf_update1(M,P,measurements(k,:)',measModel,R1,[], 1, 2, 0, 0, [iQuat, iOffset], [iMeasEuler, iMeasDoa], idx);

		% if (mod(k * dt, 1 / freq_dvl) > 0.01)
		% 	R1 = R; idx = [iMeasVel,iMeasDoa, iMeasDoppler]; R1(idx, idx) = R1(idx, idx) * 1e6;
		% 	[M,P] = ukf_update1(M,P,measurements(k,:)',measModel,R1,[], 1, 2, 0, 0, [iQuat, iOffset], [iMeasEuler, iMeasDoa], idx);
		% else
		% 	R1 = R; idx = [iMeasDoa, iMeasDoppler]; R1(idx, idx) = R1(idx, idx) * 1e6;
		% 	[M,P] = ukf_update1(M,P,measurements(k,:)',measModel,R1,[], 1, 2, 0, 0, [iQuat, iOffset], [iMeasEuler, iMeasDoa], idx);
		% end
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
	traj_max(:,k)   = M;
	U_PP(:,:,k) = P;
	traj_std(:, k) = sqrt(diag(P));

end
traj_mean = traj_max;
P_mean = [] ;
traj_sample_iwmax = [];

end