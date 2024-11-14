close all;
clear;
originalPath = path;
addpath(genpath('./.'))
global iPos iQuat iVel iOffset iBeacon;
global iMeasEuler iMeasVel iMeasDoa iMeasDoppler;
iPos = 1 : 3;
iQuat = 4 : 6;
iVel = 7 : 9;
iOffset = 10 : 12;
iBeacon = 13 : 15;

iMeasEuler = 1 : 3;
iMeasVel = 4 : 6;
iMeasDoa = 7 : 8;
iMeasDoppler = 9 : 10;
% x = [px, py, pz, eulerx, eulery, eulerz, anglex, angley, anglez, beaconx, beacony, beaconz];
folder = './';

flag = 0; %0 simulation; 1 read data;
global debug_i;
debug_i = 0;

offset_errs = [];
pos_errs = [];
pos_final_errs = [];
beacon_errs = [];
filter = 'ekf';%e.g., ukf, pf, ekf
makeplots = false;
for trial = 1 : 1
tic;
pause(5);
close all;
[x0, measurements, x_true, params] = getMeas(flag, @dynModel, @measModel, filter, folder);
x0_nonLin = x0;
N_P = 100; % Number of particles

switch filter
	case 'ukf'
		[traj_max, traj_mean, traj_std, P_mean, traj_sample_iwmax] = ...
			ukf(@initialize, @dynModel, @measModel, measurements,...
			x0_nonLin, params.Q0, params.Qprocess, params.Qmeas, params.dt, x_true);
	case 'pf'
		[traj_max, traj_mean, xl_max, xl_mean, traj_std, P_mean, traj_sample_iwmax] = ...
			particleFilter(@initialize, @dynModel, @measModel, measurements,...
			x0_nonLin, params.Q0, params.Qprocess, params.Qmeas, N_P, params.dt, x_true, makeplots);		
		
	case 'ekf'
		[traj_max, traj_mean, traj_std, P_mean, traj_sample_iwmax] = ...
			ekf(@initialize, @dynModel, @measModel, measurements,...
			x0_nonLin, params.Q0, params.Qprocess, params.Qmeas, params.dt, x_true, makeplots);
end
figure(iOffset(1));
plot(traj_mean(iOffset(1), :), 'r-'); hold on;
plot(traj_mean(iOffset(2), :), 'g-'); hold on;
plot(traj_mean(iOffset(3), :), 'b-'); hold on;
title('Offset');
figure(iBeacon(1));
plot(traj_mean(iBeacon(1), :), 'r-'); hold on;
plot(traj_mean(iBeacon(2), :), 'g-'); hold on;
plot(traj_mean(iBeacon(3), :), 'b-'); hold on;
title('Beacon');
figure(iPos(1));
plot(traj_mean(iPos(1), :), traj_mean(iPos(2), :), 'r-'); hold on;
title('Trajectory');
xlabel('East [m]'); ylabel('North [m]');
dpos = diff(x_true.gt(iPos, :)');
dist = sum(sqrt(dpos(:, 1).^2 + dpos(:, 2).^2 + dpos(:, 3).^2));
rmse_pos_pf = norm(rms(x_true.gt(iPos, :)' - traj_max(iPos, :)'));
disp(['rmse is ', num2str(rmse_pos_pf), '; dist is ', num2str(dist)]);
if (size(measurements, 2) > iMeasVel(3))
	figure(iVel(1));
	subplot(3,1,1);
	plot(measurements(:, iMeasVel(1)), 'k'); hold on;
	plot(traj_mean(iVel(1), :), 'r-'); hold on;
	legend('Measured velocity', 'Estimated velocity state');
	title('Estimated velocity ');

	subplot(3,1,2);
	plot(measurements(:, iMeasVel(2)), 'k'); hold on;
	plot(traj_mean(iVel(2), :), 'g-'); hold on;

	subplot(3,1,3);
	plot(measurements(:, iMeasVel(3)), 'k'); hold on;
	plot(traj_mean(iVel(3), :), 'b-'); hold on;
end
figure(iQuat(1));
subplot(4, 1, 1);
plot_est_yaw = plot(traj_mean(iQuat(1), :), 'r-'); hold on;
plot(measurements(:, iMeasEuler(1)), 'k');
legend('estimated roll', 'Measured roll');
title('Estimated roll ');
subplot(4, 1, 2);
plot_est_yaw = plot(traj_mean(iQuat(2), :), 'r-'); hold on;
plot(measurements(:, iMeasEuler(2)), 'k');
legend('estimated pitch', 'Measured pitch');
title('Estimated pitch ');
subplot(4, 1, 3);
plot_est_yaw = plot(traj_mean(iQuat(3), :), 'r-'); hold on;
plot(measurements(:, iMeasEuler(3)), 'k');
legend('estimated yaw', 'Measured yaw');
title('Estimated yaw ');
subplot(4, 1, 4);
NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;

plot(NormalizeAngle(traj_mean(iQuat(3), :) - measurements(:, iMeasEuler(3))'), 'r-'); hold on;
title('Yaw error');
figure(iPos(2));
subplot(3,1,1); 
plot(traj_mean(iPos(1), :), 'r-'); hold on;
plot(x_true.gt(iPos(1), :), 'g-');
title('x position [m]');
subplot(3,1,2); 
plot(traj_mean(iPos(2), :), 'r-'); hold on;
plot(x_true.gt(iPos(2), :), 'g-');
title('y position [m]');

subplot(3,1,3); 
plot(sqrt((traj_mean(iPos(1), :) - x_true.gt(iPos(1), :)).^2 + (traj_mean(iPos(2), :) - x_true.gt(iPos(2), :)).^2) , 'r-'); hold on;
title('Error [m]')
figure(iPos(3));
plot3(traj_mean(iPos(1), :), traj_mean(iPos(2), :), traj_mean(iPos(3), :), 'r-'); hold on;
plot3(x_true.gt(iPos(1), :), x_true.gt(iPos(2), :), x_true.gt(iPos(3), :), 'g-');
title('trajectory');

figure(iBeacon(end) + 2);
idx = [iPos(1) iQuat(3) iVel(1) iOffset(3) iBeacon(1)];
for i = 1 : length(idx)
	subplot(length(idx), 1, i);
	plotBoxplot(traj_std(idx(i), :));
end
% for i = 1 : length(x_true.gt(iPos(1), :))
% 	figure(25)
% 	plot(x_true.gt(iPos(1), i) + rand * 1, x_true.gt(iPos(2), i) + rand *1, 'r.'); hold on;
% 	if (mod(i, 100) == 0)
% 		text(x_true.gt(iPos(1), i) + rand * 1, x_true.gt(iPos(2), i), ['i = ', num2str(i)]);
% 	end
% 	title(['i = ', num2str(i)]);
% 	pause(0.01);
% 	
% end
offset_errs = [offset_errs; norm([traj_mean(iOffset, end) - x_true.gt(iOffset, end)])];
beacon_errs = [beacon_errs; norm([traj_mean(iBeacon, end) - x_true.gt(iBeacon, end)])];
pos_errs = [pos_errs; rmse_pos_pf];
pos_final_errs = [pos_final_errs; sqrt((traj_mean(iPos(1), end) - x_true.gt(iPos(1), end)).^2 + (traj_mean(iPos(2), end) - x_true.gt(iPos(2), end)).^2)];
pause(1);
disp([num2str(trial), '-th trial takes ', num2str(toc), ' seconds']);
end
rms(offset_errs)
disp(['RMSE of offset is ', num2str(rms(offset_errs))]);
disp(['RMSE of beacon position is ', num2str(rms(beacon_errs))]);
disp(['RMS of position rmse for multiple trials is ', num2str(rms(pos_errs))]);
disp(['RMS of final position error for multiple trials is ', num2str(rms(pos_final_errs))]);
path(originalPath);

function [xpred] = initialize(xn, Q)
	global iPos iQuat iVel iOffset iBeacon debug_i;
	xpred_pos = xn(iPos) + chol(Q(iPos, iPos),'lower') * randn(length(iPos),1);
	xpred_vel = xn(iVel) + chol(Q(iVel, iVel),'lower') * randn(length(iVel),1);
    xpred_attitude = xn(iQuat) + (chol(Q(iQuat, iQuat),'lower') * randn(length(iQuat),1));
	xpred_offset= xn(iOffset) + (chol(Q(iOffset, iOffset),'lower') * randn(length(iOffset),1));
	xpred_beacon = xn(iBeacon) +  (chol(Q(iBeacon, iBeacon),'lower') * randn(length(iBeacon),1));
	NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
	xpred_attitude = NormalizeAngle(xpred_attitude);
	xpred_offset = NormalizeAngle(xpred_offset);
    xpred = [xpred_pos; xpred_attitude; xpred_vel; xpred_offset; xpred_beacon]; 	
end
function [xpred] = dynModel(xn, dt, Q)
	
	global iPos iQuat iVel iOffset iBeacon debug_i;

    % Predict through dynamic model. Also optionally output dQuat for
    % generating odometry data
	R = euler2rot(xn(iQuat));
%     xpred_pos = xn(iPos) + xn(iVel); 
	if nargin == 2 
		xpred_pos = xn(iPos) + R * dt * xn(iVel);
		xpred_vel = xn(iVel);

		xpred_attitude = xn(iQuat);
		xpred_offset= xn(iOffset);
		xpred_beacon = xn(iBeacon);		
	else 
		xpred_pos = xn(iPos) + R * dt * xn(iVel) + chol(dt * Q(iPos, iPos),'lower') * randn(3,1);

	% 	figure(11)
	% 	plot(xpred_pos(1), xpred_pos(2), 'r.'); hold on;

		xpred_vel = xn(iVel) + chol(dt * Q(iVel, iVel),'lower') * randn(3,1);
	% 	figure(12);
		debug_i = debug_i + 1;
	% 	plot(debug_i, xpred_vel(1), 'r.'); hold on;
	% 	title('xpred_vel 1');
		xpred_attitude = xn(iQuat) + (chol(dt * Q(iQuat, iQuat),'lower') * randn(3,1));
		xpred_offset= xn(iOffset) + (chol(dt * Q(iOffset, iOffset),'lower') * randn(3,1));
		xpred_beacon = xn(iBeacon) +  (chol(dt * Q(iBeacon, iBeacon),'lower') * randn(3,1));
	end
	NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
	xpred_attitude = NormalizeAngle(xpred_attitude);
	xpred_offset = NormalizeAngle(xpred_offset);
	
    xpred = [xpred_pos; xpred_attitude; xpred_vel; xpred_offset; xpred_beacon]; 
end
function measurement = measModel(xn, Q)
% 	disp("input xn: column vector");
% 	disp("output measurement: 10 * 1 vector");
	global iPos iQuat iVel iOffset iBeacon;
	global iMeasEuler iMeasVel iMeasDoa iMeasDoppler;

 	euler = xn(iQuat);
	World2Base  = euler2rot(euler);
	base2beaconInworld = xn(iBeacon) - xn(iPos);
	base2beaconInbase = World2Base' * base2beaconInworld;
	offset = xn(iOffset);
	Base2USBL = euler2rot(offset);
    base2beaconInUSBL = Base2USBL' * base2beaconInbase;   
    velocity = xn(iVel);
	measurement = zeros(10, 1);
	if size(Q, 1) == 0
		measurement(iMeasEuler) = [euler(1); euler(2); euler(3)];

		measurement(iMeasVel) = [velocity(1); velocity(2); velocity(3)];
		measurement(iMeasDoa) = [atan2(base2beaconInUSBL(2), base2beaconInUSBL(1)); asin(base2beaconInUSBL(3) / norm(base2beaconInUSBL))];	
		measurement(iMeasDoppler) = [base2beaconInbase' * velocity / norm(base2beaconInbase); xn(iBeacon(end))]; 
	else 
		measurement(iMeasEuler) = [euler(1); euler(2); euler(3)] + chol(Q(iMeasEuler, iMeasEuler),'lower') * randn(3,1);
		measurement(iMeasVel) = [velocity(1); velocity(2); velocity(3)] + chol(Q(iMeasVel, iMeasVel),'lower') * randn(3,1);
		measurement(iMeasDoa) = [atan2(base2beaconInUSBL(2), base2beaconInUSBL(1)); asin(base2beaconInUSBL(3) / norm(base2beaconInUSBL))] ...
			+ chol(Q(iMeasDoa, iMeasDoa),'lower') * randn(2,1);
		measurement(iMeasDoppler) = [base2beaconInbase' * velocity / norm(base2beaconInbase); xn(iBeacon(end))] + chol(Q(iMeasDoppler, iMeasDoppler),'lower') * randn(2,1); 
	end
	NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
	measurement(iMeasEuler) = NormalizeAngle(measurement(iMeasEuler));
	measurement(iMeasDoa) = NormalizeAngle(measurement(iMeasDoa));
	
end
