close all;
addpath(genpath('./.'))

iPos = 1 : 3;
iQuat = 4 : 6;
iVel = 7 : 9;
iOffset = 10 : 12;
iBeacon = 13 : 15;

iMeasEuler = 1 : 3;
iMeasVel = 4 : 6;
iMeasDoa = 7 : 8;
iMeasDoppler = 9 : 10;
global iPos iQuat iVel iOffset iBeacon;
global iMeasEuler iMeasVel iMeasDoa iMeasDoppler;
% x = [px, py, pz, eulerx, eulery, eulerz, anglex, angley, anglez, beaconx, beacony, beaconz];
time_length = 100;
folder = './';

flag = 0; %0 simulation; 1 read data;
debug_i = 0;
global debug_i;
[x0, measurements, x_true, params] = getMeas(flag, @dynModel, @measModel, folder);
if (size(measurements, 2) > iMeasVel(3))
	figure;
	plot(measurements(:, iMeasVel(1)), 'r'); hold on;
	plot(measurements(:, iMeasVel(2)), 'g'); hold on;
	plot(measurements(:, iMeasVel(3)), 'b'); hold on;
	title('Velocity measurements');
end
for i = 1 : time_length
	'error'
end
x0_nonLin = x0;
N_P = 1000; % Number of particles

[traj_max, traj_mean, xl_max, xl_mean, P_max, P_mean, traj_sample_iwmax] = ...
    particleFilter(@dynModel, @measModel, measurements,...
    x0_nonLin,params.Qprocess, params.Qmeas, N_P, params.dt, x_true);

figure;
plot(traj_max(iOffset(1), :), 'r-'); hold on;
plot(traj_max(iOffset(2), :), 'g-'); hold on;
plot(traj_max(iOffset(3), :), 'b-'); hold on;
title('Offset');
figure;
plot(traj_max(iPos(1), :), traj_max(iPos(2), :), 'r-'); hold on;
title('Trajectory');
xlabel('East [m]'); ylabel('North [m]');
dpos = diff(x_true.gt(iPos, :)');
dist = sum(sqrt(dpos(:, 1).^2 + dpos(:, 2).^2 + dpos(:, 3).^2));
rmse_pos_pf = norm(rms(x_true.gt(iPos, :)' - traj_max(iPos, :)'));
disp(['rmse is ', num2str(rmse_pos_pf), 'dist is ', num2str(dist)]);
 
figure;
plot(traj_max(iVel(1), :), 'r*'); hold on;
plot(traj_max(iVel(2), :), 'g*'); hold on;
plot(traj_max(iVel(3), :), 'b*'); hold on;

title('Estimated velocity ');
figure(18);
plot_est_yaw = plot(traj_max(iQuat(3), :), 'bo'); hold on;
legend(plot_est_yaw, 'estimated yaw');

title('Estimated yaw ');

 function [xpred] = dynModel(xn, dt, Q)
	global iPos iQuat iVel iOffset iBeacon debug_i;

    % Predict through dynamic model. Also optionally output dQuat for
    % generating odometry data
	R = euler2rot(xn(iQuat));
%     xpred_pos = xn(iPos) + xn(iVel); 
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
	measurement(iMeasEuler) = [euler(1); euler(2); euler(3)] + chol(Q(iMeasEuler, iMeasEuler),'lower') * randn(3,1);
	NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;
	measurement(iMeasEuler) = NormalizeAngle(measurement(iMeasEuler));
	measurement(iMeasVel) = [velocity(1); velocity(2); velocity(3)] + chol(Q(iMeasVel, iMeasVel),'lower') * randn(3,1);
    measurement(iMeasDoa) = [atan2(base2beaconInUSBL(2), base2beaconInUSBL(1)); asin(base2beaconInUSBL(3) / norm(base2beaconInUSBL))] ...
		+ chol(Q(iMeasDoa, iMeasDoa),'lower') * randn(2,1);
	measurement(iMeasDoa) = NormalizeAngle(measurement(iMeasDoa));
    measurement(iMeasDoppler) = [base2beaconInbase' * velocity; xn(iBeacon(end))] + chol(Q(iMeasDoppler, iMeasDoppler),'lower') * randn(2,1); 
end
