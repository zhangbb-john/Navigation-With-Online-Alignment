function [initState, y, y_Q, groundTruth, params] = getMeas(mode, dynModel, measModel, filter, folder)
if strcmp(mode.data, 'field')
	[initState, y, groundTruth] = readData(folder);

elseif (strcmp(mode.data, 'sim') && strcmp(mode.solution, 'align'))
	% Default settings
	params = [];
	% pf
	switch filter
		case 'pf'
			Qparam = [1e-4, 1e-4, 1e-2, 1e-6, 4e-2];
			Q0param = [1e-4, 1e-4, 1e-4, 4e-2, 900];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'ukf'
			% ukf
			Qparam = [0.25e-4, 1e-4, 16e-4, 1e-10, 25e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-2, 6400];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'doaUkf'
			% ukf
			Qparam = [0.25e-4, 1e-4, 16e-4, 1e-10, 25e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-2, 6400];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'drUkf'
			% ukf
			Qparam = [1e-4, 1e-4, 1e-2, 1e-8, 1e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-4, 100];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'ekf'
			% ukf
			Qparam = [1e-4, 1e-4, 16e-4, 1e-10, 64e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-2, 6400];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'lsUkf'
			%lsUkf
			Qparam = [1e-4, 1e-4, 1e-4, 1e-12, 1e-6];
			Q0param = [1e-4, 1e-2, 1e-2, 1e-6, 1];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];			
		otherwise
			% othere
			Qparam = [1e-4, 1e-4, 1e-2, 1e-8, 1e-4];
			Q0param = [1e-4, 1e-4, 1e-2, 1e-4, 100];
			Rparam = [1, 16e-4, 0.0175^2, 1];			
	end
	
	Qpos = diag(20 * ones(1, 3) * Qparam(1));
	Qeuler = diag(20 * [1, 1, 4] * Qparam(2));
	Qvel = diag(20 * ones(1, 3) * Qparam(3));
	Qoffset = diag(20 * [1 1 2] * Qparam(4));
	Qbeacon = diag(20 * ones(1, 3) * Qparam(5));
	params.Qprocess = blkdiag(Qpos, Qeuler, Qvel, Qoffset, Qbeacon);
	
	Q0pos = diag(ones(1, 3) * Q0param(1));
	Q0euler = diag([1 1 4] * Q0param(2));
	Q0vel = diag(ones(1, 3) * Q0param(3));
	Q0offset = diag([1, 1, 1]* Q0param(4));
	Q0beacon = diag(ones(1, 3) * Q0param(5));

	Q0 = blkdiag(Q0pos, Q0euler, Q0vel, Q0offset, Q0beacon);
	params.Q0 = Q0;
	Reuler = diag([0.007, 0.007, 0.034]) * diag([0.007, 0.007, 0.034]) * Rparam(1);
	Rvel = diag(ones(1, 3) * Rparam(2));
	Rdepth = 0.36 * Rparam(3);
	Rdoa = diag([1 1 ] * Rparam(4));
	Rdoppler = diag([25e-4, 0.36] * Rparam(5));
	params.Qmeas = blkdiag(Reuler, Rvel, Rdepth, Rdoa, Rdoppler);
	params.trajType = mode.traj;
	params.makePlots = 1;
	params.visualiseResults = 1;
	params.dt = 0.05;
	params.freq_dvl = 1;
	params.freq_acoustic = 0.2;
	N_K = 1;
	params.N_K = N_K;
	[~, initState, y, y_Q, groundTruth] = generateData(mode, params, dynModel, measModel);	
	
elseif (strcmp(mode.data, 'sim') && strcmp(mode.solution, 'nav'))
	% Default settings
	params = [];
	% pf
	switch filter
		case 'pf'
			Qparam = [1e-4, 1e-4, 1e-2, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'ukf'
			% ukf
			Qparam = [0.25e-4, 1e-4, 16e-4, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
			% disp('This is a significant bug!!!, Q_param(3) should be 16e-4 at best');
		case 'drUkf'
			% ukf
			Qparam = [1e-4, 1e-4, 1e-2, 1e-10, 1e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-4, 1];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'doaUkf'
			% ukf
			Qparam = [1e-4, 1e-4, 1e-2, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'ekf'
			% ukf
			Qparam = [1e-4, 1e-4, 16e-4, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];
		case 'lsUkf'
			%lsUkf
			Qparam = [1e-4, 1e-4, 16e-4, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-2, 1e-4, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 1, 0.0175^2, 1];			
		otherwise
			% othere
			Qparam = [1e-4, 1e-4, 1e-2, 1e-20, 1e-10];
			Q0param = [1e-4, 1e-4, 1e-2, 1e-10, 1e-10];
			Rparam = [1, 16e-4, 0.0175^2, 1];			
	end
	if (strcmp(mode.traj, 'circle_6d'))% || strcmp(mode.traj, 'circle_sine_shallow'))
		Qparam(3) = 1e-4;
	end	
	disp(['Qparam(3) = 1e-4 is better but now we set it to ', num2str(Qparam(3))]);
	Qpos = diag(20 * ones(1, 3) * Qparam(1));
	Qeuler = diag(20 * [1, 1, 4] * Qparam(2));
	Qvel = diag(20 * ones(1, 3) * Qparam(3));
	Qoffset = diag(20 * [1 1 2] * Qparam(4));
	Qbeacon = diag(20 * ones(1, 3) * Qparam(5));
	params.Qprocess = blkdiag(Qpos, Qeuler, Qvel, Qoffset, Qbeacon);
	
	Q0pos = diag(ones(1, 3) * Q0param(1));
	Q0euler = diag([1 1 4] * Q0param(2));
	Q0vel = diag(ones(1, 3) * Q0param(3));
	Q0offset = diag([1, 1, 1]* Q0param(4));
	Q0beacon = diag(ones(1, 3) * Q0param(5));

	Q0 = blkdiag(Q0pos, Q0euler, Q0vel, Q0offset, Q0beacon);
	params.Q0 = Q0;
	Reuler = diag([0.007, 0.007, 0.034]) * diag([0.007, 0.007, 0.034]) * Rparam(1);
	Rvel = diag(ones(1, 3) * Rparam(2));
% 	Rvel = diag(ones(1, 3) * Rparam(2)) * 100;
% 	disp('This is a significant bug!!!! to to solved vel error for measurement')
	Rdepth = 0.36 * Rparam(3);
	Rdoa = diag([1, 1 * mode.elevation_error_scale * mode.elevation_error_scale] * Rparam(4));
	disp('this is a bug; Rdoa is ')
	Rdoa
	Rdoppler = diag([25e-4, 0.36] * Rparam(5));
	params.Qmeas = blkdiag(Reuler, Rvel, Rdepth, Rdoa, Rdoppler);
	params.trajType = mode.traj;

	params.makePlots = 1;
	params.visualiseResults = 1;
	params.dt = 0.05;
	params.freq_dvl = 1;
	params.freq_acoustic = 0.2;
	N_K = 1;
	params.N_K = N_K;
	[~, initState, y, y_Q, groundTruth] = generateData(mode, params, dynModel, measModel);
end
end



