function [initState, y, groundTruth, params] = getMeas(flag, dynModel, measModel, filter, folder)
if flag == 1
	[initState, y, groundTruth] = readData(folder);
else
	% Default settings
	params = [];
	% pf
	switch filter
		case 'pf'
			Qparam = [1e-4, 1e-4, 1e-2, 1e-6, 4e-2];
			Q0param = [1e-4, 1e-4, 1e-4, 4e-2, 900];
			Rparam = [1, 16e-4, 0.0175^2, 1];
		case 'ukf'
			% ukf
			Qparam = [1e-4, 1e-4, 1e-2, 1e-8, 1e-4];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-4, 100];
			Rparam = [1, 16e-4, 0.0175^2, 1];
		case 'ekf'
			% ukf
			Qparam = [1e-4, 1e-4, 16e-4, 1e-8, 1e-2];
			Q0param = [1e-4, 1e-4, 1e-4, 1e-4, 100];
			Rparam = [1, 16e-4, 0.0175^2, 1];
		case 'lsUkf'
			%lsUkf
			Qparam = [1e-4, 1e-4, 1e-2, 1e-12, 1e-6];
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
	params.trajType = '22';
	params.makePlots = 1;
	params.visualiseResults = 1;
	params.dt = 0.05;
	N_K = 1;
	params.N_K = N_K;
	[~, initState, y, groundTruth] = generateData(params, dynModel, measModel);	
end
end



