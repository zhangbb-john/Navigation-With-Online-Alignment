function [initState, y, groundTruth, params] = getMeas(flag, dynModel, measModel, folder)
if flag == 1
	[initState, y, groundTruth] = readData(folder);
else
	% Default settings
	params = [];
	Qpos = diag(20 *[1e-4, 1e-4, 1e-4]);
	Qeuler = diag(20 *[1e-4, 1e-4, 1e-4]);
	Qvel = diag(20 *[0.01, 0.01, 0.01]);
	Qoffset = diag(20 * [1e-10, 1e-10, 1e-10]);
	Qbeacon = diag(20 * [1e-20, 1e-20, 1e-20]);
	params.Qprocess = blkdiag(Qpos, Qeuler, Qvel, Qoffset, Qbeacon);
	
	Reuler = diag([0.01, 0.007, 0.0349]) * diag([0.01, 0.01, 0.0349]);
	Rvel = diag([16e-4, 16e-4, 16e-4]);
	Rdoa = diag([0.0175, 0.0175]) * diag([0.0175, 0.0175]) * 100;
	Rdoppler = diag([0.1, 0.6]) * diag([0.1, 0.6]) * 100;
	params.Qmeas = blkdiag(Reuler, Rvel, Rdoa, Rdoppler);
	params.trajType = 'bean_6D';
	params.makePlots = 1;
	params.visualiseResults = 1;
	params.dt = 0.05;
	N_K = 1;
	params.N_K = N_K;
	[~, initState, y, groundTruth] = generateData(params, dynModel, measModel);	
end
end



