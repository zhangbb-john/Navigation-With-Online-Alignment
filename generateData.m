function [dx, initState, y, y_Q, groundTruth] = generateData(mode, params,dynModel,measModel)
%% GENERATEDATA_DENSE - Generates dense data for simulation experiments
%
% Syntax:
%   [dx, initState, y, groundTruth] = generateData(params, dynModel, fieldData)
%
% In:
%   params    - Struct containing various parameters for data generation
%   dynModel  - Function handle for the dynamic model
%
% Out:
%   dx         - Simulated odometry data
%   initState  - Initial state for the simulation
%   y          - Simulated sensor measurements
%   groundTruth - Struct containing ground truth data for position, 
%                 orientation, odometry, and other relevant information
%
% Description:
%   This function generates simulated data for dense experiments, including
%   odometry data, sensor measurements, and ground truth information. The 
%   function uses provided parameters and dynamic models, and can simulate 
%   data for various trajectory types such as 2D circles, 3D squares, and 
%   others. It can also generate field data if not provided.
%
% Copyright:
%   2023-   Bingbing

%% Generate odometry data 
% Parameters should either be input via params struct or standard inputs
% are used 
% If file run separately then extract info
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
makePlots = params.makePlots;
trajType = params.trajType;



%% Simulate groundtruth data and noiseless odometry for different cases
groundTruth = [];
switch trajType
	case 'static'
        disp('Generating static-type 6D data')
        ellipse_x = 0;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * sin(K * t); zeros(size(t)); ones(size(t)) * 10];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 10]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 0; 10]]; % Initial state	
		end
	case 'static_static_attitude'
        disp('Generating static-type 6D data')
        ellipse_x = 0;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * sin(K * t); zeros(size(t)); ones(size(t)) * 10];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.0;
        k_pitch = 0.0;
        k_yaw = 0;
		w = 0.04;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 10]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 0; 10]]; % Initial state	
		end
    case 'circle_2D'
        disp('Generating 2D data on circle')
        radius = 2; % Radius of circular trajectory
        nLaps = 3; % Number of laps
        dpsi = 5; % Obtain odometry sample every 5 degrees of a circle
        psi = (0:dpsi:360 * nLaps - dpsi)*pi/180;
        N = length(psi);

        pos = [radius * cos(psi); radius * sin(psi)]; % True position
        groundTruth.pos = pos; % Save ground truth data
        initState = pos(:,1); % Initial state
        dx = diff(pos'); % Resulting odometry
    case 'bean_2D'
        disp('Generating bean-shaped 2D data')
        nLaps = 3; % Number of laps
        nDataPointsPerLap = 63; % Number of points per lap
        a = 5;
        psi = linspace(0,pi,nDataPointsPerLap)';
        r = a*sin(psi).^3 + a*cos(psi).^3;
        u = r.*cos(psi)-.3;
        v = r.*sin(psi)-.3;
%         yu = diff(u);
%         yv = diff(v);
%         th = atan2(yv,yu);
        pos = [u'; v'];
        % Center positions around zero
        pos = pos - mean([min(pos,[],2) , max(pos,[],2)],2); 
        % Make multiple laps
        pos = [pos , repmat(pos(:,2:end),[1 nLaps-1])]; % True position
        
        groundTruth.pos = pos; % Save ground truth data
        initState = pos(:,1); % Initial state
        dx = diff(pos'); % Resulting odometry
        N = length(pos);
    case 'square_3D'
        disp('Generating 3D data on square')    
        N = params.N;
        pos = [zeros(1,N/4), linspace(0,2,N/4), 2*ones(1,N/4), linspace(2,0,N/4); ...
            linspace(0,2,N/4), 2*ones(1,N/4), linspace(2,0,N/4), zeros(1,N/4)];
        % Center positions around zero
        pos = pos - mean(pos,2);
        
        groundTruth.pos = pos; % Save ground truth data
        initState = [pos(:,1);0]; % Initial state
        dx = [diff(pos'),zeros(N-1,1)]; % Resulting odometry
    case {'line_3D','line_2D','line_3D_withPos'}
        disp('Generating data on line')  
        % Simulate position data
        if isfield(params,'N')
            N = params.N;
        else 
            N = 32;
        end
        pos = [zeros(1,N); ...
            [linspace(0,3,N/2) , ...
            linspace(3,0,N/2)]];
        % Center positions around zero
        pos = pos - mean(pos,2);
        
        groundTruth.pos = pos; % Save ground truth data
        initState = pos(:,1); % Initial position
        dx = diff(pos'); % Resulting odometry
        if strcmp(trajType,'line_3D') || strcmp(trajType,'line_3D_withPos')
            initState = [initState ; 0];
            dx = [dx, zeros(N-1,1)];
        end
    case 'line_6D'
        disp('Generating 6D data on line')
        N = 32;
        pos = [zeros(1,N); ...
            [linspace(0,3,N/2) , ...
            linspace(3,0,N/2)] ; ...
            zeros(1,N)];
        pos = pos - mean(pos,2);
        quat = [[ones(N/2,1),zeros(N/2,3)] ; ...
            [zeros(N/2,3),-ones(N/2,1)]];

        % Ground truth
        groundTruth.pos = pos;
        groundTruth.quat = quat;

        % Odometry measurements
        initState = [pos(:,1) ; quat(1,:)'];
        dQuat = squeeze(multiprod(qLeft(qInv(quat(1:end-1,:))), ...
            reshape(quat(2:end,:)',[4 1 length(quat)-1])))';

        dPos = diff(pos');
        dx = [dPos , dQuat];
        N = length(pos);

	case 'straight_line'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * sin(K * t); zeros(size(t)); ones(size(t)) * 10];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 10]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 0; 10]]; % Initial state	
		end

	case 'straight_line_static_attitude'
        disp('Generating straight_line_static_attitude-type 6D data')
        ellipse_x = 50;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * sin(K * t); zeros(size(t)); ones(size(t)) * 10];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.0;
        k_pitch = 0.0;
        k_yaw = 0.0;
		w = 0.01; %0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 10]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 80.0; 10]]; % Initial state	
		end	
	case 'vertical_line'
        disp('Generating vertical_line-type 6D data')
        amp_z = 100;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [zeros(size(t)); zeros(size(t)); amp_z * sin(K * t) + 50];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 50]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 0; 50]]; % Initial state	
		end
	case 'vertical_line_static_attitude'
        disp('Generating vertical_line-type 6D data')
        amp_z = 50;
		t_end = 6000;
		circle_num = 60;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [zeros(size(t)); zeros(size(t)); amp_z * sin(K * t) + 50];
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0;
        k_pitch = 0;
        k_yaw = 0;
		w = 0.04;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * 80.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 50]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [80.0; 0; 50]]; % Initial state	
		end		
	
    case 'circle_6d'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); zeros(size(t))];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * mode.depth]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 0; mode.depth]]; % Initial state	
		end

    case 'circle_6d_static_attitude'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); zeros(size(t))];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.0;
        k_pitch = 0.0;
        k_yaw = 0;
		w = 0.04;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * mode.depth]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 0; mode.depth]]; % Initial state	
		end
    case 'bean_6D'
        disp('Generating bean-shaped 6D data')
        % Simulate position data
        nLaps = 3;
        nDataPointsPerLap = 640;
        a = 15;
        psi = linspace(0,nLaps*pi,nLaps * nDataPointsPerLap);
        r = a*sin(psi).^3 + a*cos(psi).^3;
        u = r.*cos(psi)-.3;
        v = r.*sin(psi)-.3;
        yu = diff(u);
        yv = diff(v);
        th = atan2(yv,yu); th = [th, th(end)];
        pos = [u; v ; zeros(1,length(u))];
        N = length(pos);
        R = [reshape(cos(th),[1 1 N]), reshape(sin(th),[1 1 N]), zeros(1, 1, N); ...
                reshape(-sin(th),[1 1 N]), reshape(cos(th),[1 1 N]), zeros(1, 1, N); ...
                zeros(1,1,N), zeros(1,1,N), ones(1,1,N)];
% 		R = [ones(1,1,N), zeros(1,1,N), zeros(1,1,N) ; ...
%                zeros(1,1,N),  ones(1,1,N), zeros(1,1,N) ; ...
%                 zeros(1,1,N), zeros(1,1,N), ones(1,1,N)];		
        %quat = rotm2quat(R);
% 		rot2euler(R(:, :, 2))
        quat = rmat2quat(R)';
% 		quat(2, :) 
        % Center positions around zero
        pos = pos - mean([min(pos,[],2) , max(pos,[],2)],2); 
        
        % Save ground truth data
		
        groundTruth.pos = pos; 
        groundTruth.quat = quat;
        
        dPos = diff(pos'); % Resulting odometry
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
% 		vel = ones(size(vel));
		rots = quat2rmat(quat);
		for i = 1 : size(vel, 2)
			vel(:, i) = rots(:, :, i)' * vel(:, i);
		end
		euler = zeros(3, size(quat, 2));
		for i = 1 : size(quat, 1)
			[yaw, pitch, roll] = quat2angle(quat(i, :) , 'ZYX');
			euler(:, i) = [roll, pitch, yaw]';
		end
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 0.0; ones(1, size(pos, 2)) * 0.1]; ...
			[ones(1, size(pos, 2)) * 0; ones(1, size(pos, 2)) * 0; ones(1, size(pos, 2)) * 0]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
        initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state		
	case '21'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); z * sin(K1 * K * t)];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 20.0; ones(1, size(pos, 2)) * 10.0]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 20; 10]]; % Initial state	
		end
	case '22'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); z * sin(K1 * K * t) + 20];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 20.0; ones(1, size(pos, 2)) * 10.0]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
        initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state	

	case 'circle_sine'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); z * sin(K1 * K * t) + 20];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
		w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 20.0; ones(1, size(pos, 2)) * mode.depth]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 20; mode.depth]]; % Initial state	
		end

	case 'circle_sine_static_attitude'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 20;                                                  
        K1 = 10;     
		t_end = 6000;
		circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
		t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); z * sin(K1 * K * t) + 20];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
		dPos = diff(pos'); % Resulting odometry
        N = length(pos);
		vel = dPos./ params.dt;
		vel = [vel(1, :); vel]';
        k_roll = 0.0;
        k_pitch = 0.0;
        k_yaw = 0;
		w = 0.04;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
		rots = zeros(3, 3, size(euler, 2));
		quat = zeros(4, size(euler, 2));
		for i = 1 : size(vel, 2)
			rot = euler2rot(euler(:, i));
			vel(:, i) = rot' * vel(:, i);
			quat(:, i) = rmat2quat(rot)';
		end		
		
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
		groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
			[ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 20.0; ones(1, size(pos, 2)) * mode.depth]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
		if (strcmp(mode.solution, 'align'))
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
		else 
			initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 20; mode.depth]]; % Initial state	
		end

    case 'circle_sine_shallow'
        disp('Generating 21-type 6D data')
        ellipse_x = 50;
        ellipse_y = 50;
        z = 5;                                                  
        K1 = 50;     
        t_end = 6000;
        circle_num = 8;
        K = 2 * pi / (t_end / circle_num); 
        t = 0 : params.dt : t_end;
        pos = [ellipse_x * cos(K * t) - ellipse_x; ellipse_y * sin(K * t); z * sin(K1 * K * t) + 5];
%         dp = [-K * ellipse_x * sin(K * t); K * ellipse_y * cos(K * t); z * K1 * K * cos(K1 * K * t));
        dPos = diff(pos'); % Resulting odometry
        N = length(pos);
        vel = dPos./ params.dt;
        vel = [vel(1, :); vel]';
        k_roll = 0.9;
        k_pitch = 0.9;
        k_yaw = 3.14;
        w = 0.04 / k_yaw;
        euler = [k_roll * cos(w * t); k_pitch * sin(w * t); k_yaw * sin(w * t)];                  
        rots = zeros(3, 3, size(euler, 2));
        quat = zeros(4, size(euler, 2));
        for i = 1 : size(vel, 2)
            rot = euler2rot(euler(:, i));
            vel(:, i) = rot' * vel(:, i);
            quat(:, i) = rmat2quat(rot)';
        end		
        
        % Save ground truth data
        groundTruth.pos = pos; 
        groundTruth.quat = quat;  
        groundTruth.gt = [pos; euler; vel; [ones(1, size(pos, 2)) * 0.0349; ones(1, size(pos, 2)) * 0.0698; ones(1, size(pos, 2)) * 0.1047]; ...
            [ones(1, size(pos, 2)) * -50.0; ones(1, size(pos, 2)) * 20.0; ones(1, size(pos, 2)) * mode.depth]];
% 		quat2euler(quat(:, 2))
        % Odometry measurements
        if (strcmp(mode.solution, 'align'))
            initState = [pos(:,1) ; euler(:, 1); vel(:, 1); zeros(3, 1); zeros(3, 1)]; % Initial state				
        else 
            initState = [pos(:,1) ; euler(:, 1); vel(:, 1); [0.0349; 0.0698; 0.1047]; [-50.0; 20; mode.depth]]; % Initial state	
        end
end


%% Add measurement noise to odometry by using the dynamic model to put it 
% on the way that is assume in the model
Qprocess = params.Qprocess * 0.0001;
if size(Qprocess,3) == 1 % Allow for both time-varying and constant Qprocess
      Qprocess = repmat(Qprocess,[1 1 N]);
end
% Run dynamic model forward 
dt = params.dt;
x = zeros(N,length(initState));
y = zeros(N, iMeasDoppler(end));% bearing, elevation, doppler
x(1, :) = initState;

for i = 2:N
	[x(i, :)] = dynModel(x(i - 1, :)', dt, Qprocess(:,:,i - 1));
	x(i, 7 : 9) = vel(: , i);
	x(i, 4 : 6) = euler(:, i);
end
Qmeas = params.Qmeas;% * 1e-10;
if size(Qmeas,3) == 1 % Allow for both time-varying and constant Qprocess
      Qmeas = repmat(Qmeas,[1 1 N]);
end


y_Q = Qmeas;

for i = 1 : N
	y(i, :) = measModel(groundTruth.gt(:, i), Qmeas(:, :, i))';
	idx = [];
	if (mod(i * dt, 1 / params.freq_dvl) > 0.01)
		idx = [idx, iMeasVel]; 
	end
	if (mod(i * dt, 1 / params.freq_acoustic) > 0.01)
		idx = [idx, iMeasDoa, iMeasDoppler]; 
	end 	
	y_Qi = Qmeas(:, :, i);
	y_Qi(idx, idx) = y_Qi(idx, idx) * 1e6;
	y_Q(:, :, i) = y_Qi;
end
dx = [diff(x(:,1:3))];
groundTruth.odometry = x;
groundTruth.Qprocess = Qprocess;

%% Visualize ground truth and odometry
if makePlots && params.visualiseResults
	figure(1); cla; hold on

	plot_gt_pos = plot(groundTruth.pos(1,:), groundTruth.pos(2,:), 'k', 'LineWidth', 2);
	plot_gt_odom = plot(groundTruth.odometry(:,1), groundTruth.odometry(:,2), 'r', 'LineWidth', 2);
	axis equal
% 	xlim([min(xt(:,1)) max(xt(:,1))])
% 	ylim([min(xt(:,2)) max(xt(:,2))])
	legend([plot_gt_pos, plot_gt_odom], {'true trajectory', 'odometry'});
	title('True map, true trajectory, odometry')
end

end