% Parameters
numSensors = 6;               % Number of sensors in the circular array
radius = 0.1;                   % Radius of the circular array (in meters)
frequency = 10000;             % Signal frequency (in Hz)
c = 1500;                     % Speed of sound in water (m/s)
wavelength = c / frequency;   % Wavelength of the signal
angles = linspace(1, 80, 8000); % True elevation angles (in degrees)
errors = zeros(size(angles)); % To store elevation errors

% Simulate array geometry
thetaArray = linspace(0, 2*pi, numSensors+1); % Sensor positions in radians
thetaArray(end) = []; % Remove duplicate point
sensorPositions = radius * [cos(thetaArray); sin(thetaArray); zeros(size(thetaArray))];

% Beamforming Simulation
for i = 1:length(angles)
    trueAngle = angles(i);          % True elevation angle (degrees)
    theta = deg2rad(trueAngle);     % Convert to radians
    sourceDirection = [sin(theta); 0; cos(theta)]; % Source unit vector
    
    % Calculate time delays for the signal
    delays = sensorPositions' * sourceDirection(1:3) / c;
	delays = delays + randn(size(delays)) * 1e-6;
    
    % Simulate received signal at each sensor
    signal = exp(1j * 2 * pi * frequency * delays);
    
    % Perform beamforming for a range of candidate angles
    candidateAngles = linspace(1, 89, 360);
    beamformingOutput = zeros(size(candidateAngles));
    
    for j = 1:length(candidateAngles)
        candidateTheta = deg2rad(candidateAngles(j));
        candidateDirection = [sin(candidateTheta); 0; cos(candidateTheta)];
        candidateDelays = sensorPositions' * candidateDirection(1:3) / c;
        candidateSignal = exp(-1j * 2 * pi * frequency * candidateDelays);
        beamformingOutput(j) = abs(sum(signal .* candidateSignal));
    end
    
    % Find the estimated angle (maximum beamforming response)
    [~, maxIdx] = max(beamformingOutput);
    estimatedAngle = candidateAngles(maxIdx);
    
    % Calculate error
    errors(i) = abs(trueAngle - estimatedAngle);
end

% Plot the results
figure;
plot(angles, errors, 'LineWidth', 1.5);
xlabel('True Elevation Angle (degrees)');
ylabel('Elevation Error (degrees)');
title('Elevation Error vs. True Elevation Angle with 6 Sensors');
grid on;
group = repelem(1:8, 1000)';  % 20 groups, each of size 50

boxplot(errors, group);
xlabel('Real elevation x10[deg]');
ylabel('Elevation Error (degrees)');
title('Boxplot of Data by Group');
grid on;