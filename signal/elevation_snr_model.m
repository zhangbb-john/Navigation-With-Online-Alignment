% Parameters
numSensors = 6;               % Number of sensors in the circular array
radius = 0.1;                 % Radius of the circular array (in meters)
frequency = 10000;            % Signal frequency (in Hz)
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
    
    % Simulate received signal at each sensor (without calculating delays)
    signal = exp(1j * 2 * pi * frequency * (sensorPositions' * sourceDirection(1:3)) / c);
    
    % Add noise to the signals (not delay noise, but signal noise)
    noise = (randn(size(signal)) + 1j * randn(size(signal))) * 0.2 * sqrt(0.5); % Complex Gaussian noise
    noisySignal = signal + noise;
    
    % Perform beamforming for a range of candidate angles
    candidateAngles = linspace(1, 89, 360);
    beamformingOutput = zeros(size(candidateAngles));
    
    for j = 1:length(candidateAngles)
        candidateTheta = deg2rad(candidateAngles(j));
        candidateDirection = [sin(candidateTheta); 0; cos(candidateTheta)];
        
        % Simulate the candidate signal at each sensor
        candidateSignal = exp(-1j * 2 * pi * frequency * (sensorPositions' * candidateDirection(1:3)) / c);
        
        % Compute the beamforming output by taking the dot product of the signals
        beamformingOutput(j) = abs(sum(noisySignal .* candidateSignal));
    end
    
    % Find the estimated angle (maximum beamforming response)
    [~, maxIdx] = max(beamformingOutput);
    estimatedAngle = candidateAngles(maxIdx);
    
    % Calculate error
    errors(i) = abs(trueAngle - estimatedAngle);
end
% signal = exp(1j * 2 * pi * frequency * (sensorPositions' * sourceDirection(1:3)) / c);
% noise = (randn(size(signal)) + 1j * randn(size(signal))) * 0.1;
% Calculate signal power
signalPower = mean(abs(signal).^2); 

% Calculate noise power
noisePower = mean(abs(noise).^2);

% Calculate SNR
snrLinear = signalPower / noisePower;
snrDb = 10 * log10(snrLinear);

% Display SNR
disp(['SNR (linear): ', num2str(snrLinear)]);
disp(['SNR (dB): ', num2str(snrDb)]);

% Plot the results
figure;
plot(angles, errors, 'LineWidth', 1.5);
xlabel('True Elevation Angle (degrees)');
ylabel('Elevation Error (degrees)');
title('Elevation Error vs. True Elevation Angle with 6 Sensors');
grid on;

% Create a boxplot of errors
group = repelem(1:8, 1000)';  % 8 groups, each of size 1000

figure;
boxplot(errors, group);
xlabel('True Elevation Angle x10[deg]');
ylabel('Elevation Error (degrees)');
title('Boxplot of Elevation Error by Group');
grid on;
mean_err = [];
for i = 1 : 8
	error_group  = errors((i - 1) * 1000 + 1: i * 1000);
	mean_err = [mean_err, mean(error_group)];
end
figure;
plot(mean_err, 'r');