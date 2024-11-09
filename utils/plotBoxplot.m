% Sample data (replace this with your actual time series data)
% time_series = randn(1, 10000); % Example data
function plotBoxplot(time_series)
if (size(time_series, 1) > 1)
	time_series = time_series';
end
num_parts = 50;

% Determine the length of each part
n = length(time_series);
part_size = floor(n / num_parts);

% Pre-allocate arrays to store statistics
means = zeros(1, num_parts);
stds = zeros(1, num_parts);
data = [];
% Split the time series and calculate mean and std for each part
for i = 1:num_parts
    % Define the indices for each part
    start_idx = (i-1) * part_size + 1;
    end_idx = min(i * part_size, n);
    
    % Extract the segment
    segment = time_series(start_idx:end_idx);
    data = [data, segment'];
    % Calculate mean and std
end

% Combine means and stds into a matrix for boxplot
data_for_boxplot = [means; stds]'; % Transpose to make it [20x2] format

% Generate boxplot with time parts as x-axis labels
boxplot(data);
xlabel('Time');
ylabel('Value');

% Set custom x-tick labels to represent time parts
% set(gca, 'XTick', 1:20, 'XTickLabel', arrayfun(@(x) sprintf('Part %d', x), 1:num_parts, 'UniformOutput', false));
end
