function writeToFolder(my_data, folderName, description)
varName = inputname(1);
% Define the data

% Define folder name and check if it exists, if not, create it
% folderName = 'MyDataFolder';
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

% Define file paths for the .mat file and the description .txt file
matFileName = strcat(varName, '.mat');
matFileName = fullfile(folderName, matFileName);
txtFileName = strcat(varName, '.txt');
txtFileName = fullfile(folderName, txtFileName);
new_data = my_data;
if (size(new_data, 2) == 1)
	new_data = new_data';
end
disp(['file to be saved is ', txtFileName, '; length(size(new_data)) is ', num2str(length(size(new_data)))])
if (length(size(new_data)) < 4)
    writematrix(new_data, txtFileName);  % Save as a text file
end
desFileName = fullfile(folderName, 'description.txt');
disp(['file to be saved is ', matFileName])

% Save the array as a .mat file in the folder
save(matFileName, "my_data");

% Create a text file with the description
fileID = fopen(desFileName, 'a');
fprintf(fileID, '%s\n', description);
fclose(fileID);

% Display success message
disp(['Data and description saved in folder: ', folderName]);
end
