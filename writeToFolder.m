function writeToFolder(myData, folderName, description)
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
writematrix(myData, txtFileName);  % Save as a text file

desFileName = fullfile(folderName, 'description.txt');

% Save the array as a .mat file in the folder
save(matFileName, "myData");

% Create a text file with the description
fileID = fopen(desFileName, 'a');
fprintf(fileID, '%s\n', description);
fclose(fileID);

% Display success message
disp(['Data and description saved in folder: ', folderName]);
end
