clear,clc
filename = 'Log_encoder_2.bin'; 

% Open the file in binary read mode
fileID = fopen(filename, 'rb');
if fileID == -1
    error('Error: Unable to open file %s', filename);
end

% Read the entire file as little-endian binary data
rawData = fread(fileID, 'uint8');
fclose(fileID);

% Define record size based on the LogEntry structure
recordSize = 4 + 2; % 4 bytes for time + 2 bytes for position = 6 bytes
numRecords = floor(length(rawData) / recordSize);

% Initialize arrays for time and position
timeData = zeros(numRecords, 1, 'uint32');
positionData = zeros(numRecords, 1, 'uint16');

% Parse each record
for i = 1:numRecords
    % Calculate the start index of the current record
    startIdx = (i - 1) * recordSize + 1;
    
    % Extract time (4 bytes, little-endian)
    timeData(i) = typecast(uint8(rawData(startIdx:startIdx+3)), 'uint32');
    
    % Extract position (2 bytes, little-endian)
    positionData(i) = typecast(uint8(rawData(startIdx+4:startIdx+5)), 'uint16');
end

% Convert time to seconds (assuming time is in milliseconds)
timeInSeconds = double(timeData) / 1000;

% Plot the parsed data
figure;
plot(timeInSeconds, positionData, '-o');
xlabel('Time (s)');
ylabel('Position');
title('Encoder Position Over Time');
grid on;

% Save parsed data to a .mat file for further analysis
save('parsed_encoder_data.mat', 'timeInSeconds', 'positionData');

% Display summary
fprintf('Parsed %d records from %s.\n', numRecords, filename);
