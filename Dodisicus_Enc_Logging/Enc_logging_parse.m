% MATLAB script to parse a single binary log file and plot time vs position

% Specify the log file to parse
logFile = 'Log_encoder_1.bin'; % Adjust this filename to the actual file

% Open the log file
fileID = fopen(logFile, 'rb');
if fileID == -1
    error('Could not open file: %s', logFile);
end

% Read binary data from the file
logData = fread(fileID, '*uint8');
fclose(fileID);

% Validate the data length (each entry is 6 bytes: 4 for time, 2 for position)
excessBytes = mod(length(logData), 6);
if excessBytes ~= 0
    warning('Log file contains incomplete data entries. Truncating excess %d byte(s).', excessBytes);
    logData = logData(1:end - excessBytes); % Truncate excess bytes
end

% Reshape the data into rows of 6 bytes (one entry per row)
logData = reshape(logData, 6, []).';

% Parse data: split into time and position
rawTimes = logData(:, 1:4); % First 4 bytes are time
rawPositions = logData(:, 5:6); % Last 2 bytes are position

% Convert binary data to numerical values (ensure little-endian interpretation)
times = typecast(uint8(reshape(rawTimes.', 1, [])), 'uint32');
positions = typecast(uint8(reshape(rawPositions.', 1, [])), 'uint16');

% Ensure positions are within 12-bit range
positions = bitand(positions, 4095); % Mask out any higher bits

% Convert times to seconds (assuming the Teensy uses `millis()`)
times = double(times) / 1000.0;

% Plot the results
figure;
plot(times, positions, '-o');
grid on;
title('Time vs Position');
xlabel('Time (s)');
ylabel('Position (counts)');
