% File path
filename = 'timelogbatcheck.bin';

% Open the binary file for reading
fileID = fopen(filename, 'rb');

% Read all timestamps as unsigned 32-bit integers
timeData = fread(fileID, 'uint32');

% Close the file
fclose(fileID);

% Plot the timestamps
figure;
plot(timeData, '-o');
xlabel('Sample Index');
ylabel('Timestamp (µs)');
title('Logged Timestamps at 1 kHz');
grid on;

% Calculate intervals between timestamps
timeIntervals = diff(timeData);
disp('Time intervals (µs):');
disp(timeIntervals);

% Plot intervals to verify timing consistency
figure;
plot(timeIntervals, '-o');
xlabel('Sample Index');
ylabel('Time Interval (µs)');
title('Time Intervals Between Samples');
grid on;
