clc, clear

% this program will try to get data off of the first serial port it can
% find. You might need to modify this to select a different serial port
% also it only works on 2024 or newer matlab (won't work on my bolved
% 2023b)

delete(serialportfind);
portslist = serialportlist()
serialObj = serialport(portslist(1),9600) % change this line to change which port is selected
configureTerminator(serialObj,"CR/LF");
flush(serialObj);
serialObj.UserData = struct("Data",[],"Count",1)

n = 0;
while(n < 10)
    % serialObj.UserData.Data = [];
    % serialObj.UserData.Count = 0;
    maxDataPoints = 1002; 
    configureCallback(serialObj, "terminator", @(src,event) readSineWaveData(src,event,maxDataPoints))
    plot(serialObj.UserData.Data(2:end));
    % n = n + 1;
    pause(0.05)
end

function readSineWaveData(src, ~, maxDataPoints)

% Read the ASCII data from the serialport object.
data = readline(src);

% Convert the string data to numeric type and save it in the UserData
% property of the serialport object.
src.UserData.Data(end+1) = str2double(data);

% Update the Count value of the serialport object.
src.UserData.Count = src.UserData.Count + 1;

% If over maxDataPoints points have been collected from the Arduino, switch off the
% callbacks and plot the data, starting from the second point. 
if src.UserData.Count > maxDataPoints
    configureCallback(src, "off");
end
end