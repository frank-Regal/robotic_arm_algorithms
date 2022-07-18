%% Title:    THA3, Parsing Function For calreadings Files
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [di, ai, ci, NumFrames]=ReadCalReadings(FileName)

% Parse Cal Readings File Contents 
fileID = fopen(FileName,'rt');
header= fgetl(fileID);
header_cell = textscan(header, '%f%f%f%f%s', 'Delimiter', ',');
Nd = header_cell{1};
Na = header_cell{2};
Nc = header_cell{3};
NumFrames = header_cell{4};
data = cell2mat(textscan(fileID, '%f%f%f', 'Delimiter', ','));

% Initialize Matrices
di = zeros(Nd,NumFrames*3);
ai = zeros(Na,NumFrames*3);
ci = zeros(Nc,NumFrames*3);

% Number of Frames
count = Nd+Na+Nc;

for i = 1:NumFrames
    buff = data((i*count)-(count-1) : i*count,:);
    % Get Coordinates of Optical Tracker points on EM base
    di(:,(i*3)-2:(i*3)) = buff(1:Nd,:);

    % Get Coordinates of Optical Tracker points on Calibration Object
    ai(:,(i*3)-2:(i*3)) = buff(1+Nd:Nd+Na,:);

    % Get Coordinates of EM points on Calibration Object
    ci(:,(i*3)-2:(i*3)) = buff(1+Nd+Na:Nd+Na+Nc,:);
end

fclose(fileID);

end