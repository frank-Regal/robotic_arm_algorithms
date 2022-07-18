%% Title:    THA3, Parsing Function For optpivot Files
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [di,hi,NumFrames]=ReadOptPivot(FileName)

% Parse Cal Readings File Contents 
fileID = fopen(FileName,'rt');
header= fgetl(fileID);
header_cell = textscan(header, '%f%f%f%s', 'Delimiter', ',');
Nd = header_cell{1};
Nh = header_cell{2};
NumFrames = header_cell{3};
data = cell2mat(textscan(fileID, '%f%f%f', 'Delimiter', ','));

% Initialize Matrices
di = zeros(Nd,NumFrames*3);
hi = zeros(Nh,NumFrames*3);

% Create Frame Count
count = Nd+Nh;

for i = 1:NumFrames
    
    % Break Up Data
    buff = data((i*count)-(count-1) : i*count,:);

    % Get Coordinates of Optical Tracker points on EM base
    di(:,(i*3)-2:(i*3)) = buff(1:Nd,:);

    % Get Coordinates of Optical Tracker points on Probe
    hi(:,(i*3)-2:(i*3)) = buff(1+Nd:Nd+Nh,:);
end

fclose(fileID);

end