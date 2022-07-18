%% Title:    THA3, Parsing Function For calbody Files
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [di, dBar, dTilda, ci, cBar, cTilda, ai, aBar, aTilda]=ReadCalBody(FileName)

% Parse CalBody File Contents 
fileID = fopen(FileName,'rt');
header= fgetl(fileID);
header_cell = textscan(header, '%f%f%f%s', 'Delimiter', ',');
Nd = header_cell{1};
Na = header_cell{2};
Nc = header_cell{3};
data = cell2mat(textscan(fileID, '%f%f%f', 'Delimiter', ','));

% Get Coordinates of Optical Tracker points on EM base
di = data(1:Nd,:);

% Get Coordinates of Optical Tracker points on Calibration Object
ai = data(1+Nd:Nd+Na,:);

% Get Coordinates of EM points on Calibration Object
ci = data(1+Nd+Na:Nd+Na+Nc,:);

% Calculate a,c,d tildas and a,c,d bars for PS to PS Registrations
% Init
aTilda = zeros(Na,3);
cTilda = zeros(Nc,3);
dTilda = zeros(Nd,3);

% Create Centroid of Points on Each Body
aBar = (1/Na) * sum(ai);
cBar = (1/Nc) * sum(ci);
dBar = (1/Nd) * sum(di);

% Create Matrix of Distances from Centroid
for i = 1:Na
    aTilda(i,:) = ai(i,:) - aBar;
end
for i = 1:Nc
    cTilda(i,:) = ci(i,:) - cBar;
end
for i = 1:Nd
    dTilda(i,:) = di(i,:) - dBar;
end

% Close File
fclose(fileID);

end