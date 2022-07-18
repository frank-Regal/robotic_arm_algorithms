%% Title:    THA3, Parsing Function For empivot Files
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [gi,NumFrames]=ReadEmPivot(FileName)

% Parse Cal Readings File Contents 
fileID = fopen(FileName,'rt');
header= fgetl(fileID);
header_cell = textscan(header, '%f%f%s', 'Delimiter', ',');
Ng = header_cell{1};
NumFrames = header_cell{2};
data = cell2mat(textscan(fileID, '%f%f%f', 'Delimiter', ','));

% Initialize Matrices
gi = zeros(Ng,NumFrames*3);

for i = 1:NumFrames
    gi(:,(i*3)-2:(i*3)) = data((i*Ng)-(Ng-1) : i*Ng,:);
end

fclose(fileID);

end