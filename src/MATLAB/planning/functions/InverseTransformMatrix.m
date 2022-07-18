%% Title:    THA 1, Programming Assignment - Inverse Transform Matrix
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Calculate the Inverse of a Tranformation Matrix

% Input:
%    T: 4x4 homogenous transformation matrix
%
% Output:
%    inverse: 4x4 homogenous transformation matrix inverse

function inverse=InverseTransformMatrix(T)

inverse = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4);
           [0 0 0] 1];
end