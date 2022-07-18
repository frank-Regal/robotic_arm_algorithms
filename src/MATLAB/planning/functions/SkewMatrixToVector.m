%% Title:    THA 1, Programming Assignment - Skew to Vector
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Takes a 3x3 skew symmetric matrix and converts it back to a vector

% Input:
%     SkewMatrix: 3x3 skew symmetric matrix defined by the following
%     SkewMatrix = [0  -w3  w2;
%                   w3   0 -w1;
%                  -w2  w1   0];
%
% Output:
%     vector: 3x1 vector representation of a skew symmetric matrix

function vector=SkewMatrixToVector(SkewMatrix)

% Check to make sure it is a 3x3 matrix
[r,c] = size(SkewMatrix);
if r < 3 || c < 3
    vector = 'Undefined';
else 
% Assign variables to matrix
    vector = [SkewMatrix(3,2);
              SkewMatrix(1,3);
              SkewMatrix(2,1)];
end

end
