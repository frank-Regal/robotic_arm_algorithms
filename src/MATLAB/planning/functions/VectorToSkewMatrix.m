%% Title:    THA 1, Programming Assignment - Vector to Skew Sym Matrix
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Takes a 3x1 vector and converts it to a skew symmetric matrix

% Input:
%     w: 3x1 vector representation of a skew symmetric matrix
% Output:
%     matrix: 3x3 skew symmetric matrix defined by the following
%     matrix = [0  -w3  w2;
%               w3   0 -w1;
%              -w2  w1   0];
%

function matrix=VectorToSkewMatrix(w)
% Takes in a 3x1 vector and converts it to a 3x3 skew symmetric matrix

% a = [1;2;3]
% b = [3;4;5]
% a x b = (a)^b = [a]b 
% Finding the skew symmetric of a (skew symmetric notation = [a]) times b 
% is exactly the same as the cross product of a x b

% Definition of Skew Symmetric Matrix
matrix = [0 -w(3) w(2);
          w(3) 0 -w(1);
         -w(2) w(1) 0];

end
