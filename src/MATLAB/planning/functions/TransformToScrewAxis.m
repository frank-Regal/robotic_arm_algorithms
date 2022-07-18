%% Title:    THA 1, Programming Assignment - Transform to Screw Axis
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Transform to a screw axis

% Input:
%    T: 4x4 homogenous transformation matrix
%
% Output:
%    S: 6x1 screw axis with angular and linear velocity
%    theta: [rad] scalar theta that is found from input

function [S,theta]=TransformToScrewAxis(T)

% Calculate the trace of the rotation component
tra = T(1,1) + T(2,2) + T(3,3);

% Define p as the tranlation part of the transform
p = [T(1,4); T(2,4); T(3,4)];

% Define an identity matrix
I = eye(3,3);

if (tra == 3)
    % If Rotation is Identity
    % angular velocity is zero
    omega = 0; 
    % dist is equal to magnitude of translation
    theta = sqrt(p(1)^2 + p(2)^2 + p(3)^2); 
    % linear velocity
    v = p / theta; 
    S = [omega;v];
else
    % Solve Rodriques Formula
    % Reference: Equation 2.17, Murray, A Mathematical Introduction 
    % to Robotic Manipulation   
    theta = acos((tra - 1)/2); 
    omega = (1 / (2*sin(theta))) * [T(3,2) - T(2,3);
                                    T(1,3) - T(3,1);
                                    T(2,1) - T(1,2)];
    % Solve for linear velocity
    w_skew = VectorToSkewMatrix(omega);
    G_theta = ((1/theta)*I) - ((1/2)*w_skew) + ((1/theta) - ...
        ((1/2)*cot(theta/2))) * (w_skew^2);
    v = G_theta*p;
    S = [omega;v];
end
end