%% Title:    THA 1, Programming Assignment - Quat to Axis Angle
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Quaterion to a Axis-Angle Rep

% Inputs:
%    q: 1x4 quaternion
%
% Outputs:
%    w: 3x1 vector defining the associated rotation axis
%    theta: [rad] angle of rotation about w

function [theta,w]= QuaternionToAxisAngle(q)

% Ref: Murray, A Mathetmatical Introduction to Robotic Manipulation
theta = 2*acos(q(1));

if theta ~= 0
    w(1:3) = q(2:4)/ (sin(theta/2));
else
    w(1:3) = [0;0;0];
end

end