%% Title:    THA 1, Programming Assignment - Helper Function
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a  Axis-Angle Rep to a Quaterion
% Ref: Murray, A Mathetmatical Introduction to Robotic Manipulation

% Inputs:
%    w: 3x1 vector defining the axis of rotation
%    theta: defines the angle of rotations

% Outputs:
%    output: 4x1 Unit Quaternion

function output = AxisAngleToQuaternion(w,theta_rad)

if isnan(w) == true
    output = [1,0,0,0];
else
    % Create unit vector
    w = w/sqrt(w(1)^2 + w(2)^2 + w(3)^2);

    % Create initial Quaternion, Lynch Equation B9
    q = [cos(theta_rad/2) , w(1)*sin(theta_rad/2) , w(2)*sin(theta_rad/2) , w(3)*sin(theta_rad/2) ];
    output = q;
end
end