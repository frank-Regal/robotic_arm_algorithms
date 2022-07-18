%% Title:    THA 1, Programming Assignment Problem 2.A.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a  Axis-Angle Rep to a Rotation Matrix
% Ref: Murray, A Mathetmatical Introduction to Robotic Manipulation

% Inputs:
%    w: 3x1 vector defining the axis of rotation
%    theta_rad: angle in (rad) that defines the angle of rotation about w

% Outputs:
%    RotMat: 3x3 rotation matrix

function RotMat=AxisAngleToRot(w, theta_rad)

% Create a Unit Vector
mag = sqrt((w(1)^2) + (w(2)^2) + (w(3)^2));
w = w/mag;

% Setup Identity Matrix
I = eye(3,3);

% Convert Input Vector to Skew Symmetric Matrix (3x3)
w_skew = VectorToSkewMatrix(w);

% Square of Skew Symmetric Matrix
w_skew_sqr = w_skew*w_skew;

% Use Rodrigues Formula if Magnitude of Axis Vector is Unity
% Equation 2.14, Murray
RotMat = I + w_skew*sin(theta_rad) + (w_skew_sqr)*(1-cos(theta_rad));

end