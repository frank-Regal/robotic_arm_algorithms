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
%    theta: degree (deg) that defines the angle of rotations

% Outputs:
%    RotMat: 3x3 rotation matrix

function RotMat=AxisAngleToRotOld(w, theta_rad)

% Find Magnitude of Input Vector
mag = sqrt((w(1)^2) + (w(2)^2) + (w(3)^2));

% Setup Identity Matrix
I = eye(3,3);

% Convert Input Vector to Skew Symmetric Matrix (3x3)
w_skew = VectorToSkewMatrix(w);
% Square of Skew Symmetric Matrix
w_skew_sqr = w_skew*w_skew;

if mag == 1
    % Use Rodrigues Formula if Magnitude of Axis Vector is Unity
    % Equation 2.14, Murray
    RotMat = I + w_skew*sin(theta_rad) + (w_skew_sqr)*(1-cos(theta_rad));
else
    % Use Expansion of Rodrigues Formula if Magnitude is not Unity
    % Murray
    RotMat = I + (w_skew/mag)*sin(mag*theta_rad) + ...
                (w_skew_sqr/(mag^2))*(1-cos(mag*theta_rad));
end

end