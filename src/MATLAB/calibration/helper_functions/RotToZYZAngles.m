%% Title:    THA 1, Programming Assignment Problem 1.C ZYZ Euler Angles.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Rotation Matrix to a ZYZ Euler Angles

% Input:
%    R: 3x3 SO(3) rotation matrix
%
% Output:
%    Z1: [rad] angle about first z-axis rotation
%    Y1: [rad] angle about second y-axis rotation
%    Z2: [rad] angle about third z-axis rotation

function [Z1, Y, Z2]=RotToZYZAngles(R)

% Get Corresponding angle of input rotation matrix for checks
[theta, ~] = RotToAxisAngle(R);

if (theta > 0 && theta < pi)
    % Reference: Lecture W3-L1 Slide 4
    Z1 = atan2(R(2,3),R(1,3));
    Y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    Z2 = atan2(R(3,2),-R(3,1));

elseif (theta > -pi && theta < 0)
    % Reference: Lecture W3-L1 Slide 5
    Z1 = atan2(-R(2,3),-R(1,3));
    Y = atan2(-sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    Z2 = atan2(-R(3,2),R(3,1));
else
    disp("ERROR: SINGULARITY, Outside of pi to -pi bounds!")
    Z1 = nan;
    Y = nan;
    Z2 = nan;
end

end