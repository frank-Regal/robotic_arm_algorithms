%% Title:    THA 1, Programming Assignment Problem 1.C Roll-Pitch-Yaw Euler Angles.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Rotation Matrix to Roll-Pitch-Yaw Euler Angles

% Input:
%    R: 3x3 SO(3) rotation matrix
%
% Output:
%    Roll: [rad] angle about first x-axis rotation
%    Pitch: [rad] angle about second y-axis rotation
%    Yaw: [rad] angle about third z-axis rotation

function [roll, pitch, yaw]=RotToRPYAngles(R)

% Get Corresponding angle of input rotation matrix for checks
[theta, ~] = RotToAxisAngle(R);

if (theta > -pi/2 && theta < pi/2)
    % Reference: Lecture W3-L1 Slide 7
    yaw = atan2(R(3,2),R(3,3));
    pitch = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2));
    roll = atan2(R(2,1),R(1,1));

elseif (theta > pi/2 && theta < (3*pi)/2)
    % Reference: Lecture W3-L1 Slide 7
    yaw = atan2(-R(3,2),-R(3,3));
    pitch = atan2(-R(3,1),-sqrt(R(3,2)^2 + R(3,3)^2));
    roll = atan2(-R(2,1),-R(1,1));
else
    disp("ERROR: SINGULARITY!")
    roll = nan;
    pitch = nan;
    yaw = nan;
end

if (roll == pi/2 || roll == -pi/2 || yaw == pi/2 || yaw == -pi/2 || pitch == pi/2 || pitch == -pi/2)
    disp("ERROR: SINGULARITY at least one angle at pi/2 or -pi/2")
    roll = nan;
    pitch = nan;
    yaw = nan;

end