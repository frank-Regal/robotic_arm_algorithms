%% Title:    THA 1, Programming Assignment Problem 1.B.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Rotation Matrix to a Quaternion

% Ref: Lynch, Modern Robotics

% Inputs:
%    R: 3x3 rotation matrix 

% Outputs:
%    q: 4x1 unit quaternion


function q=RotToQuaternion(R)

    [theta, ~] = RotToAxisAngle(R);

    if (theta >= pi || theta <= -pi)
        disp("ERROR: Rotation Matrix is out of [-pi, pi] bounds that " + ...
            "this calculation is valid for!")
        q = [nan, nan, nan, nan];
    else
        % Equation B.10, Lynch, Modern Robotics
        q(1) = 0.5 * sqrt(1 + R(1,1) + R(2,2) + R(3,3));

        % Equation B.11, Lynch, Modern Robotics (Has unit quaternion form)
        % Current Equation is from Slide 11, W3-L1
        q(2) = 0.5 * (sgn(R(3,2) - R(2,3)) * ...
            sqrt(R(1,1) - R(2,2) - R(3,3) + 1));
        q(3) = 0.5 * (sgn(R(1,3) - R(3,1)) * ...
            sqrt(R(2,2) - R(3,3) - R(1,1) + 1));
        q(4) = 0.5 * (sgn(R(2,1) - R(1,2)) * ...
            sqrt(R(3,3) - R(1,1) - R(2,2) + 1));
    end
end

% Definition of SGN Function Slide 11, W3-L1
function output=sgn(x)
    if x >= 0
        output = 1;
    elseif x < 0
        output = -1;
    end
end