%% Title:    THA 2, Barrett Technology WAM Robot Parameters for Body and Space Frame
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Returns the Space or Body Frame Parameters for the Kuka Quantec Robot
% Ref: Techanical Data Sheet for KR 120 R1800 nano

% Inputs:
%    frame: Determination for whether the space frame or body frame points
%           and screw axes should be used 
%    symbolic_theta: Determination of whether the theatas should be 
%    symbolic variables 

% Outputs:
%    s_hat: 3xn matrix of screw axes for each joint
%    q: 3xn matrix of vectors defining the screw axis for each joint
%    h: 1xn vector defined for pitch for each joint
%    M: 4x4 homogenous transformation matrix defining the body frame {b} w.r.t.
%       the base frame {s}
%    theta: 1xn vector defining the angles of rotation for each joint

function [s_hat,q,h,M,theta]=GetBarrettTechWAMParams(frame,symbolic_theta)

% Link Lengths
L1 = 550; % [mm]
L2 = 300; % [mm]
L3 = 60;  % [mm]
W1 = 45;  % [mm]

% Define Screw Axes
% Angular Velocity Rotations
s1 = [0;0;1];
s2 = [0;1;0];
s3 = [0;0;1];
s4 = [0;1;0];
s5 = [0;0;1];
s6 = [0;1;0];
s7 = [0;0;1];
s_hat = [s1, s2, s3, s4, s5, s6, s7];

% Pitch Vector
h = zeros(1,length(s_hat(1,:)));

% Homogenous Transformation of Body Frame w.r.t. Base Frame
M = [1 0 0 0;
     0 1 0 0;
     0 0 1 L1+L2+L3;
     0 0 0 1];

% Theta Vector
switch (symbolic_theta)
    case('true')
        syms t1 t2 t3 t4 t5 t6 t7
        theta = [t1, t2, t3, t4, t5, t6, t7];
    case('false')
        theta = zeros(1,length(s_hat(1,:)));
end

switch (frame)
    case ("space")
        % Define Vectors to Screw Axis
        q1 = [0;0;0];
        q2 = [0;0;0];
        q3 = [0;0;0];
        q4 = [W1;0;L1];
        q5 = [0;0;L1+L2];
        q6 = [0;0;L1+L2];
        q7 = [0;0;L1+L2];
        q = [q1,q2,q3,q4,q5,q6,q7];

    case ("body")
        % Define Vectors to Screw Axis
        q1 = [0;0;-(L1+L2+L3)];
        q2 = [0;0;-(L1+L2+L3)];
        q3 = [0;0;-(L1+L2+L3)];
        q4 = [W1;0;-(L2+L3)];
        q5 = [0;0;-L3];
        q6 = [0;0;-L3];
        q7 = [0;0;-L3];
        q = [q1,q2,q3,q4,q5,q6,q7];
end

end