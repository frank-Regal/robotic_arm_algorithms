%% Title:    THA 2, Jacobian Transpose Kinematics
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Iterates through steps from a starting configuration until it reaches the
% desired enpoint configuration
% Ref: Week 8 Lecture 1 slides 6 through 13
% Ref: Week 10 Lecture 1 slides 9 through 10
% Ref: Buss, Samuel "Introduction to Inverse Kinematics with
%      Jacobian Transpose, Pseudoinverse and Damped Least Squares methods",
%      2009

% Inputs:
%    Tsd: Transformation matrix between the desired location and the base frame
%    StartThetas: All theta values of the robots starting configuaration
%    InFigure: Value to create a figure to visualize the robots path


% Outputs:
%    theta: Theta values when robot reaches the endpoint

function theta=J_transpose_kinematics(Tsd,StartThetas, InFigure)

% Get Parameters of the robot (Kuka Quantec)
[s_hat,q,h,M,~]=GetKukaQuantecParams("body","false");

figure(InFigure);
% Get foward Kinematic of robot in body Frame
Tsb = FK_body(s_hat,q,h,M,StartThetas,'PlotMe');
hold on

% Get transformation matrix from body frame to desired endpoint
Tbd = InverseTransformMatrix(Tsb)*Tsd;

% Calculate the twist given the transformation matrix
Vb_bracket = GetMatrixLogOfTransform(Tbd);

% Convert to Vector Notation from Bracket Notation
Vb = ScrewBracketToVector(Vb_bracket);

% Break Twist into w and v components and get error values for each
Vbw = Vb(1:3);
Vbv = Vb(4:6);

% Calculate Initial Error
error_w = norm(Vbw);
error_v = norm(Vbv);

% Set acceptable error metrics
eps_w = 0.2;
eps_v = 6;

% Set theta to the starting theta values and the loop counter to one
theta = StartThetas;
count = 1;

% Iterate algorthim until the current robot configuration is within one of
% the acceptable error ranges
while (error_w > eps_w && error_v > eps_v && count < 500)
    fprintf("\n\n===================================================\n");
    disp("Iteration: " + count);

    % Create the body jacobian
    body_j = J_body(s_hat,q,h,theta);
    
    % Get Error Vector
    e_bar = [Vbw;Vbv];

    % Calculating Alpha 
    % Source: Buss, Samuel "Introduction to Inverse Kinematics with
    % Jacobian Transpose, Pseudoinverse and Damped Least Squares methods",
    % 2009
    A = body_j * body_j';
    B = A*e_bar;
    alpha =  dot(e_bar,B) / dot(B,B);

    % Calculating Delta Theta
    del_theta = alpha*body_j'*e_bar;

    % Recalculate Theta
    theta = theta + del_theta;

    % Get foward Kinematic of robot in body Frame
    Tsb = FK_body(s_hat,q,h,M,theta,'PlotMe');
    GetManipEllipse(Tsb,body_j,InFigure);
    hold on

    % Plot goal position
    scale_crd_frame = norm(M(1:3,4)) / 7;
    Plot3DAxis(Tsd,'goal',scale_crd_frame);
    hold on;
    
    % Get transformation matrix from body frame to desired endpoint
    Tbd = InverseTransformMatrix(Tsb)*Tsd;

    % Calculate the twist given the transformation matrix
    Vb_bracket = GetMatrixLogOfTransform(Tbd);

    % Break Twist into w and v components and get error values for each
    Vb = ScrewBracketToVector(Vb_bracket);
    Vbw = Vb(1:3);
    Vbv = Vb(4:6);
    error_w = norm(Vbw);
    error_v = norm(Vbv);
    
    % Pause for visualization
    pause(0);

    % Advance count
    count = count +1;
    
    %Replace figure if another iteration if comming
    if (error_w > eps_w && error_v > eps_v)
        clf(InFigure);
    end

end

end