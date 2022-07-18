%% Title:    THA 4, Part B - Distance, Joint Limits, & Rotation Constraints
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.05.12
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clear;
clc;

%% Setup Robot

% // Starting Joint Angles
%theta = [pi/6,0,0,0,pi/4,0]; % [rad]
theta = [pi/6,0,-pi/4,0,pi/4,0]; % [rad]

% // Plot Robot in Start Position
InFigure = 1;
figure(InFigure);
[s_hat,q,h,M,~] = GetKukaQuantecParams('space','false');
Tsb = FK_space(s_hat,q,h,M,theta,'PlotMe');
title("Translation and Rotation Constraints");
hold on;

% // Get Rotational Component
orig_rot = Tsb(1:3,1:3);
last_rot = orig_rot;

% // Setup Effector Tip
tip_vec = [0;100;0;1]; % tip vector from {b} to end effector tip {a}
t = Tsb*tip_vec;       % tip vector from {s} to end effector tip {a}

% plot line for tip vector
line([Tsb(1,4) t(1)], ...
    [Tsb(2,4) t(2)], ...
    [Tsb(3,4) t(3)], ...
    'color','m','linewidth', 3,'LineStyle','-');
hold on;

% plot dot on end effector tip
plot3(t(1),t(2),t(3),...
    '-o','Color','k','MarkerSize',6, 'MarkerFaceColor','m');
hold on;

%% Set Goal Point Waypoints

% // Set Goal X mm Away
% set_dist = [-22;  % [mm] x
%     -1;  % [mm] y
%     -2]; % [mm] z

set_dist = [-10;  % [mm] x
              5;  % [mm] y
             -2]; % [mm] z

% goal vector
p_goal = t(1:3) + set_dist;

% goal transform
p_goal_TF = [1 0 0 p_goal(1);
    0 1 0 p_goal(2);
    0 0 1 p_goal(3);
    0 0 0 1];

% Create a small path for robot
p_goal_TF_list = GetShortPath(p_goal_TF);
[p_rows,p_cols] = size(p_goal_TF_list); % get size

% Set Checker
IsGood = true;

%% Solve for Joint Angles and Command Robot to Move
for z = 1:p_rows/4
    
    % Test for ending waypoint traveling
    if (IsGood == true)

        % Grab Current Goal from Waypoints
        p_goal_TF = p_goal_TF_list((z*4)-3:(z*4),:);
        p_goal = p_goal_TF(1:3,4);

        % Plot goal position
        Plot3DAxis(p_goal_TF,'goal',120);
        hold on;

        % Check start distance
        start_dist = norm(p_goal - t(1:3));
        fprintf("\n");
        disp("Start Distance from goal: " + start_dist + " mm");
        end_dist = start_dist;

        % Setup Twist for Error Checking
        Tbd = InverseTransformMatrix(Tsb)*p_goal_TF;

        % Calculate the twist given the transformation matrix
        Vb_bracket = GetMatrixLogOfTransform(Tbd);

        % Break Twist into w and v components and get error values for each
        Vb = ScrewBracketToVector(Vb_bracket);
        Vbw = Vb(1:3);
        Vbv = Vb(4:6);
        error_w = 500;
        error_v = 500;

        % Set acceptable error metrics
        eps_w = 0.97;
        eps_v = 110;

        % Reset Plot
        clf(InFigure);

        % Set Checker
        count = 1;

        % Solve for Joint Angles Needed to Get to Goal
        % Reference: 
        % Ming Li et al., "Telerobotic Control by Virtual Fixturs ..."
        % John Hopkins University

        while (error_w > eps_w && error_v > eps_v && IsGood == true)
            
            % Get Space Jacobian for Current Location
            space_j = J_space(s_hat,q,h,theta);
            J_alpha = space_j(1:3,:);   % parse Jacobian for rotational component
            J_eplison = space_j(4:6,:); % parse Jacobian for translational component

            % Get skew matrix for t (tip vector from {s} to end effector tip {a})
            t_hat = VectorToSkewMatrix(t);
            
            % /////////////////////////////////////////////////////////////
            % Setup Objective Function (solving for delta q)
            C = eye(6,6); % lhs
            d = zeros(6,1); % rhs

            % /////////////////////////////////////////////////////////////
            % // Setup Distance Contraints

            % // Setup [-t_hat*J_alpha + J_eplison] into matrix form
            f1 = [-t_hat    , eye(3,3)  ;
                zeros(3,3), zeros(3,3)];
            h1 = space_j;
            J1 = f1*h1;    % Jacobian for First Constraint

            % // Setup A matrix from 22.3.1 (Ref. JHU Ming, Li)
            n = 10;           % Setup number of polyhedron vertices
            m = n;           % square only
            A1 = zeros(n,6); % Init the A1 Matrix

            % Fill A1 Matrix
            for i = 1:n
                A1_alpha = (i*2*pi)/n; % alpha param
                j = i;                 % to match literature
                A1_beta = (j*2*pi)/m;  % beta param

                % row vector for A
                A1(i,:) = [cos(A1_alpha)*cos(A1_beta), ...
                    cos(A1_alpha)*sin(A1_beta), ...
                    sin(A1_alpha), ...
                    0, 0, 0];
            end

            % Calculate b1
            distError = [t(1:3) - p_goal;  % translational error component
                0; 0; 0];         % rotational error component
            A1_w_distError = A1*distError; % A1 multiplied by distance error

            eps1 = sqrt(3);  % [mm] distance threshold
            epsVec = ones(length(A1_w_distError),1)*eps1;

            b1 = epsVec - A1_w_distError;  % rhs of constraint
            RhsConst1 = b1;
            LhsConst1 = A1*J1; % equation 22.4

            [c1Rows,c1cols] = size(LhsConst1);

            if (c1Rows < 6)
                num_rows_mis = 6 - c1Rows;
                LhsConst1 = [LhsConst1;zeros(num_rows_mis,6)];
                RhsConst1 = [RhsConst1;zeros(num_rows_mis,1)];
            end

            % /////////////////////////////////////////////////////////////
            % // SETUP ANGLE CONSTRAINT

            % Provided Joint Constraints by Kuka
            qmax = [180;45;150;350;125;350]*(pi/180);
            qmin = [-180;-145;-130;-350;-125;-350]*(pi/180);

            LhsConst2 = [ eye(6,6);
                -eye(6,6)];
            RhsConst2 = [(qmax - theta'); (theta' - qmin)];

            % /////////////////////////////////////////////////////////////
            % // SETUP ROTATIONAL CONSTRAINT

            A3 = zeros(n,6);
            for j = 1:n
                A3_alpha = (i*2*pi)/n;
                A3_beta = (i*2*pi)/m;
                A3(i,:) = [0 , 0 , 0 , cos(A3_alpha)*cos(A3_beta) ,...
                           cos(A3_alpha)*sin(A3_beta) , sin(A3_alpha)];
            end

            % Grab Euler Angles
            [lR,lP,lY] = RotToRPYAngles(last_rot);
            [OR,OP,OY] = RotToRPYAngles(orig_rot);

            % Get Last and Current Euler Angles
            last_euler = ([lR,lP,lY]/norm([lR,lP,lY]))';
            orig_euler = ([OR,OP,OY]/norm([OR,OP,OY]))';

            % Angles
            angles = cross(orig_euler,last_euler);
            del_rot = [0;0;0; angles];

            rot_error_eps = pi/6;
            b3 = (ones(n,1)*rot_error_eps) - A3*del_rot;

            RhsConst3 =  b3;
            LhsConst3 = A3*J1; % equation 22.4

            % /////////////////////////////////////////////////////////////
            % // Finalize All Constraints
            A = [LhsConst1;LhsConst2;LhsConst3];
            b = [RhsConst1;RhsConst2;RhsConst3];

            % // Solve for Delta Theta
            options = optimoptions(@lsqlin,'Display','off');
            delta_theta = lsqlin(C,d,A,b,[],[],[],[],[],options);
           
            % Make sure least squares produced a solution
            if (isempty(delta_theta))
                IsGood = false;
            else
                % Add the theta to previous theta
                theta_temp = theta' + delta_theta;
                theta = theta_temp';
            end

            % /////////////////////////////////////////////////////////////
            % REPLOT ARM
            Tsb = FK_space(s_hat,q,h,M,theta,'PlotMe');
            title("Translation and Rotation Constraints");
            hold on;

            % Get Rotational Component
            last_rot = Tsb(1:3,1:3);

            % Setup Effector Tip
            tip_vec = [0;100;0;1]; % tip vector from {b} to end effector tip {a}
            t = Tsb*tip_vec;       % tip vector from {s} to end effector tip {a}

            % plot line for tip vector
            line([Tsb(1,4) t(1)], ...
                [Tsb(2,4) t(2)], ...
                [Tsb(3,4) t(3)], ...
                'color','m','linewidth', 3,'LineStyle','-');
            hold on;

            % plot dot on end effector tip
            plot3(t(1),t(2),t(3),...
                '-o','Color','k','MarkerSize',6, 'MarkerFaceColor','m');
            hold on;

            % // Plot goal position
            Plot3DAxis(p_goal_TF,'goal',120);
            hold on;

            % /////////////////////////////////////////////////////////////
            % Calculate Error for While Loop
            Tbd = InverseTransformMatrix(Tsb)*p_goal_TF;

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

            % /////////////////////////////////////////////////////////////
            % Check end distance
            end_dist = norm(p_goal - t(1:3));
            disp("End Distance from goal: " + end_dist + " mm");

            count = count +1;
            if (count == 5)
                IsGood = false;
            end

        end % while-loop end

        % Replace figure if another iteration is comming
        if (error_w > eps_w && error_v > eps_v && IsGood == true)
            clf(InFigure);
        end

    end % if statement end

end % for-loop end
