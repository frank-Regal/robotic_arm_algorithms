%% Title:    THA 2, Space Frame Foward Kinematics
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Create the Foward Kinematics Matrix for the current Robot Configuration
%       In the Space Frame
% Ref: Week 6 Lecture 1 Slides 2 and 10

% Inputs:
%    s_hat: 3xn matrix of screw axes for each joint
%    q: 3xn matrix of vectors defining the screw axis for each joint
%    h: 1xn vector defined for pitch for each joint
%    M: 4x4 homogenous transformation matrix defining the body frame {b} w.r.t.
%       the base frame {s}
%    theta: 1xn vector defining the angles of rotation for each joint
%    PlotMe: variable to define wether the funciotn should produce a plot
%            of the foward kinematics

% Outputs:
%    output: Foward Kinematics of the Current Robot Configuration

function FK=FK_space(s_hat, q, h, M, theta, PlotMe)

% Check if Optional Args were passed
if (~exist('PlotMe','var'))
    PlotMe = false;
else 
    PlotMe= true;
end

% Init 
[~,cols] = size(s_hat);
FK = eye(4,4);

% Get Screw Representation
S_Screws = GetScrewVector(s_hat,q,h);

% Plot Base Frame Axis
if (PlotMe)
    scale_crd_frame = norm(M(1:3,4)) / 7;
    %figure();
    axis square;
    Plot3DAxis(FK,'origin', scale_crd_frame);
    hold on;
end

% Loop Through Joints
for i = 1:cols
    % Save the Transform from Previous Joint
    FK_prev = FK;

    % Pre-Multiple Forward Kinematics by the Screw Exponential
    FK = FK * GetScrewExp(S_Screws(:,i),theta(i));

    % Plotting Links In Between Joints
    % Translation component between current joint and base frame
    Cur_Trans_TF = [1 0 0 q(1,i);
                    0 1 0 q(2,i);
                    0 0 1 q(3,i);
                    0 0 0 1];

    % Transform of the Current Joint to the Space Frame
    Cur_TF_to_base = FK*Cur_Trans_TF;

    % Ignore if at first joint
    if i ~= 1
        % Translation component between previous joint and the base frame
        Prev_Trans_TF = [1 0 0 q(1,i-1);
                         0 1 0 q(2,i-1);
                         0 0 1 q(3,i-1);
                         0 0 0 1];
        if (PlotMe)
            % Plot links
            Prev_TF_to_base = FK_prev*Prev_Trans_TF;
            Plot3DLinks(Prev_TF_to_base,Cur_TF_to_base);
            hold on
        end
    end
    
    if (PlotMe)
        % Plot the 3D Axes of Current Joint on Loop
        Plot3DAxis(Cur_TF_to_base,'base', scale_crd_frame);
        hold on

        % Plotting Screw Axis of Current Joint
        PlotScrewAxis(FK,Cur_Trans_TF,s_hat(:,i),scale_crd_frame*4)
        hold on;
    end
end

% Output
FK = FK*M;

if (PlotMe)
    Plot3DAxis(FK,'base', scale_crd_frame);
    axis([-2000 2000 -2000 2000 0 2500]);
    grid on;
    title('Space Frame Foward Kinematics');
    xlabel('x axis [mm]');
    ylabel('y axis [mm]');
    zlabel('z axis [mm]');
end

end