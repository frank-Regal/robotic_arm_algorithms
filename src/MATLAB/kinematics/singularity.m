%% Title:    THA 2, Singularity Calculations 
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Either Calculates a list of some singulatities or whether the given
% configuration is a singularity
% Ref: Week 7 Lecture 1 slide 14

% Inputs:
%    Jacobian: the Jacobian of interest
%    theta: the thetas of the configuraiton of interest (symbolic if
%           multiple singularites are desired
%    FindMultiple: Value for deciding whether to find a set of singulaties
%                  or if a given configuraiton is a singularity

% Outputs:
%    OutConfig: Theta values for a singular point
%    IsSingular: True or false statement for whether the given point is
%                singular

function [OutConfig, IsSingular]=singularity(Jacobian,theta,FindMultiple)

% Get Size of Jacobian
[rows,cols] = size(Jacobian);

% Check if Optional Args were passed
if (~exist('FindMultiple','var'))
    FindMultiple = false;
else 
    FindMultiple = true;
end

% If Jacobian is Square and User wants a list of singularities
if (FindMultiple)
    IsSingular = true;
    % Iterate until 10 singularities are found
    for i = 1:3
        % If Jacobian is square
        if (rows == cols)
            % Take Determinant of the Jacobian
            d_jac = det(Jacobian);
            % Set determinant equal to zero
            eqn = d_jac == 0;
            % Solve equation, which will result in a singularity
            % configuration
            OutConfig(i) = vpasolve(eqn,[theta(1),theta(2),theta(3),theta(4),theta(5),theta(6)],'Random',true);
        end
    end

    % Determine if input Jacobian is singular with theta's already incorporated
elseif (~FindMultiple)
    OutConfig = theta;
    m = rank(Jacobian);
    % // If Jacobian is Max Rank, Not at a singularity
    if (m == min(rows,cols))
        IsSingular = false;
    % // If Jacobian is Not Max Rank, It is at a Singularity
    else
        IsSingular = true;
    end
end
PrintResults(IsSingular);
end


function []=PrintResults(IsSingular)
if (IsSingular)
    fprintf("\nAt Singular Configuration\n");
else
    fprintf("\nNot a Singular Configuration\n");
end
end