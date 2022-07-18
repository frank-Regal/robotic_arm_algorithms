%% Title:    THA 2, Body Frame Jacobian
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Creates the Body Frame Jacobian of the Robot at the Current Configuration
% Ref: Week 7 Lecture 1 slides 10 through 12

% Inputs:
%    s_hat: 3xn matrix of screw axes for each joint
%    q: 3xn matrix of vectors defining the screw axis for each joint
%    h: 1xn vector defined for pitch for each jointframe: Determination for whether the space frame or body frame points
%           and screw axes should be used 
%    theta: 1xn vector defining the angles of rotation for each joint
%    MakeSymbolic: Variable determining whether the Thetas in the Jacobian
%    should be symbolic

% Outputs:
%    BaseJacobian: The created body frame Jacobian 

function BaseJacobian=J_body(s_hat, q, h, theta, MakeSymbolic)

% Check if Optional Args were passed
if (~exist('MakeSymbolic','var'))
    MakeSymbolic = false;
else 
    MakeSymbolic = true;
end

% Init 
[~,cols] = size(s_hat);

% User Input to Make Jacobian Symbolic or Not
if (MakeSymbolic)
    BaseJacobian = sym(zeros(6,cols));
else
    BaseJacobian = zeros(6,cols);
end

B_screws = GetScrewVector(s_hat,q,h);

% Loop through each joint
for i = 1:cols
  
    % Set Screw Vector of Joint 1 Equal to First Column in Jacobian
    if (i == cols)
        BaseJacobian(:,i) = B_screws(:,i);
    % Otherwise 
    else
        % Calculate the screw exponential for each joint from n down to i
        for n = cols:-1:i+1
            cur_S_exp = GetScrewExp(-B_screws(:,n),theta(n));
            % Multiple each screw exp together
            if (n == cols)
                T = cur_S_exp;
            else
                T = T * cur_S_exp;
            end
        end
        % Get Adjoint of the multiplied screw exp and multiple by the
        % current joint
        BaseJacobian(:,i) = GetAdjoint(T)*B_screws(:,i);
    end
end

end