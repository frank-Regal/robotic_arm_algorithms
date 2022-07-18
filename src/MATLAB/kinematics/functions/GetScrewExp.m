function S_exp=GetScrewExp(S,theta)

% Get Components
w = S(1:3); % angular velocity
v = S(4:6); % linear velocity
 
% Create an identity matrix for calcs
I = eye(3,3);

% //Proposition 3.25: Modern Robotics
% //Rotation
if norm(w) == 1
    % skew symmetric matrix for w. Redefined for variable consitency
    w_skew = VectorToSkewMatrix(w);

    % square of skew symmetrtic matrix
    w_skew_sqr = w_skew*w_skew;

    % Solving for exp([S]Theta)
    % Equation 3.51 Modern Robotics, Lynch, Rodrigues Formula, Page 84
    elm11 = I + sin(theta)*w_skew + (1-cos(theta))*(w_skew_sqr);

    % Equation 3.88 Modern Robotics, Lynch
    elm12 = (I*theta + (1-cos(theta))*w_skew + (theta - sin(theta))*w_skew_sqr)*v;
    elm21 = [0 0 0];
    elm22 = 1;

    % Ouput for Matrix Form
    S_exp = [elm11 elm12;
             elm21 elm22];
% //Pure Translation
elseif norm(w) == 0 && norm(v) == 1
    S_exp = [I v*theta;
             zeros(1,3) 1];
% //Error
else 
    fprintf('ERROR: Screw Vectors Defined Wrong');
end

end
