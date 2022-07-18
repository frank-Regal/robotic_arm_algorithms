function S=GetScrewVector(s_hat,q,h)

[~,cols] = size(s_hat);

S = zeros(6,cols);

% Defining Screw Axis Variables Needed for Calculations
for i = 1:cols
    % angular velocity
    w = s_hat(:,i);
    % skew symmetric matric for screw vector
    s_skew = VectorToSkewMatrix(s_hat(:,i));
    % linear velocity
    v = -s_skew*q(:,i) + h(i)*s_hat(:,i);
    % Output 6x1 screw vector
    S(:,i) = [w;v];
end
end