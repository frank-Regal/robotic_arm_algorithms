function T = GetShortPath(p_goal_TF)

% x-y plan quarter circle (Radius)
r = norm(p_goal_TF(1:2,4));

x = p_goal_TF(1,4);
y = p_goal_TF(2,4);
z = p_goal_TF(3,4);
t = atan2(x,y);

% Set Spacing
%n = 8;
n = 50;

% break up arc
t = linspace(t,t-pi/2,n);


% Init Output T
T = zeros(4*n,4);

% drop = linspace(0,100,n);
drop = linspace(0,700,n);
% Build out Transform
for i = 1:n
    x = r*sin(t(i));
    y = r*cos(t(i));
    z_temp = z - drop(i) ;
    T((i*4)-3:(i*4),:) = [p_goal_TF(1:3,1:3),[x;y;z_temp];zeros(1,3),1];
%     plot3(T((i*4)-3,4),T((i*4)-2,4),T((i*4)-1,4),'-');
%     hold on;
end

end