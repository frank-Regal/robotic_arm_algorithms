function [omega_bracket,theta_out]=GetLogOfRot(R)


I = eye(3);

% Check if R is idenity
R_is_idenity = true;
for i = 1:3
    for j = 1:3
        if (R(i,j) > -0.0001 && R(i,j) < 0.0001)
            R(i,j) = 0;
        end
        if abs(R(i,j)) ~= I(i,j)
            R_is_idenity = false;
        end
    end
end

   % if no rotation / Rotation is idenity
    if (R_is_idenity)
        disp("ERROR: No Rotation theta: 0; omega: Undefined");
        theta = 0;
        omega_skew = nan;

    % not a unique solution, this is surjective onto the space of rotations
    elseif trace(R) == -1
        theta = pi;
        
        option1 = (1/sqrt(2*(1 + R(3,3))));
        option2 = (1/sqrt(2*(1 + R(2,2))));
        option3 = (1/sqrt(2*(1 + R(1,1))));
    
        if ~isinf(option1)
            omega =  option1 * [  R(1,3);   R(2,3); (1+R(3,3))];
        elseif ~isinf(option2)
            omega =  option2 * [  R(1,2); 1+R(2,2);     R(3,2)];
        elseif ~isinf(option3)
            omega =  option3 * [1+R(1,1);   R(2,1);     R(3,1)];
        else
            disp("ERROR: RotToAxisAngle Function");
        end 

        omega_skew = VectorToSkewMatrix(omega);
    % otherwise
    else 
        theta = acos(0.5*( trace(R)- 1));

        % Make sure theta is in range
        while theta >= pi
            theta = theta - pi;
        end
        omega_skew = (1/(2*sin(theta))) * (R-R');
    end
    
    theta_out = theta;
    omega_bracket = omega_skew;
end