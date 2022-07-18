function [vectors,lambdas]=GetJacobianEigs(J,velocity)
    
if (velocity == "angular")
    % Get Angular Velocities from Jacobian Input
    J = J(1:3,:);
elseif (velocity == "linear")
    % Get Linear Velocities from Jacobian Input
    J = J(4:6,:);
end
    % Transpose the Matrix of Angular Velocities
    J_T = transpose(J);
    
    % Calcalate the JxJ(transpose) 
    A = J*J_T;
    
    % Get Eigenvalues of JxJ(transpose)
    [vecs,vals] = eig(A);
   
    x_comp = vecs(:,1);
    y_comp = vecs(:,2);
    z_comp = vecs(:,3);
    vectors = [x_comp, y_comp, z_comp];
    

    lam1 = vals(1,1);
    lam2 = vals(2,2);
    lam3 = vals(3,3);
    lambdas = [lam1, lam2, lam3];

end