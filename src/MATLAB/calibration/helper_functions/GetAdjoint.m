function adjoint=GetAdjoint(T)

% Grab Rotation Matrix 
R = [T(1,1:3);
     T(2,1:3);
     T(3,1:3)];

% Grab Translation Vector
p = T(1:3,4);

% Create Skew Symmetric Translation Matrix
p_skew = VectorToSkewMatrix(p);

% Create Zero Matrix
zero_vec = zeros(3,3);

% Calculate Adjoint
adjoint = [R zero_vec;
           p_skew*R R];
end