function Adjoint=GetAdjointSpaceToBody(M)
    
R = [M(1,1:3);
     M(2,1:3);
     M(3,1:3)];

p = M(1:3,4);

p_bracket = VectorToSkewMatrix(p);

zero_mat = zeros(3,3);

Adjoint = [R' zero_mat;
           -R'*p_bracket R'];

end