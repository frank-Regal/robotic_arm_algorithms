function log_T=GetMatrixLogOfTransform(T)

R = T(1:3,1:3);
[rows,cols] = size(R);
I = eye(rows,cols);
p = T(1:3,4);

% Check if R is idenity
R_is_idenity = true;
for i = 1:rows
    for j = 1:cols
        if (R(i,j) > -0.0001 && R(i,j) < 0.0001)
            R(i,j) = 0;
        end
        if abs(R(i,j)) ~= I(i,j)
            R_is_idenity = false;
        end
    end
end

if (R_is_idenity == true)
    w = zeros(3,1);
    v = p/norm(p);
    theta = norm(p);
    w_brack = VectorToSkewMatrix(w);
else 
    [w_brack,theta] = GetLogOfRot(R);
    G_inv = ((1/theta)*eye(3)) - ((1/2)*w_brack) + ((1/theta) - (1/2)*cot(theta/2))*(w_brack*w_brack);
    v = G_inv*p;
end

log_T = [w_brack*theta v*theta;
         0 0 0 0];