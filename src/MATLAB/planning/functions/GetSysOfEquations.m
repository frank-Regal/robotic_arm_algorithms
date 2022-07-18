function eqn=GetSysOfEquations(j)

[rows,cols] = size(j);
r = 0;

for i = 1:rows
    for k = 1:cols
        r = r + j(i,k);
    end
    eqn(i) = r == 0;
    r = 0;
end

end