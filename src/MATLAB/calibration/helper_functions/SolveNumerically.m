function solution=SolveNumerically(sys_of_eqns,vars)

for n = 1:10
    solution = vpasolve(sys_of_eqns,vars,'Random',true)
end