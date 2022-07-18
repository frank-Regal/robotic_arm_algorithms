function T_Body=FKBody(s_hat,q,h,M,theta)

[~,cols] = size(s_hat);

T_Body = M;

B_Screws = GetScrewVector(s_hat,q,h);

for i = 1:cols
    % Save Previous Transform for Plotting
    T_Prev = T_Body;
    
    T_Body = T_Body * GetScrewExp(B_Screws(:,i),theta(i));
end
