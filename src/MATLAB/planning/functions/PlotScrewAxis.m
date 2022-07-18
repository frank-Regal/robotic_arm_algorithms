%% Title:    THA 1, Programming Assignment Helper Function
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Used to plot a screw axis
%
% Input:
%    q: 3x1 vector from frame to screw axis
%    s: 3x1 unit vector for screw axis
%    length: scaler multiplier for how long you want your screw axis

function PlotScrewAxis(FK, Cur_Trans_TF, s_hat, scale)

    Cur_Trans_TF = FK*Cur_Trans_TF;
    rot_temp = [Cur_Trans_TF(1,1) Cur_Trans_TF(1,2) Cur_Trans_TF(1,3);
                Cur_Trans_TF(2,1) Cur_Trans_TF(2,2) Cur_Trans_TF(2,3);
                Cur_Trans_TF(3,1) Cur_Trans_TF(3,2) Cur_Trans_TF(3,3)];
    q_temp = [Cur_Trans_TF(1,4) ; Cur_Trans_TF(2,4) ; Cur_Trans_TF(3,4)];
    s_temp = rot_temp*s_hat;
    s_temp = s_temp*scale/2;
    line([q_temp(1)-s_temp(1) q_temp(1)+s_temp(1)] ,[q_temp(2)-s_temp(2) q_temp(2)+s_temp(2)],[q_temp(3)-s_temp(3) q_temp(3)+s_temp(3)], 'color', 'k', 'linewidth', 1, 'LineStyle','--' )
    


end
