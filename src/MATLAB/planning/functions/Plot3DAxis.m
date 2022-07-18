%% Title:    THA 2, Programming Assignment Helper Function
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Used for plotting unit axes of coordinate transforms
%
% Input:
%     T: 4x4 homogeneous transform
% Output:
%     style: string for either 'origin' or 'base' transform

function Plot3DAxis(T,style,scale)

switch (style)
    case ('origin')
        origin = [T(1,4) T(2,4) T(3,4)]';
        x_axis = [T(1,1) T(2,1) T(3,1)]'*scale;
        y_axis = [T(1,2) T(2,2) T(3,2)]'*scale;
        z_axis = [T(1,3) T(2,3) T(3,3)]'*scale;

        plot3(origin(1),origin(2),origin(3),'k.')
        hold on

        line([origin(1) origin(1)+x_axis(1)],[origin(2) origin(2)+x_axis(2)],[origin(3) origin(3)+x_axis(3)], 'color', 'k', 'linewidth', 4)
        line([origin(1) origin(1)+y_axis(1)],[origin(2) origin(2)+y_axis(2)],[origin(3) origin(3)+y_axis(3)], 'color', 'k', 'linewidth', 4)
        line([origin(1) origin(1)+z_axis(1)],[origin(2) origin(2)+z_axis(2)],[origin(3) origin(3)+z_axis(3)], 'color', 'k', 'linewidth', 4)
            
    case ('base')
        origin = [T(1,4) T(2,4) T(3,4)]';
        x_axis = [T(1,1) T(2,1) T(3,1)]'*scale;
        y_axis = [T(1,2) T(2,2) T(3,2)]'*scale;
        z_axis = [T(1,3) T(2,3) T(3,3)]'*scale;

        plot3(origin(1),origin(2),origin(3),'r.')
        hold on

        line([origin(1) origin(1)+x_axis(1)],[origin(2) origin(2)+x_axis(2)],[origin(3) origin(3)+x_axis(3)], 'color', 'r', 'linewidth', 2);
        line([origin(1) origin(1)+y_axis(1)],[origin(2) origin(2)+y_axis(2)],[origin(3) origin(3)+y_axis(3)], 'color', 'g', 'linewidth', 2);
        line([origin(1) origin(1)+z_axis(1)],[origin(2) origin(2)+z_axis(2)],[origin(3) origin(3)+z_axis(3)], 'color', 'b', 'linewidth', 2);
        % Plot Screw Axis

     case ('goal')
        origin = [T(1,4) T(2,4) T(3,4)]';
        x_axis = [T(1,1) T(2,1) T(3,1)]'*scale;
        y_axis = [T(1,2) T(2,2) T(3,2)]'*scale;
        z_axis = [T(1,3) T(2,3) T(3,3)]'*scale;

        plot3(origin(1),origin(2),origin(3),'g.')
        hold on

        line([origin(1) origin(1)+x_axis(1)],[origin(2) origin(2)+x_axis(2)],[origin(3) origin(3)+x_axis(3)], 'color', '#D95319', 'linewidth', 4)
        line([origin(1) origin(1)+y_axis(1)],[origin(2) origin(2)+y_axis(2)],[origin(3) origin(3)+y_axis(3)], 'color', '#D95319', 'linewidth', 4)
        line([origin(1) origin(1)+z_axis(1)],[origin(2) origin(2)+z_axis(2)],[origin(3) origin(3)+z_axis(3)], 'color', '#D95319', 'linewidth', 4)

    case ('endEffector')

        origin = [T(1,4) T(2,4) T(3,4)]';
        x_axis = [T(1,1) T(2,1) T(3,1)]'*scale;
        y_axis = [T(1,2) T(2,2) T(3,2)]'*scale;
        z_axis = [T(1,3) T(2,3) T(3,3)]'*scale;

        plot3(origin(1),origin(2),origin(3),'r.')
        hold on

        %line([origin(1) origin(1)+x_axis(1)],[origin(2) origin(2)+x_axis(2)],[origin(3) origin(3)+x_axis(3)], 'color', 'r', 'linewidth', 2);
        line([origin(1) origin(1)+y_axis(1)],[origin(2) origin(2)+y_axis(2)],[origin(3) origin(3)+y_axis(3)], 'color', 'g', 'linewidth', 4);
       % line([origin(1) origin(1)+z_axis(1)],[origin(2) origin(2)+z_axis(2)],[origin(3) origin(3)+z_axis(3)], 'color', 'b', 'linewidth', 2);

end

end