function [aInt, xi, yi] = arcInt(c1, c2)
%   arcInt(c1, c2) calculate the first intersection between two arcs.
%
%   Inputs:   
%        c1, c2 : (1x5) arc arrays with format:
%                 c = [xc, yc, r, theta_min, theta_max] 
%                        xc : x coordinate of the arc center;
%                        yc : y coordinate of the arc center;
%                         r : radius of the arc;
%                 theta_min : start arc angle in the range [-pi pi];
%                 theta_max : end arc angle in the range [-pi pi].
%
%   Outputs:
%         aInt  : boolean, true if at least one intersecton is detected;
%            xi : x coordinate of first intersection;
%            yi : y coordinate of first intersection.
%
%   Note: start and end arc angles are defined counterclockwise

    
%% Calculate Intersection
    aInt = false;
    arcThresh = pi/6;
    xi = 0;
    yi = 0;
    
    dx2 = (c2(1) - c1(1))^2;
    dy2 = (c2(2) - c1(2))^2;
    rho2 = dx2 + dy2;
    if (rho2==0) && (c1(3)==c2(3))
        % Arcs are overlapping
        return
    end
    c = (c1(3)^2 - c2(3)^2 - dx2 - dy2)/(2*c2(3));
    rho = sqrt(rho2);
    dCoRho = c/rho;
    if abs(dCoRho) > 1
        % No intersection between circles
        return
    end
    dx = c2(1) - c1(1);
    dy = c2(2) - c1(2);
    beta = atan2(dx, dy);
    asinCoRho = asin(dCoRho);
    theta2 = [asinCoRho, pi*sign(asinCoRho) - asinCoRho] - beta;
    theta2 = atan2(sin(theta2), cos(theta2));
    theta1 = atan2((dy+c2(3)*sin(theta2)), (dx+c2(3)*cos(theta2)));
    if c1(4)>0 && c1(5)<0
        c1(5) = 2*pi+c1(5);
        theta1(theta1<0) = theta1(theta1<0)+2*pi;
    end
    if c2(4)>0 && c2(5)<0
        c2(5) = 2*pi+c2(5);
        theta2(theta2<0) = theta2(theta2<0)+2*pi;
    end
    mask = theta1>c1(4) & theta1<c1(5) & theta2>c2(4) & theta2<c2(5);
    theta1 = theta1(mask);
    theta2 = theta2(mask);
    if isempty(theta1)
        % No intersection between arcs
        return
    end
    v1 = c1(3)*[cos(theta1(1)),sin(theta1(1))];
    v2 = c2(3)*[cos(theta2(1)),sin(theta2(1))];
    alpha = acos((v1(1)*v2(1)+v1(2)*v2(2))/(c1(3)*c2(3)));
    if abs(alpha) < arcThresh
        % Intersection angle less than threshold (pi/6)
        return
    else
        aInt = true;
        xi = c1(1)+v1(1); 
        yi = c1(2)+v1(2);
    end    
end


