function [obsDet, rho] = testObsVec(q, theta_us, res, R, obs)
%   testObsVec(p, theta, res, R, rho) measures the distance from the robot
%   to the closest obstacle.
%
%   Inputs:   
%             q : (1x3) array [x,y,theta] configuration of the robot
%      theta_us : sensor relative angle in rad
%           res : sensor resolution
%             R : sensor range
%           obs : (nx4) array [x1min,x1max,y1min,y1max; ...] of n
%                 rectangular obstacles
%
%   Outputs:
%        obsDet : boolean, true if at least one obstacle is detected
%           rho : distance to the closest obstacle; 
%                 if no obstacles are detected returns R
              
    r = (2*res:res:R)';
    q = repmat(q(1:2), length(r),1) + [r*cos(theta_us+q(3)), r*sin(theta_us+q(3))];
    mask = q(:,1) >= obs(1,1) & q(:,1) <= obs(1,2) & q(:,2) >= obs(1,3) & q(:,2) <= obs(1,4);
    for i = 2:size(obs,1)
         mask = mask | (q(:,1) >= obs(i,1) & q(:,1) <= obs(i,2) & q(:,2) >= obs(i,3) & q(:,2) <= obs(i,4));
    end
    if max(mask) == 1
        obsDet = true;
        rho = r(find(mask, 1, 'first')) - res;
    else
        obsDet = false;
        rho = R;
    end

end