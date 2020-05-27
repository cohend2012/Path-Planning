function [obshdl] = TangetBugPlotObs(obs)
%TangetBugPlot this is the ploting funiton
%  obs is the obstacles in a aray should look like obs = [700 1000 -600 400; 
% 800 600 800];
% qs is the start configraiton needs to be qs=(o ( mm),
% 0(mm),0deg/rad)-need one 

% find howmany rows
  
    [m,n] = size(obs);
    if n<4
        disp('Check Matrix look and obs and m,n')
    end 

    for row=1:m
        x_m=obs(row,1);
        y_m=obs(row,3);
        w_m=obs(row,2)-obs(row,1);
        h_m=obs(row,4)-obs(row,3);
        obshdl(row)=rectangle('Position',[x_m y_m w_m h_m],'FaceColor','r');
    end
    

end

