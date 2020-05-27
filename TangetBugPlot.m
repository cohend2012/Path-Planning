function [n,rphdl] = TangetBugPlot(xpos,ypos,robothdl,rangehdl,thetaMhdl,n,thetaM,theta,newRho,indexOfedge)
%TangetBugPlot this is the ploting funiton
%  obs is the obstacles in a aray should look like obs = [700 1000 -600 400; 
% 800 600 800];
% qs is the start configraiton needs to be qs=(o ( mm),
% 0(mm),0deg/rad)-need one 

% find howmany row
    
    rphdl = [];
    
    set(robothdl,'XData',xpos(1:n));
    set(robothdl,'YData',ypos(1:n));
    
    set(thetaMhdl,'XData',[xpos(n),xpos(n)+200*cos(thetaM)]);
    set(thetaMhdl,'YData',[ypos(n),ypos(n)+200*sin(thetaM)]);
    
    xmatrix = zeros(length(newRho),2);
    
    for i = 1:length(newRho)
        
        xpos_rp = [xpos(n),xpos(n) + newRho(i)*cos((indexOfedge(i)-121)*pi/180+theta)];
        ypos_rp = [ypos(n),ypos(n) + newRho(i)*sin((indexOfedge(i)-121)*pi/180+theta)];        
        
        rphdl(i) = plot(xpos_rp,ypos_rp);
        
    end
    
    %set(rphdl,'XData',[xpos_rp,xpos_rp+200*cos(indexOfedge(i)*pi/180)]);
    %set(rphdl,'YData',[ypos_rp,ypos_rp+200*sin(indexOfedge(i)*pi/180)]);
    
    ViewRange = linspace((120*pi/180),-(120*pi/180),100);
    R = 1000;
    xView = R*cos(ViewRange+theta) + xpos(n);
    yView = R*sin(ViewRange+theta) + ypos(n); 
    set(rangehdl,'XData',xView);
    set(rangehdl,'YData',yView);
    
    drawnow;
    
    %disp(xpos_rp)
    %disp(xpos)
    

end