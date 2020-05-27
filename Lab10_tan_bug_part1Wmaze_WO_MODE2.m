
% Daniel Cohen
% ME 406L  Lab 7
%Tan Bug with Mode 2

clear; clc;

scan = 0;
theta = 0;        % rad
thetaM = 0;
indexOfedge = 0;
x = 250;            % mm
y = 250;            % mm
ii = 0;
xs = 250;           % mm
ys = 250;           % mm
thetas = 0;       % rad
masteri = 0;
xg = 1650;        % mm 1650
yg = 1650;         % mm 1650
thetag = 0;   % rad
xpos = x;
ypos = y;
xpos_rp = 0;
ypos_rp= 0;
n = 1;
qs = [xs,ys,thetas];           % Config of robot
qg = [xg,yg,thetag];
theta_us = (-120:1:120).*(pi/180);% deg2rad does not work on this computer 
% Relative angle of sensor (in rad)
res = 5;                       % Resolution (mm) 
R = 1000;%5000; %1000                      % Range (mm)
         %(nx4) array [x1min,x1max,y1min,y1max;...] of n rectangular obstacles
wT = 175; % 50
obs = [      0 2440       0   wT;
             0   wT       0 2440;
             0 2440 2440-wT 2440;
       2440-wT 2440       0 2440;
             0  720 1030-wT 1030
        720-wT  720     470 1030;
        720-wT  720    1570 2440;
           460  720 1580-wT 1580;
       1240-wT 1240       0  450;
       1240-wT 1980  910-wT  910;
       1260-wT 1980 1420-wT 1420;
       1260-wT 1260    1420 1950;
       1980-wT 1980    1420 1950;
       1260-wT 1520 1950-wT 1950]; 

ds = 10;%15    % mm dist setep changed to 20~based on Dr. G recomdation 0
Kp = .157;% .157
obsDet = 0; % Assume no obstacle detected initially

% INITIALIZE ANGLE OF SENSOR TO START AT 120 DEG
fighdl=figure('Name','Tanget bug plot'); 
filename = 'E:\Other\TAN_BUGz_.gif';
axis([-100,1600,-700,900]);
axis off
obshdl=TangetBugPlotObs(obs);
hold on
robothdl=scatter(xs,ys);
plot([ qs(1,1),qg(1,1)],[qs(1,2) ,qg(1,2)],'r');
min_h_dist=0;
ViewRange = linspace((120*pi/180),-(120*pi/180),100);           
xView = R*cos(ViewRange) + xs;                                  
yView = R*sin(ViewRange) + ys;                                  
rangehdl = plot(xView,yView); 
thetaMhdl = plot([x,x+200*cos(thetaM)],[y,y+200*sin(thetaM)],'r','LineWidth',2);
%rphdl = plot([xpos_rp,xpos_rp+200*cos(indexOfedge*pi/180)],[ypos_rp,ypos_rp+200*sin(indexOfedge*pi/180)]);
rphdl = [];
correctionThreshold = 45*pi/180;   % In rads
wallThreshold = 145; %145        % In mm
%moation to goal (mode 1)
mode = 1;
tic 
for jj=1:281474976710655 % while the current pos of the robot is not at the goal 
    masteri=masteri+1;
    for scan=1:241
        ii =ii+1;  
        theta_us(ii);
   
        q = [x,y,theta];
    
        [obsDet(ii),rho(ii)]=testObsVec(q, theta_us(ii), res, R, obs);
    end
    ii=0;     
    thetaM = atan2(yg-y,xg-x);
    dist2goal = sqrt((xg-x)^2 + (yg-y)^2);
    theta_us_goal=thetaM-theta;
    goal_Inside_FOV=false;
    
    if mode == 1
        
        min_h_dist_old=min_h_dist;
        if(abs(theta_us_goal)<(2*pi/3))
            [~,indexOf_theta_goal]=min(abs(theta_us-theta_us_goal));
            goal_Inside_FOV=true;
        end
        
        if (goal_Inside_FOV && rho(indexOf_theta_goal)<dist2goal) || (goal_Inside_FOV == false)

            [newRho,indexOfedge]= FindEdge(rho,R); %Find the edge of obs
            
            if (min(newRho)<R)&&(min(newRho)>0)
                
                [ min_h_dist,~,ThetaIMHD] = LowestHeristicDist(newRho,indexOfedge,xg,yg,x,y,theta,theta_us); % need to passs the angle govended by ii or masteri
                thetaM=theta+ThetaIMHD; 
                % find out if last hist dist is larger then newest                
                 
            end             
        end 
        
        [minRho,index_minRho] = min(rho);
        if (minRho < wallThreshold)
            
            if (theta_us(index_minRho) > pi/24)   % Obs on Left
                
                thetaM = thetaM - correctionThreshold;
                
            elseif (theta_us(index_minRho) < -pi/24)   % Obs on Right
                
                thetaM = thetaM + correctionThreshold;      
            end            
        end
                                                    
        
        
    end
    
    
    if ~isempty(rphdl)
        
        for i = 1:length(rphdl)            
            delete(rphdl(i));            
        end
        
    end
    
    [n,rphdl] = TangetBugPlot(xpos,ypos,robothdl,rangehdl,thetaMhdl,n,thetaM,theta,newRho,indexOfedge);
    newRho = [];
    indexOfedge = [];
    theta = theta + Kp*(thetaM - theta);
    x = x + ds*cos(theta);                    % mm
    y = y + ds*sin(theta);                    % mm
    
    state_vectorX(masteri)=x;
    state_vectorY(masteri)=y;
    
    
    n = n+1;
    xpos(n) = x;
    ypos(n) = y;
    % if you want to look more SLAM then uncomet line " [n,rphdl]" ...
    %[n,rphdl] = TangetBugPlot(xpos,ypos,robothdl,rangehdl,thetaMhdl,n,thetaM,theta,newRho,indexOfedge);
    drawnow
    axis equal
    frame = getframe(1);
    im = frame2im(frame);
      [imind,cm] = rgb2ind(im,256);
      if jj == 1;
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
      else
          imwrite(imind,cm,filename,'gif','WriteMode','append');
      end
    
     if (sqrt((x-xg)^2+(y-yg)^2)<1*ds)
        break
     end  
end 
toc 




