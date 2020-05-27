% Stephanie Frederick
% Daniel Cohen
% ME 406L  Lab 7

clear 
clc 
close all

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
xg = 1650;        % mm
yg = 1650;         % mm
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
R = 1000;                       % Range (mm)
         %(nx4) array [x1min,x1max,y1min,y1max;...] of n rectangular obstacles
wT = 50;
obs = [      0 2440       0   wT;
             0   wT       0 2440;
             0 2440 2440-wT 2440;
       2440-wT 2440       0 2440;
             0  720 1030-wT 1030;
        720-wT  720     460 1030;
        720-wT  720    1570 2440;
           460  720 1580-wT 1580;
       1240-wT 1240       0  450;
       1240-wT 1980  910-wT  910;
       1260-wT 1980 1420-wT 1420;
       1260-wT 1260    1420 1950;
       1980-wT 1980    1420 1950;
       1260-wT 1520 1950-wT 1950]; 

ds = 25;    % mm dist setep changed to 20~based on Dr. G recomdation 
Kp = 1;
obsDet = 0; % Assume no obstacle detected initially

% INITIALIZE ANGLE OF SENSOR TO START AT 120 DEG
fighdl=figure('Name','Tanget bug plot'); 
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
wallThreshold = 145;         % In mm
%moation to goal (mode 1)
mode = 1;
for jj=1:1000 % while the current pos of the robot is not at the goal 
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
                                                    
        %mode = 2
%         if min_h_dist> min_h_dist_old+res
%             mode=2;
%         
%         else
%             disp('mode1')
%         end 
        
    end
%     if mode == 2
%         %mode2
%         disp('mode2')
%         [ newRho ,indexOfedge ]= FindEdge(rho);
%         min_rho=min(rho);
%         [~,cols]=find(rho==min_rho);
%         min_cols=min(cols);
%         Theta_us_mode_2=theta_us(min_cols);
%         
%         
%         if(  Theta_us_mode_2<0)
%             thetaM=theta+Theta_us_mode_2+(pi/2);
%         else 
%             thetaM=theta-Theta_us_mode_2+(pi/2);
%         end 
%         
%         % Update dmin 
%         robo2goalx=xpos-xg;
%         robo2goaly=ypos-yg;
%         dist_robo2goal=sqrt(robo2goalx^2+robo2goaly^2);
%         dmin(masteri)=dist_robo2goal-min_cols;
%         [~,dmin_cols]=find(dmin>0);
%         sorted_dmin=sort(dmin(dmin_cols));
%         dmin_abs=sorted_dmin(1);
%         % Update dleave
%         
%         
%         theta_us_goal=thetaM-theta;
%         if (rho(121)<400)
%             
%             dleave=dist_robo2goal;
%             
%         else 
%             [~,indexOf_theta_goal]=min(abs(theta_us-theta_us_goal));
%             
%             
%             dleave=dist_robo2goal-rho(indexOf_theta_goal);
%             
%         end 
%         
%         if (dleave<dmin+(3*res))
%             mode=1;
%         end 
%         
%         
%     end
    
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
    
    %[n,rphdl] = TangetBugPlot(xpos,ypos,robothdl,rangehdl,thetaMhdl,n,thetaM,theta,newRho,indexOfedge);
    drawnow
    axis equal
    
    if (sqrt((x-xg)^2+(y-yg)^2)<1*ds)
        break
    end 
end 

