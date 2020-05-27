
% Stephanie Frederick
% Daniel Cohen
% ME 406L  Lab 6

clear all
 clc
 clear 



% Initialize brick
ev3Mode = 'wifi';
ev3IP = '172.19.24.242';
ev3ID = '0016534263b6';
Robot = legoev3('wifi','172.19.24.242', '0016534263b6');
%playTone(Robot,440,2);

% Initialize Motors
MotorL = motor(Robot, 'C');
MotorR = motor(Robot, 'B');
MotorUS = motor(Robot, 'D');
% Reset Sensors
resetRotation(MotorL); 
resetRotation(MotorR);
USsensor = sonicSensor(Robot,3);

R=27.0;
L=97.0;
res = 5;
x = 250;
y = 250;

resetRotation(MotorL);
resetRotation(MotorR);
resetRotation(MotorUS);

s=250;
s_old=0;
direction = 1; % direction ( 1 VS -1)
oldRot = 0;
Rot=0;
vcomm = 10;    % Speed 16
thetaRpre = 0;
thetaLpre = 0;
thetaLact = 0;
thetaRact = 0;
oldTime=0;
newTime=0; 
thetaLact = double(readRotation(MotorL)*pi/180);
thetaRact = double(readRotation(MotorR)*pi/180);
Theta=R*((thetaRact)-(thetaLact))/L;
ii=0;
dt=0;
ThetaGyro=0;
tic
toc
oldTime=toc;
start(MotorL); 
start(MotorR);
Sstart=0;
MotLspeed=vcomm;
MotRspeed=vcomm;
save=0;

while (toc < 60)
    ii=ii+1;
    TIME(ii)=toc;
  
        thetaLpre = thetaLact;
        thetaRpre = thetaRact;
        thetaLact = double(readRotation(MotorL)*pi/180);
        thetaRact = double(readRotation(MotorR)*pi/180);

        Theta = R*((thetaRact)-(thetaLact))/L;
        d_thetaR = thetaRact - thetaRpre;
        d_thetaL = thetaLact - thetaLpre;
        x = x + R *((d_thetaR + d_thetaL)/2) * cos(Theta);
        y = y + R *((d_thetaR + d_thetaL)/2) * sin(Theta);
        s=s+ R *((d_thetaR + d_thetaL)/2)
        
            start(MotorL);
            start(MotorR);
            MotorL.Speed = vcomm;
            MotorR.Speed = vcomm;
            
           % save=1;
            while abs(Sstart-s)<100 %mm
               while save==1
                Sstart=s;
                save=0;
               end 
                s_old=s;

                thetaLpre = thetaLact;
                thetaRpre = thetaRact;
                thetaLact = double(readRotation(MotorL)*pi/180);
                thetaRact = double(readRotation(MotorR)*pi/180);     
                Theta = R*((thetaRact)-(thetaLact))/L;
                d_thetaR = thetaRact - thetaRpre;
                d_thetaL = thetaLact - thetaLpre;
                x = x + R *((d_thetaR + d_thetaL)/2) * cos(Theta);
                y = y + R *((d_thetaR + d_thetaL)/2) * sin(Theta);
                s=s+ R *((d_thetaR + d_thetaL)/2)
       
            end 
        
         break 
       % [Wcomm]  = Wcommdes( state_vectorX,state_vectorY,x,y);   

       % if ii+1==length(state_vectorX) 
         %   break
 
 %end 
        
%     Gyro = gyroSensor(Robot,1);
%     resetRotationAngle(Gyro); 
%     theta_des=atan2(state_vectorY(ii+1)-y,state_vectorX(ii+1)-x);
%     ThetaROT = double(readRotationAngle(Gyro)*pi/180);
%     
%        
%     while theta_des>ThetaROT
%             ThetaROT = double(readRotationAngle(Gyro)*pi/180);
%             WGyro = double(readRotationRate(Gyro)*pi/180.0);
%             WL=(d_thetaL/dt);
%             WR=(d_thetaR/dt);
%             vel=(R*(WR+WL)/2);
%             ThetaGyro = ThetaGyro + WGyro*dt;
%             W=(WR+WL)/2;
%             MotorL.Speed = vcomm-Wcomm;
%             MotorR.Speed = vcomm+Wcomm;
%             thetaLpre = thetaLact;
%             thetaRpre = thetaRact;
%             thetaLact = double(readRotation(MotorL)*pi/180);
%             thetaRact = double(readRotation(MotorR)*pi/180);
%             Theta = R*((thetaRact)-(thetaLact))/L;
%             d_thetaR = thetaRact - thetaRpre;
%             d_thetaL = thetaLact - thetaLpre;
%             x = x + R *((d_thetaR + d_thetaL)/2) * cos(Theta);
%             y = y + R *((d_thetaR + d_thetaL)/2) * sin(Theta);
%             s=s+ R *((d_thetaR + d_thetaL)/2);
% 
%             newTime=toc;
%             dt=newTime-oldTime;
%             oldTime=newTime;
%     end 
        
        hold on
        plot(x,y,'b*')
        axis equal
        grid on
        drawnow
        
   end 
     
stop(MotorL);
stop(MotorR);


% if abs(s-s_old)>20
%     s_old=s;
%     stop(MotorL);
%     stop(MotorR);
%     MotorUS.Speed = 30*direction;
%     start(MotorUS);
%        
%     for i = 1:36 
%         
%         while (abs(Rot-oldRot) < 2.09) 
%             
%             Rot = -direction*double(readRotation(MotorUS)) * pi/180 
%         end
% 
%         MotorUS.Speed = 0;
%         oldRot = Rot;
% 
%         for j = 1:5
%             temp(j) = (readDistance(USsensor)) * 1000;  % Type double and in mm
%         end 
% 
%         rho(i) = mean(temp);
%         theta_US(i) = Rot/24+pi/2*direction;
%         xrp(i) = x + rho(i)*cos(theta + theta_US(i));
%         yrp(i) = y + rho(i)*sin(theta + theta_US(i));
%         XRP(ii)=xrp(i) ;
%         YRP(ii)=yrp(i) ;
%             
%         MotorUS.Speed = 30*direction;
%         
%     end   
% 
      % Plot
%         hold on
%         plot(x,y,'b*')
%         plot(xrp,yrp,'bx')
%         xlabel('X (mm)')
%         ylabel('Y (mm)')
%         axis equal
%         grid on
%         drawnow
%         plot([x,x+500*cos(theta+thetaM)],[y,y+500*sin(theta+thetaM)])
%         direction=-1*direction;
%         start(MotorL);
%         start(MotorR);
%         stop(MotorUS);

   
   
   
   
%    newTime=toc;
%    dt=newTime-oldTime;
%    oldTime=newTime;
%    
% if (abs(x-xg)<10)
%         break
end
%    







