% Stephanie Frederick
% Daniel Cohen
% ME 406L  Lab 6 



% Set Up Brick
ev3Mode = 'wifi';
ev3IP = '172.19.24.242';
ev3ID = '0016534263b6';
Robot = legoev3(ev3Mode, ev3IP, ev3ID);
playTone(Robot,600,.10);
playTone(Robot,400,1);
playTone(Robot,600,.10);
pause(.1)
playTone(Robot,600,.10);
playTone(Robot,400,1);
playTone(Robot,600,.10);

% Initialize Motors
MotorL = motor(Robot, 'C');
MotorR = motor(Robot, 'B');

% Set up Gyro
Gyro = gyroSensor(Robot,1);

% ThetaUs to 
MotorUS = motor(Robot, 'D');
USsensor = sonicSensor(Robot,3);
% Reset Sensors
resetRotation(MotorL); 
resetRotation(MotorR); 
resetRotationAngle(Gyro);
resetRotation(MotorUS);

start(MotorL);
start(MotorR);

vcomm = 30 ;% Speed  6

load('a.mat');

R=27.0;
L=97.0;
x=250;
y=250;
ThetaGyro=0;
thetaRpre = 0;
thetaLpre = 0;
thetaLact = 0;
thetaRact = 0;
oldTime=0;
newTime=0; 
thetaLact = double(readRotation(MotorL)*pi/180);
thetaRact = double(readRotation(MotorR)*pi/180);
ThetaDot  = double(readRotationAngle(Gyro)*pi/180);
Theta=R*((thetaRact)-(thetaLact))/L;
ii=0;
dt=0;
tic
toc
Kp=6.75; %7
oldTime=toc;
s=0;
s_old=0;
rho=0;
theta_US=0;
Rot=0;
oldRot=0;
temp=0;
direction=1; % pos l->R neg R->L 
kk = 49;
while (toc < 1000)
  ii=ii+1;
   TIME(ii)=toc;
  
%    if toc>3
%        vcomm=10;
%    end 
   thetaLpre = thetaLact;
   thetaRpre = thetaRact;
   
   thetaLact = double(readRotation(MotorL)*pi/180);
   thetaRact = double(readRotation(MotorR)*pi/180);
   
   Theta = R*((thetaRact)-(thetaLact))/L;
   d_thetaR = thetaRact - thetaRpre;
   d_thetaL = thetaLact - thetaLpre;
   x = x + R *((d_thetaR + d_thetaL)/2) * cos(Theta);
   y = y + R *((d_thetaR + d_thetaL)/2) * sin(Theta);

   WGyro = double(readRotationRate(Gyro)*pi/180.0);
   WL=(d_thetaL/dt);
   WR=(d_thetaR/dt);
   vel=(R*(WR+WL)/2);
   ThetaGyro = ThetaGyro + WGyro*dt;
   W=(WR+WL)/2; 
   
   [  v2,ThetaM ] = Wcommdes( state_vectorX,state_vectorY,x,y,dt);   
   
    %s_old=s;
   
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
        
    etheta=ThetaM-Theta;
    
    Wcomm=Kp*etheta  ; 
   
   if (Wcomm > 50)
       Wcomm = 50;    
   end 
   
   if (Wcomm < -50)   
      Wcomm = -50;   
   end 
   
   MotorL.Speed = vcomm-Wcomm;   %-
   MotorR.Speed = vcomm+Wcomm;   %+
   
   if abs(s-s_old)>10000000000
       
       s=0;
        s_old=s;
   
        MotorL.Speed = 0 ;  %-
        MotorR.Speed = 0; 
        MotorUS.Speed = 30*direction;
        start(MotorUS);
       
    for i = 1:36 
        
        while (abs(Rot-oldRot) < 2)    
            Rot = -direction*double(readRotation(MotorUS)) * pi/180; 
        end

        MotorUS.Speed = 0;
        oldRot = Rot;

        for j = 1:5
            temp(j) = (readDistance(USsensor)) * 1000;  % Type double and in mm
        end 

        rho(i) = temp(j);%mean(temp);
        theta_US(i) = Rot/24+pi/2*direction;
        xrp(i) = x + rho(i)*cos(theta + theta_US(i));
        yrp(i) = y + rho(i)*sin(theta + theta_US(i));
             
        MotorUS.Speed = 30*direction;
        
        
    end  
    kk=kk+1;
    atm(kk).xrp=xrp;
    atm(kk).yrp=yrp;
    atm(kk).q(1)=x;
    atm(kk).q(2)=y;
    atm(kk).q(3)=Theta;
    atm(kk).rho=rho;
    atm(kk).theta_US=theta_US;
    MotorUS.Speed = 0;
    resetRotation(MotorUS);
    direction=-1*direction;
    
   end

   hold on 
   plot(x,y,'*',v2(1),v2(2),'k*')
   
   newTime=toc;
   dt=newTime-oldTime;
   oldTime=newTime;
   
%    if (sqrt((x-xg)^2+(y-yg)^2)<1*ds)
%         break
%     end 
   
end

stop(MotorL);
stop(MotorR);


