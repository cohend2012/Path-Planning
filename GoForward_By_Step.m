function [ thetaLpre,thetaLact,thetaRpre,thetaRact,R,L,Theta,d_thetaR,d_thetaL,x,y,s] = GoForward_By_Step( vcomm,thetaLpre,thetaRpre,thetaLact,thetaRact,R,L,Theta,d_thetaR,d_thetaL,x,y,s)

    
MotorL.Speed = vcomm;
MotorR.Speed = vcomm;
start(MotorL);
start(MotorR);
thetaRpre = 0;
thetaLpre = 0;
thetaLact = 0;
thetaRact = 0;

if abs(s-s_old)>=23 %mm
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
        s=s+ R *((d_thetaR + d_thetaL)/2);
        
        
        
end 






end

