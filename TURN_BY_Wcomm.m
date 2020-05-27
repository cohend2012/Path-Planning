function  [thetaLpre,thetaLact,thetaRpre,thetaRact,R,L,Theta,d_thetaR,d_thetaL,x,y,s] = TURN_BY_Wcomm(vcomm,Wcomm,state_vectorY(ii+1),y,state_vectorX(ii+1),thetaLpre,thetaLact,thetaRpre,thetaRact,R,L,Theta,d_thetaR,d_thetaL,x,y,s);
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Set up Gyro
    % Set up Gyro
    Gyro = gyroSensor(Robot,1);
    resetRotationAngle(Gyro);
    ThetaGyro=0;
    oldTime=0;
    newTime=0; 
    theta_des=atan2(state_vectorY(ii+1)-y,state_vectorX(ii+1)-x);
    ThetaROT = double(readRotationAngle(Gyro)*pi/180);
    tic
    toc
    
    while theta_des>=ThetaROT
        ThetaROT = double(readRotationAngle(Gyro)*pi/180);
        WGyro = double(readRotationRate(Gyro)*pi/180.0);
        WL=(d_thetaL/dt);
        WR=(d_thetaR/dt);
        vel=(R*(WR+WL)/2);
        ThetaGyro = ThetaGyro + WGyro*dt;
        W=(WR+WL)/2;
        MotorL.Speed = vcomm-Wcomm;
        MotorR.Speed = vcomm+Wcomm;
        
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

        newTime=toc;
        dt=newTime-oldTime;
        oldTime=newTime;
    end 
    

end

