function [ min_h_dist,indexOf_hist,ThetaIMHD ] = LowestHeristicDist(newRho,indexOfedge,xg,yg,x,y,theta,theta_us)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

ii=1;
h=0;
xrp=0;
yrp=0;
min_h_dist=0;
indexOf_hist=0;

for kk=(ii:length(newRho))
    
    xrp(kk)=x+newRho(kk)*cos(theta+theta_us(indexOfedge(kk)));% with old x 
    yrp(kk)=y+newRho(kk)*sin(theta+theta_us(indexOfedge(kk)));% with old y
    h(kk)=newRho(kk)+sqrt((xg-xrp(kk))^2+(yg-yrp(kk))^2);% hiristics dist
    
end 


min_h_dist=min(h);
indexOf_hist=find(h==min_h_dist);

ThetaIMHD=(theta_us(indexOfedge(indexOf_hist)));

if abs(ThetaIMHD)>pi/6
    ThetaIMHD=(pi/6)*sign(ThetaIMHD);
end
end

