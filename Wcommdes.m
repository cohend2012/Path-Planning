function [  v2,ThetaM ] = Wcommdes( state_vectorX,state_vectorY,x,y,dt)
%Wcommdes Finds the desired Wcomm for part 2 
%   the followinc calc are to find Wcomm at any state_vectors 
tic 
toc
Kptheta=.1;
KI=.15;
chapinter=0;
oldTime=toc;

Z=0;
% no z dir just x and you 

if length(state_vectorX)~=length(state_vectorY)
    disp(' your state vector are not the same size')
    
end 


vp=[x,y,Z]';
mindis=zeros(1,length(state_vectorX));

for kk=1:length(state_vectorX)
    v=[state_vectorX(kk),state_vectorY(kk),Z]';
    mindis(kk)=norm(vp-v);
  
end

[val,i_min]=min(mindis);
   
v1=[state_vectorX(i_min),state_vectorY(i_min),Z]'; % current step on traj

v2=[state_vectorX(i_min+10),state_vectorY(i_min+10),Z]'; % nexr step on tralj

ThetaM=atan2(state_vectorY(i_min+5)-y,state_vectorX(i_min+5)-x);

% if i_min==length(state_vectorX) 
%    Wcomm=BackUpWcomm;
%     
% end 
a=v2-v1;
    
b=vp-v1;    

  
MagA=sqrt(a(1)^2+a(2)^2+a(3)^2);

MagB=sqrt(b(1)^2+b(2)^2+b(3)^2);

newMagB=MagB;
alpha=asin(cross(a,b)/((MagA*MagB)));

% if sign(alpha(3))>.5
%     chapinter=-1;
%     
% if sign(alpha(3))<.5
%     chapinter=1;
% end 


% Wcomm=-Kptheta*MagB*sign(alpha(3));
% disp(alpha)
% BackUpWcomm=Wcomm;
% 
% newTime=toc;
% 
% dt=newTime-oldTime;
% oldTime=newTime;



% tic 
% toc
% Kptheta=1;
% KI=.15;
% chapinter=0;
% oldTime=toc;
% 
% Z=0;
% % no z dir just x and you 
% 
% if length(state_vectorX)~=length(state_vectorY)
%     disp(' your state vector are not the same size')
%     
% end 
% 
% 
% vp=[x,y,Z]';
% mindis=zeros(1,length(state_vectorX));
% 
% for kk=1:length(state_vectorX)
%     v=[state_vectorX(kk),state_vectorY(kk),Z]';
%     mindis(kk)=abs(sqrt((vp(1)-v(1))^2+(vp(2)-v(2))^2));
%   
% end
% 
% [val,i_min]=min(mindis);
% 
% 
% 
% % for  kk=1:length(state_vectorX)
% %     v_2=[state_vectorX(kk),state_vectorY(kk),Z]';
% %     mindis_2(kk)=norm(vp+15-v_2);
% %     
% %     
% % end 
% % 
% % [val,i_min_2]=min(mindis_2);
% 
% 
% 
% v1=[state_vectorX(i_min),state_vectorY(i_min),Z]'; % current step on traj
% 
% v2=[state_vectorX(kk+1),state_vectorY(kk+2),Z]'; % nexr step on tralj
% 
% angle=atan2(vp(2)-v2(2),vp(1)-v2(1));
% 
% % if i_min==length(state_vectorX) 
% %    Wcomm=BackUpWcomm;
% %     
% % end 
% a=v2-v1;
%     
% b=vp-v1;    
% 
%   
% MagA=sqrt(a(1)^2+a(2)^2+a(3)^2);
% 
% MagB=sqrt(b(1)^2+b(2)^2+b(3)^2);
% 
% newMagB=MagB;
% 
% alpha=asin(cross(a,b)/((MagA*MagB)));
% 
% % if sign(alpha(3))>.5
% %     chapinter=-1;
% %     
% % if sign(alpha(3))<.5
% %     chapinter=1;
% % end 
% 
% 
% Wcomm=-Kptheta*MagB*sign(alpha(3));%-KI*MagB*dt*sign(alpha(3));%+chapinter;
% disp(alpha)
% BackUpWcomm=Wcomm;
% 
% newTime=toc;
% 
% dt=newTime-oldTime;
% oldTime=newTime;


end

