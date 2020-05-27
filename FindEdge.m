function [newRho,indexOfedge] = FindEdge(rho,R)

%FindEdge find the edge of the obs
%   GIVE only obsDet 

len=length(rho);
newRho=0;
indexOfedge=0;
masteri=0;
Dthreshold = 200;
oldrho = rho;
rho(rho>=R) = 10*R;

for kk=(2:len-1)

    %if(((rho(kk-1)==R)&&rho(kk)~=R)||((rho(kk)~=R)&&rho(kk+1)==R))
    if (abs(rho(kk-1)-rho(kk)) > Dthreshold)
        
        [~,index] = min(rho(kk-1:kk));
        masteri=masteri+1;
        indexOfedge(masteri)=kk+index-2;
        newRho(masteri)=oldrho(kk+index-2); 
       
    end 
    
end

