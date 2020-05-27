function [ newRho ,indexOfedge ] = FindAllrhoundernumber(rho)
%FindEdge find the edge of the obs
%   GIVE only obsDet 
len=length(rho);
newRho=0;
jj=0;
indexOfedge=0;
masteri=0;

for kk=(1:len)

    if(rho(kk)<1000)
        masteri=masteri+1;
        indexOfedge(masteri)=kk;
        jj=jj+1;
        newRho(jj)=rho(kk);
              
    else
       
    end 
    
end