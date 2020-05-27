% dan cohen

clf 
clc

thetaUS_1=-120;
thetaUS_2=120;
n=0;
for (ii=1:length(atm)) 
    thetaUS_1=-120;
    
    thetaUS_2=120;
    
    for(jj=1:36)
       
            
            xc1=atm(ii).q(1) ;
            
            yc1=atm(ii).q(2) ;
            
            r1=atm(ii).rho(jj);
            
            thetaUS_1=1+thetaUS_1;
            
            theta_min1=((atm(ii).q(3))*180/pi)+thetaUS_1-9;
            
            theta_max1=((atm(ii).q(3))*180/pi)+thetaUS_1+9;
            
            c1=[xc1,yc1,r1,theta_min1,theta_max1]';
            
            
            xc2=atm(ii).q(1) ;
            
            yc2=atm(ii).q(2) ;
            
            
            r2= atm(ii).rho(jj);
            
            thetaUS_2=thetaUS_2+1;
            
            theta_min2=((atm(ii).q(3))*180/pi)+thetaUS_2-9;
            
            theta_max2=((atm(ii).q(3))*180/pi)+thetaUS_2+9;
            
            c2=[xc2,yc2,r2,theta_min2,theta_max2]';
            
            
            [aInt, xi, yi] = arcInt(c1, c2);
            if(aInt==true)
                plot(xi,yi,'x')
                hold on
               n=n+1;
                disp(n)
                % plot
                
            end
            hold on 
            plot(xc1,yc1,'r*')
            
            
        end
    end 


axis equal 



