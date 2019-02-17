function [Out]= Traj(t)



%First define a trajectory giving
   %1. R, Omega
   %2  
   
   
   % while(1)
%
%  t=t+0.05


%  Trajectory Parameters
tfinal=3.0;         % Total time per section including rest
       % Total Rest time between points
time=t; % t= 0 --> tsection then resets

if t<tfinal
    Rinit=TransMat(-pi/4,'z','rot')*TransMat(-pi/3,'x','rot')*TransMat(-pi/7,'z','rot');
    Rfinal=TransMat(-pi/2,'y','rot')*TransMat(-pi/4,'x','rot')*TransMat(-pi/3,'z','rot');
elseif t<2*tfinal
    Rinit=TransMat(-pi/2,'y','rot')*TransMat(-pi/4,'x','rot')*TransMat(-pi/3,'z','rot');
    Rfinal=TransMat(-pi/13,'z','rot');
    time=time-tfinal;
else
    Rinit=TransMat(-pi/13,'z','rot');
    Rfinal=TransMat(rand(1),'z','rot')*TransMat(pi/6,'x','rot')*TransMat(-pi/7,'z','rot');
    time=time-(2*tfinal);
end


%Interpolation function between points

[rt rtdot rtddot]=TrajGen_5Poly(time,tfinal);

%Get Current Variables from trajectory definition and
[R,Omega,Omegadot]=TrajGen_Vari(Rinit,Rfinal,rt,rtdot,rtddot);

disp '================================================'

R % Output, Euler/Quaternion
reshape(R,1,9)
Omega
Omegadot
Out=[reshape(R,1,9), Omega',Omegadot'];
time
disp '================================================'


% Interpolation function between points
    function [rt rtdot rtddot]=TrajGen_5Poly(time,tsection)
        
        % Interpolation function between points on robot path
        %  Inputs: time=current time (or time spent on this this trajectory
        %          tsection=time per section
        %  Outputs: Interpolation parameters representing 5 degree polynomial
        
        
      
        %Interpolation Function Including rest time options
        
        rt=(10*((time/(tsection))^3))-(15*((time/(tsection))^4))+(6*((time/(tsection))^5))
        rtdot =(30*time^2)/(tsection)^3 - (60*time^3)/(tsection)^4 + (30*time^4)/(tsection)^5
        rtddot=((120*time^3)/(tsection)^5 - (180*time^2)/(tsection)^4 + (60*time)/(tsection)^3)
    end

%Get Current Variables from trajectory definition
    function [R,Omega,Omegadot]=TrajGen_Vari(Rinit,Rfinal,rt, rtdot, rtddot)
        % Function TrajGen_Vari to return current parameters of trajectory
        %
        %   Inputs: Initial, final frame parameters, interpolation parameters
        %   Outputs: Current Pose, Velocity, Acceleration
        %
        
        
        
       
        RotuAlpha=Rfinal*transpose(Rinit);
        
        Calpha=0.5*(RotuAlpha(1,1)+RotuAlpha(2,2)+RotuAlpha(3,3)-1);
        Salpha=0.5*(((RotuAlpha(2,3)-RotuAlpha(3,2))^2+(RotuAlpha(3,1)-RotuAlpha(1,3))^2+(RotuAlpha(1,2)-RotuAlpha(2,1))^2)^0.5);
        alpha=atan2(Salpha,Calpha);
        
        if alpha==0 %No change
            
            RotuAlpha_rt=eye(3);
            u=zeros(3,1);
            
        else
            
            u= [(RotuAlpha(3,2)-RotuAlpha(2,3))/(2*Salpha)
                (RotuAlpha(1,3)-RotuAlpha(3,1))/(2*Salpha)
                (RotuAlpha(2,1)-RotuAlpha(1,2))/(2*Salpha)];
            uskew=skew(u);
            nu=rt*alpha; %evolution of alpha in time
            RotuAlpha_rt=(u*(u').*(1-cos(nu))) + (eye(3)*cos(nu))+ (uskew*sin(nu));
            
        end
        
       
        R=RotuAlpha_rt*Rinit;
        Omega=(u*rtdot*alpha);
        Omegadot=(u*rtddot*alpha);
    end

end