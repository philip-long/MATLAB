function [Out]= TrajGenerator_Task(t)

% Function Generates a task trajectory for the Kuka
% lwr robot returns X and Xdot and Xddot in a row vector

%Global Parameters

%DESCRIPTION: This function outputs a trajectory in position , velocity
%,acceleartion for a simulink program. 


global qinit %Initial Joint configration of the arm






%  Trajectory Parameters
tsection=1.0;         % Total time per section including rest
RestTime=0.05;         % Total Rest time between points
time=mod(t,tsection); % t= 0 --> tsection then resets
T=T70(qinit);     % Get initial location of tool frame i.e  tranosformation from joint 7 to frame 0


% ====== Trajectory type Defining Intial and final points in sequence================

%[Pinit,Pfinal,Rinit,Rfinal]=Traj1_Plane(t,tsection,T); %Moving robot in
%plane

[Pinit,Pfinal,Rinit,Rfinal]=Traj2_6Dofdisplay(t,tsection,T); % Demonstrating 6 DOF motion

%[Pinit,Pfinal,Rinit,Rfinal]=Traj3_StiffnessTests(t,tsection,T); %
%Stiffness tests

%=======================================================================






%================== Interpolation function between initial and final points==================

[rt rtdot rtddot]=TrajGen_5Poly(time,tsection,RestTime);

%Get Current Variables from trajectory definition and
[P,R,ualpha,V,Omega,Vdot,Omegadot]=TrajGen_Vari(Pinit,Pfinal,Rinit,Rfinal,rt, rtdot, rtddot);

%disp '-------------------Tdesired-----------------'
%Tdes=[R P;0 0 0 1]
%disp '--------------------------------------------'
R=reshape(R,9,1);

% This is the output of the function, note in this case R is the DCM of 9
% elements, if you want a minimum representation you can easily convert to
% euler angles 




Out=[P;R;V(1:3);Omega(1:3);Vdot(1:3);Omegadot(1:3)];


%========================================================================

% Interpolation function between points
    function [rt rtdot rtddot]=TrajGen_5Poly(time,tsection,RestTime)
        
        % Interpolation function between points on robot path
        %  Inputs: time=current time (or time spent on this this trajectory
        %          tsection=time per section
        %  Outputs: Interpolation parameters representing 5 degree polynomial
        
        
        if time>(tsection-RestTime) % A rest time between each trajectory section is defined
            time=tsection-RestTime;
            disp '========= Resting ===========' 
        end
        
        %5 DOF Interpolation Function Including rest time options
        % 5 DOF polynomial implies that the curve is continous in
        % position,velocity and acceleration (twice differentialable), if you want you can use a
        % smaller degree and just make it contious in position velocity
        
        rt=(10*((time/(tsection-RestTime))^3))-(15*((time/(tsection-RestTime))^4))+(6*((time/(tsection-RestTime))^5));
        rtdot =(30*time^2)/(tsection-RestTime)^3 - (60*time^3)/(tsection-RestTime)^4 + (30*time^4)/(tsection-RestTime)^5;
        rtddot=((120*time^3)/(tsection-RestTime)^5 - (180*time^2)/(tsection-RestTime)^4 + (60*time)/(tsection-RestTime)^3);
    end

%Get Current Variables from trajectory definition
    function [P,R,ualpha,V,Omega,Vdot,Omegadot]=TrajGen_Vari(Pinit,Pfinal,Rinit,Rfinal,rt, rtdot, rtddot)
        % Function TrajGen_Vari to return current parameters of trajectory
        %
        %   Inputs: Initial, final frame parameters, interpolation parameters
        %   Outputs: Current Pose, Velocity, Acceleration
        %
        
        
        
        D=Pfinal-Pinit; % Takes the difference between the intial and final points of a section in position 
        RotuAlpha=Rfinal*transpose(Rinit); % Takes the difference between the intial and final points of a section in oreintation
        
        Calpha=0.5*(RotuAlpha(1,1)+RotuAlpha(2,2)+RotuAlpha(3,3)-1);
        Salpha=0.5*(((RotuAlpha(2,3)-RotuAlpha(3,2))^2+(RotuAlpha(3,1)-RotuAlpha(1,3))^2+(RotuAlpha(1,2)-RotuAlpha(2,1))^2)^0.5);
        alpha=atan2(Salpha,Calpha);
        
        if alpha==0 %No change
            
            RotuAlpha_rt=eye(3);
            u=zeros(3,1);
            
        else
            
            u= [(RotuAlpha(3,2)-RotuAlpha(2,3))/(2*Salpha)
                (RotuAlpha(1,3)-RotuAlpha(3,1))/(2*Salpha)
                (RotuAlpha(2,1)-RotuAlpha(1,2))/(2*Salpha)]; % Define the difference in orientation as a rotation around a axis u by angle alpha
            uskew=skew(u); % function getting the skew symmetric matrix of u to preform the cross product
            nu=rt*alpha; %evolution of alpha in time
            RotuAlpha_rt=(u*(u').*(1-cos(nu))) + (eye(3)*cos(nu))+ (uskew*sin(nu)); 
            
        end
        
       
        %----------------------------------------------------------------------
        %----------  Defining Current desired Parameters-----------------------
        %----------------------------------------------------------------------
        P=Pinit+rt*D;
        R=RotuAlpha_rt*Rinit;
        Allut=vrrotmat2vec(R)';
        ualpha=Allut(1:3)*Allut(4);
        V=rtdot*D;
        Omega=(u*rtdot*alpha);
        Vdot=rtddot*D;
        Omegadot=(u*rtddot*alpha);
    end
    

    
end
