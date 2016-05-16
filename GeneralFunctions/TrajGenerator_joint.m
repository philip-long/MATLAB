function [Out]= TrajGenerator_joint(t)

% This function outputs a joint trajectory for a initial and final position
% Trajectory is given in joint position, velocity accleration
%
%

%Global Parameters

%DESCRIPTION: This function outputs a trajectory in position , velocity
%,acceleartion for a simulink program.


global qinit %Initial Joint configration of the arm






%  Trajectory Parameters
tsection=1.0;         % Total time per section including rest
RestTime=0.05;         % Total Rest time between points
time=mod(t,tsection); % t= 0 --> tsection then resets

% ====== Trajectory type Defining Intial and final points in sequence=====

[qi,qf]=Traj1_Plane(t,tsection,qinit);
%=======================================================================






%================== Interpolation function between initial and final points==================

[rt rtdot rtddot]=TrajGen_5Poly(time,tsection,RestTime);

%Get Current Variables from trajectory definition and
[q,qdot,qddot]=TrajGen_Vari(qi,qf,rt, rtdot, rtddot);

q

Out=[q;qdot;qddot];



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
    function [q,qdot,qddot]=TrajGen_Vari(qinit,qfinal,rt, rtdot, rtddot)
        % Function TrajGen_Vari to return current parameters of trajectory
        %
        %   Inputs: Initial, final joint parameters, interpolation parameters
        %   Outputs: Current joint position, Velocity, Acceleration
        %
        
        
        
        D=qfinal-qinit; % Takes the difference between the intial and final points of a section in position
        q=qinit+rt*D;
        qdot=rtdot*D;
        qddot=rtddot*D;
        
        
    end
% Trajectory Definition
%  Function is defined to go to a desired position then return to the home
%  position, as many times as we want, between each mini trajectory there
%  is a rest time
%
%



    function [qinit,qfinal]=Traj1_Plane(t,tsection,qinit)
        
        qhome=qinit; % Original Initial position
        Section=floor(t/tsection); % Section 'counts' number of resets
        
        % Now define desired offset from current position
        d1q=0.1;
        d2q=0.2;
        d3q=-0.1;
        
        
        switch Section % Switch defines the intial position currently and also the desired final position
            % in this section
            
            
            case 0   %
                qfinal=[qinit(1);qinit(2)+d2q;qinit(3)];
                
                
            case 1   % Y motion Return
                
                qinit=[qinit(1);qinit(2)+d2q;qinit(3)];
                qfinal=qhome;% Return to home position
                
                
            case 2   % X motion Forward
                
                qfinal=[qinit(1)+d1q;qinit(2);qinit(3)];
                
                
                
            case 3   % X motion Return
                
                qinit=[qinit(1)+d1q;qinit(2);qinit(3)];
                qfinal=qhome;% Return to home position
                
            case 4  % RZ motion Forward
                
                qinit=[qinit(1);qinit(2);qinit(3)+d3q];
                qfinal=qhome;% Return to home position
                
            case 5 % RZ motion Return
                
                qinit=[qinit(1);qinit(2);qinit(3)+d3q];
                qfinal=qhome;% Return to home position
                
                
            otherwise
                qinit=qhome;
                 qfinal=qhome;
                
        end
        
        
        
        
        
    end


end
