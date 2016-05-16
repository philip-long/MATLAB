function q=diffIGM(Td,j,qr)

% This function calculates the IGM from a end effector position using
% both the GENJAC function and GEN DGM function to iteratively find a
% solution for a general robot structure, using Khalil alogrithm 6.6

% Td is the deisred robot frame, if the robot is redundant
% j is the value of the redundant joint
% qr is the limit of the redundant joint


% % For the stanford robot
% %  Td=[  -0.7801    0.5227    0.3438   -0.1667
% %     0.6246    0.6831    0.3786   -0.1280
% %    -0.0370    0.5101   -0.8593   -0.0182
% %          0         0         0    1.0000];
% % 
% % For the Staubli Robot
% %  Td =   [-0.9378   -0.1263    0.3234    0.0703
% %     0.1485    0.6960    0.7025    0.1070
% %    -0.3138    0.7068   -0.6340   -0.0841
% %          0         0         0    1.0000];


global qlower qupper

global sigma Joints


ql=qlower;qu=qupper;
n=Joints;
des=n; %Choose at which frame NOT SUPPORTED!
RedFix=0;
if exist('j','var') %Redundant with fixed value
    
    if n>6 %Redundant Robot
        disp 'Redundant Robot so fixing joint at given value'
        
        if qr<qu(j) && qr>ql(j) %Check if value is in limit
            
            qu(j)=[];ql(j)=[]; %Deleteing associated limit
            
            RedFix=1; %Flag
            n=n-1;    %Fixing redundant joint so reducing number of joints
            
        else
            disp 'Given value is outside joint limit'
            pause()
        end
        
    end
    
end





%% BEGIN ITERATIVE SOLUTION:






iterBig=0;

while iterBig < 500 % number of reinitialisations
    
    % 1. Initialise q by any random values within joint limits
    qc=rand(n,1).*(qu'-ql')+ql';
    
    
    iter=0;
    
    % Searching in the neighbourhood of qc
    while(1)
        
        
        
        % 1.5 Insert Joint extra Joint values
        if RedFix==1
            qcr=[qc(1:(j-1));qr;qc(j:end)];
            T=GENDGM(qcr);
        else
            % 2.  Find Current End Effector location
            T=GENDGM(qc);
            
        end
        
        
        
        
        % 3. Find Error in Position & Orietation
        % Poisition error is equivalent to linear velocity
        % Error in rotation matrix is like and angular velocity
        
        E=CartesianError([T{des};Td]);
        
        % 4. Check size of Error
        norm(E(1:3))
        if norm(E(1:3))<0.0001 && norm(E(4:6))<0.04 
            ConfigFound=1; %Error is small exit loop
            break
        elseif iter>100
            ConfigFound=0; % Max iterations reached exit loop
            break
        else
            
            % 5. Calculate Jacobian matrix at desired frame
          
            
            if RedFix==1
                J=GENJAC(qcr,des);
                J(:,j)=[];
            else
                J=GENJAC(qc,des);

            end
            
            % 6. Calculate dq from Jacobian matrix
            dq= J\E;
            
            % 7. Update the current joint parameters
            qc=qc+dq;
            
            %qcd=qc*180/pi % Working in degrees
            
            % This for loop converts large negative rotations to
            % small positive ones,and vice versa
            
            for i=1:n
                if sigma(i)==0 %revolute joint
                    % step 1 mod it by 360
                    qc(i)=mod(qc(i),2*pi);
                    %qcd(i)=mod(qcd(i),360);
                    
                    if qc(i) > pi
                        qc(i)=qc(i)-(2*pi);
                    end
                    
                    %         if qcd(i) > 180
                    %             qcd(i)=qcd(i)-360;
                    %         end
                    
                end
            end
            
            iter=iter+1;
        end
    end
    
    % Check if solution has been found:
    %       if so check joint limits
    %                % reject solution outside joint limits and reintilise qc
    %       if not reintialise max interations has been reached, reinitilise
    
    if ConfigFound==1
        
        if all([qc<qu';qc>ql'])
            disp 'Solution Found'
            pause()
            if RedFix==1
                q=qcr;
            else
                q=qc;
            end
            
            break
        
        else
            disp 'qc outside joint limits-restart'
            qc
            pause()
            q=0;
        end
    else
        q=0;
        %disp 'No Config Found'
    end
    iterBig=iterBig+1;
    
    
end


%
T0L =[

    1.0000         0         0         0
         0    1.0000         0    0.4000
         0         0    1.0000         0
         0         0         0    1.0000];

disp 'T in world frame'
T0L*T70(q)
 E=CartesianError([T{des};T70(q)])
disp 'T in local frame'
T70(q)








