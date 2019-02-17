function q=diffIGM_GS(Td,Leg)

% This function calculates the IGM from a end effector position using
% both the GENJAC function and GEN DGM function to iteratively find a
% solution for a general robot structure, using Khalil alogrithm 6.6
% This function is specific for G-S platform
% Leg chooses which leg to model
% Td is only valid for position

global sigma Joints

qu=ones(1,3)*pi;
ql=ones(1,3)*-pi;
n=Joints;
des=3;

%% BEGIN ITERATIVE SOLUTION:






iterBig=0;

while iterBig < 125 % number of reinitialisations
    
    % 1. Initialise q by any random values within joint limits
    qc=(rand(n,1)*2*pi)-pi;
    iter=0;
    
    % Searching in the neighbourhood of qc
    while(1)
        

            % 2.  Find Current End Effector location
    
            T=DGM_leg(qc,Leg,3);
            
  
        % 3. Find Error in Position & Orietation
        % Poisition error is equivalent to linear velocity
        % Error in rotation matrix is like and angular velocity
        
        E=CartesianError([T;Td]);
        
        % 4. Check size of Error
        
        if norm(E(1:3))<0.0001
            ConfigFound=1; %Error is small exit loop
            break
        elseif iter>100
            ConfigFound=0; % Max iterations reached exit loop
            break
        else
            
            % 5. Calculate Jacobian matrix at desired frame
          
            J=GENJAC_tree(qc,des);
            
            % 6. Calculate dq from Jacobian matrix
            dq= J(1:3,1:3)\E(1:3);

            % 7. Update the current joint parameters
            qc=qc+dq;
           % qc(3)= 1.4632;
            %pause()
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
        
            disp 'Solution Found'
           q=qc;
            break
        
    else
        q=0;
    end
    iterBig=iterBig+1;
    
    
end













