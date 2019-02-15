%% main_wk_inverse
%  In this script we calculate the polytopes and the end point of robot 
% This is a workspace analysis
%  we cycle through a discretized version of the workspace, at each cell
%  we obtain the IKM. Both the Elbow up and Elbow down configurations are 
%  saved. We save the following
%         ELBOW_DOWN=[ELBOW_DOWN; ...  #configuration
%                     Xdesired,Ydesired,... #position 
%                     q_pos,...  # joint position
%                     Q_arm.volume,... # Polytope information
%                     Q_arm_obs.volume,...
%                     Q_arm_jl.volume,...
%                     Q_arm_obs_jl.volume,...
%                     V_arm.volume,...
%                     V_arm_obs.volume,...
%                     V_arm_jl.volume,...
%                     V_arm_obs_jl.volume];
%
%
%  The results are saved in WK then are plotted in the DATA folder. 

close all
clear all
clc
addpath(genpath('~tbxmanager'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))


ELBOW_DOWN=[];
ELBOW_UP=[];

joint_names={'q11', 'q12'};
total_frames=length(joint_names);
qp_upperlimits=ones(total_frames,1)*1.0;
qp_lowerlimits=qp_upperlimits*-1;
q_max=[pi/2,3*pi/2];
q_min=-q_max;
for Xdesired=-1.35:0.018:(1.35)
    for Ydesired=-1.35:0.018:(1.35)
        % Returns Q_configs where elbow down config is first row.
        Q_configs=getIGM(Xdesired,Ydesired);
        % Check joint limits
        if(length(Q_configs)>1)
            config=checkJointLimits(Q_configs,q_max,q_min);
        else
            disp 'No IGM solution'
            ELBOW_DOWN=[ELBOW_DOWN; ...
                Xdesired,Ydesired,...
                nan(1,11)];
            ELBOW_UP=[ELBOW_UP; ...
                Xdesired,Ydesired,...
                nan(1,11)];
            continue % No IGM solution
        end
        if(isempty(config))
            disp 'No valid IGM solution'
            ELBOW_DOWN=[ELBOW_DOWN; ...
                Xdesired,Ydesired,...
                nan(1,11)];
            ELBOW_UP=[ELBOW_UP; ...
                Xdesired,Ydesired,...
                nan(1,11)];
            continue % No valid IGM function
        end
        %%
        if(length(config)<2)
            disp 'Only 1 valid solution'
            % Push back other as NaN
            if(config==1)
                ELBOW_UP=[ELBOW_UP; ...
                    Xdesired,Ydesired,...
                    nan(1,11)];
            elseif(config==2) % Elbow up q(2) -ve
                ELBOW_DOWN=[ELBOW_DOWN; ...
                    Xdesired,...
                    Ydesired,...
                    nan(1,11)];
            end
        end
        for c=1:length(config) % cycle through all valid configs
            q_pos=Q_configs(config(c),:);
            [phi_max,phi_min]=getJointLimitPenalty(q_pos,q_max,q_min,4);
            PHI=eye(total_frames*2)-diag([phi_max,phi_min]);
            T=getTransforms(q_pos); % Get all transforms
            J=getJacobians(q_pos);  % Get all Jacobians
            % set saftey threshold
            TnE=eye(4); TnE(1,4)=0.55;   % terminal point
            % Define Control point and publish frame
            % This is necessary  since my chain ends with the last joint
            [T0E,J0E]=getEEInfo(T{total_frames},J{total_frames},TnE,'world',1);
            T{end+1}=T0E;
            J{end+1}=J0E;
            
            %%  Object initialization
            ObjectPositions=[];
            ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
            ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
            ObjectPositions(3,:)=[0.8;-0.45;0.0]; % in world frame adding a third object
            
            numberSteps=5;
            % Field steps
            A=[eye(2) ;-eye(2)];
            B=[qp_upperlimits ;qp_upperlimits];
           
            dfdesired=12.0;
            for o=1: size(ObjectPositions,1)
                for i=1:(size(T,2)-1)
                    iTip1=inv(T{i})*T{i+1};
                    iPip1=iTip1(1:3,4);
                    if(norm(iPip1)<0.001)% Links are coicident
                        disp ' Links coincident'
                    else  % Links are not
                        
                        for j=1:numberSteps
                            distance_along_link=j*(T{i}(1:3,1:3)*iPip1)/numberSteps;
                            cp=T{i}(1:3,4)+distance_along_link;
                            Lhat=skew(distance_along_link);
                            screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                            JJ=screwTransform*J{i};
                            rdiff=ObjectPositions(o,:)'-cp;
                            n=(rdiff)/norm(rdiff);
                            JJ=[JJ, zeros(6,length(qp_upperlimits)-size(JJ,2))];
                            A=[A;n'*JJ(1:3,:)];
                            if(norm(rdiff)<0.001)
                                rdiff=0.001;
                            end
                            B=[B;(dfdesired*norm(rdiff)^2)-norm(rdiff)];
                        end
                    end
                end
            end
            
            % Initial undeformed polytope
            Q_arm=Polyhedron(A(1:4,:),B(1:4));Q_arm.computeVRep;
            V_arm= getCartesianPolytope(J{end}(1:2,:),Q_arm);
            % Arm Joint Polytope after object Deformation
            Q_arm_obs=Polyhedron(A,B); Q_arm_obs.computeVRep;
            V_arm_obs= getCartesianPolytope(J{end}(1:2,:),Q_arm_obs);
            % Arm Joint Polytope after joint limits
            Q_arm_jl=Polyhedron(A(1:4,:),PHI*B(1:4)); Q_arm_jl.computeVRep;
            V_arm_jl= getCartesianPolytope(J{end}(1:2,:),Q_arm_jl);
            % Arm Joint Polytope after joint limits and obstacles
            Q_arm_obs_jl=Polyhedron(A,[PHI*B(1:4);B(5:end)]); Q_arm_obs_jl.computeVRep;
            V_arm_obs_jl= getCartesianPolytope(J{end}(1:2,:),Q_arm_obs_jl);
            c_extended=getExtendedManipulability(q_pos,ObjectPositions,q_max,q_min);
            
            
            if(config(c)==1) % Elbow down q(2) +ve
                
                ELBOW_DOWN=[ELBOW_DOWN; ...
                    Xdesired,Ydesired,...
                    q_pos,...
                    Q_arm.volume,...
                    Q_arm_obs.volume,...
                    Q_arm_jl.volume,...
                    Q_arm_obs_jl.volume,...
                    V_arm.volume,...
                    V_arm_obs.volume,...
                    V_arm_jl.volume,...
                    V_arm_obs_jl.volume,...
                    c_extended];
            elseif(config(c)==2) % Elbow up q(2) -ve
                ELBOW_UP=[ELBOW_UP; ...
                    Xdesired,Ydesired,...
                    q_pos,...
                    Q_arm.volume,...
                    Q_arm_obs.volume,...
                    Q_arm_jl.volume,...
                    Q_arm_obs_jl.volume,...
                    V_arm.volume,...
                    V_arm_obs.volume,...
                    V_arm_jl.volume,...
                    V_arm_obs_jl.volume,...
                    c_extended];
            else
                disp 'Some error'
            end
            
        end
    end
    
end
save('WK','ELBOW_UP','ELBOW_DOWN');
