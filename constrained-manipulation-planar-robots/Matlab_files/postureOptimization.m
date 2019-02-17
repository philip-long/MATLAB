function [ wp_star ] = postureOptimization(q)
%postureOptimization objective function
total_frames=4;
total_mb=total_frames/2;
total_arm=total_frames/2;
q_mb=q(1:2);
q_pos=q(3:4);

qdot_mb_max=ones(total_mb,1)*0.5;
qdot_mb_min=qdot_mb_max*-1;
q_mb_max=[4.0;4.0];
q_mb_min=[-4.0;-4.0];
[phi_max,phi_min]=getJointLimitPenalty(q_mb,q_mb_max,q_mb_min,4);
PHI_mb=eye(total_mb*2)-diag([phi_max;phi_min]);
w_T_mb=getTransforms4DOF([q_mb,q_pos]); % Get all transforms
w_J_mb=getJacobians4DOF([q_mb,q_pos]);  % Get all Jacobians


qdot_arm_max=ones(total_arm,1)*1.0;
qdot_arm_min=qdot_arm_max*-1;
% position
q_arm_max=[pi/2;3*pi/4];
q_arm_min=[-pi/2;-3*pi/4];
[phi_max,phi_min]=getJointLimitPenalty(q_pos,q_arm_max,q_arm_min,4);
PHI=eye(total_arm*2)-diag([phi_max;phi_min]);
mb_T_arm=getTransforms(q_pos); % Get all transforms
mb_J_arm=getJacobians(q_pos);  % Get all Jacobians



n_T_ee=eye(4); n_T_ee(1,4)=0.55;   % terminal point
% Define Control point and publish frame
% This is necessary  since my chain ends with the last joint
[mb_T_arm_ee,mb_J_arm_ee]=getEEInfo(mb_T_arm{total_arm},mb_J_arm{total_arm},n_T_ee,'world',1);
mb_T_arm{end+1}=mb_T_arm_ee;
mb_J_arm{end+1}=mb_J_arm_ee;
w_T_arm=mb_T_arm;
w_J_arm=mb_J_arm;
for i=1:size(mb_T_arm,2)
    w_T_arm{i}=w_T_mb{2}*mb_T_arm{i};
    w_J_arm{i}=[w_T_mb{2}(1:3,1:3) zeros(3) ; zeros(3) w_T_mb{2}(1:3,1:3)]*mb_J_arm{i}; % For completeness doesn't do anything as orientation is equal
end

% Transforms mobile base to end effector frame
% again since there is no rotation, this is merely
% for completeness.
Lhat=skew(mb_T_arm{end}(1:3,4));
screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
w_J_mb{end+1}=screwTransform*w_J_mb{end};

ObjectPositions=[];
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
ObjectPositions(3,:)=[0.6;0.3;0.0]; % in world frame
ObjectPositions(4,:)=[0.4;0.8;0.0]; % in world frame


numberSteps=5;
% Field steps
A=[eye(2) ;-eye(2)];
Amb=[eye(2) ;-eye(2)];
B=[qdot_arm_max ;-qdot_arm_min];
Bmb=[qdot_mb_max;-qdot_mb_min];
Amec=[eye(4) ;-eye(4)];
Bmec=[qdot_mb_max;qdot_arm_max;-qdot_mb_min;-qdot_arm_min];


Q_arm_init=Polyhedron(A,B);Q_arm_init.computeVRep;
Q_mb_init=Polyhedron(Amb,Bmb);Q_mb_init.computeVRep;
frame_nbr=0;
dfdesired=12.0;
for o=1: size(ObjectPositions,1)
    
    % Submechanims, mobile base and arm
    for i=1:(size(w_T_arm,2)-1)
        iTip1=inv(w_T_arm{i})*w_T_arm{i+1};
        iPip1=iTip1(1:3,4);
        if(norm(iPip1)<0.001)% Links are coicident
            i;
        else  % Links are not
            for j=1:numberSteps
               
                distance_along_link=j*(w_T_arm{i}(1:3,1:3)*iPip1)/numberSteps;
                cp=w_T_arm{i}(1:3,4)+distance_along_link;
                %publishControlPoint(cp,frame_nbr);
                Lhat=skew(distance_along_link);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ=screwTransform*mb_J_arm{i};
                rdiff=ObjectPositions(o,:)'-cp;
                n=(rdiff)/norm(rdiff);
                JJ=[JJ, zeros(6,length(qdot_arm_max)-size(JJ,2))];
                if(norm(rdiff)<0.001)
                    rdiff=0.001;
                end
                
                                % Mobile base Constraints
                % we simply screw transform the jacobian to the control point.
                cp=mb_T_arm{i}(1:3,4)+distance_along_link;
                
                frame_nbr=frame_nbr+1;
                Lhat=skew(cp);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ_mb=screwTransform*w_J_mb{2};
                JJ_mb=[JJ_mb, zeros(6,total_mb-size(JJ_mb,2))]; % Ensure dimensionality consistency
                
                eta=(dfdesired*norm(rdiff)^2)-norm(rdiff);
                eta_arm=checkEtaValue(n,JJ,Q_arm_init.V);
                eta_mb=checkEtaValue(n,JJ_mb,Q_mb_init.V);
                
                
                if(any(eta_arm>eta) && any(eta_mb>eta))
                  %  disp 'Both exceed'
                    A=[A;n'*JJ(1:3,:)];
                    B=[B;0.5*eta];
                    Amb=[Amb;n'*JJ_mb(1:3,:)];
                    Bmb=[Bmb;0.5*eta];
                elseif(any(eta_arm>eta))
                   % disp 'Arm exceeds'
                    A=[A;n'*JJ(1:3,:)];
                    B=[B;eta-max(eta_mb)];
                elseif(any(eta_mb>eta))
                    %disp 'Mobile Base exceeds'
                    Amb=[Amb;n'*JJ_mb(1:3,:)];
                    Bmb=[Bmb;eta-max(eta_arm)];
                else
                  %  disp 'None exceed'
                end
            end
        end
    end    
end


Q_arm=Polyhedron(A(1:4,:),B(1:4));Q_arm.computeVRep;
V_arm= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm);
Q_mb=Polyhedron(Amb(1:4,:),Bmb(1:4)); Q_mb.computeVRep;
V_mb= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb);
V_arm_mb=Polyhedron(minkowskiSum(V_arm.V,V_mb.V));

Q_arm_obs_jl=Polyhedron(A,[PHI*B(1:4);B(5:end)]); Q_arm_obs_jl.computeVRep;
Q_mb_obs_jl=Polyhedron(Amb,[PHI_mb*Bmb(1:4);Bmb(5:end)]); Q_mb_obs_jl.computeVRep;
V_arm_obs_jl= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm_obs_jl);
V_mb_obs_jl= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb_obs_jl);
V_arm_mb_objs_jl=Polyhedron(minkowskiSum(V_mb_obs_jl.V,V_arm_obs_jl.V));


Q_mink=Polyhedron(minkowskiSum([Q_mb.V,zeros(size(Q_mb.V,1),2)],[zeros(size(Q_arm.V,1),2),Q_arm.V]));
Qjs_mink=Polyhedron(minkowskiSum([Q_mb_obs_jl.V,zeros(size(Q_mb_obs_jl.V,1),2)],[zeros(size(Q_arm_obs_jl.V,1),2),Q_arm_obs_jl.V]));

%wp_star=-V_arm_mb.volume;
%wp_star=(V_arm_mb.volume-V_arm_mb_objs_jl.volume)^2;
wp_star=-V_arm_mb_objs_jl.volume;
%wp_star=(Q_mink.volume-Qjs_mink.volume)^2;

% V_arm_mb_objs_jl.volume
% V_arm_mb.volume
% eta=Qjs_mink.volume/Q_mink.volume

end

