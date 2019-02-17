%% main_mink_analysis
%
%
% roslaunch constrained_manipulation planar_humanoids_2018.launch
%  
%  This file obtains the constrained manipulability for a planar 
%  4 DOF robot. The current joint positions are read via the ros 
%  interface. Nbr of objects and their pose can be configured from line 97
%
%
% In this file we use the minkowski sum appraoch to create the composite
% mechanism. This is legitmate and it is shown by this approach. However,
% by treating the whole mechanism together great velocities can be attained
% by enforcing interdependency between joints. This is not possible by
% composite method. 


close all,clear all,
clc
addpath(genpath('~/tbxmanager'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
% test configurations q1=[0.8,-0.9], q2=[0.8,-1.53]
try
    joint= rossubscriber('/joint_states');
catch
    rosinit
    joint= rossubscriber('/joint_states');
end

scandata = receive(joint,0.5);
joint_names={'q0', 'q1','q2','q3'};
total_frames=length(joint_names);
total_mb=total_frames/2;
total_arm=total_frames/2;
q_all=scandata.Position(contains(scandata.Name,joint_names));
q_mb=q_all(1:2);
q_pos=q_all(3:4);
%%

% Define Mobile Base limits
% velocity
qdot_mb_max=ones(total_mb,1)*0.5;
qdot_mb_min=qdot_mb_max*-1;

q_mb_max=[2.0;2.0];
q_mb_min=[-2.0;-2.0];
[phi_max,phi_min]=getJointLimitPenalty(q_mb,q_mb_max,q_mb_min,4);
PHI_mb=eye(total_mb*2)-diag([phi_max;phi_min]);
w_T_mb=getTransforms4DOF(q_all); % Get all transforms
w_J_mb=getJacobians4DOF(q_all);  % Get all Jacobians

% Need to screw Transform mobile base to end effector.


%% Define arm limits
% velocity
qdot_arm_max=ones(total_arm,1)*1.0;
qdot_arm_min=qdot_arm_max*-1;
% position
q_arm_max=[pi/2;pi/2];
q_arm_min=[-pi/2;-pi/2];
[phi_max,phi_min]=getJointLimitPenalty(q_pos,q_arm_max,q_arm_min,4);
PHI=eye(total_arm*2)-diag([phi_max;phi_min]);

mb_T_arm=getTransforms(q_pos); % Get all transforms
mb_J_arm=getJacobians(q_pos);  % Get all Jacobians



n_T_ee=eye(4); n_T_ee(1,4)=0.55;   % terminal point
% Define Control point and publish frame
% This is necessary  since my chain ends with the last joint
[mb_T_arm_ee,mb_J_arm_ee]=getEEInfo(mb_T_arm{total_arm},mb_J_arm{total_arm},n_T_ee,'world');
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


Qs=expandJointPolytope([1 2],qdot_arm_max,qdot_arm_min);


%% Entire Mechanism togerth

[phi_max,phi_min]=getJointLimitPenalty(q_all,[q_mb_max;q_arm_max],[q_mb_min;q_arm_min],4);
PHI_mec=eye(total_frames*2)-diag([phi_max;phi_min]);
w_T_mec=getTransforms4DOF(q_all); % Get all transforms
w_J_mec=getJacobians4DOF(q_all);  % Get all Jacobians

[w_T_mec_ee,w_J_mec_ee]=getEEInfo(w_T_mec{total_frames},w_J_mec{total_frames},n_T_ee,'world');
w_T_mec{end+1}=w_T_mec_ee;
w_J_mec{end+1}=w_J_mec_ee;

%%  Object initialization
ObjectPositions=[];
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame

Obj_info={}; % is a cell (number of objects) of cells (number of links),

for i=1:size(ObjectPositions,1)
    LinkOrien=rot2Quat_xyzw(w_T_arm{i}(1:3,1:3)); % used just to align object and link for aesthetic reasons
    pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
    marker=publishObjectInfoRviz(ObjectPositions(i,:),'world',i,LinkOrien);
    send(pub,marker) % oublish the marker
end

%% Get Jacobians at all control points
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
Q_mec_init=Polyhedron(Amec,Bmec);Q_mec_init.computeVRep;
E=[];
E2=[];
frame_nbr=0;
dfdesired=12.0;
for o=1: size(ObjectPositions,1)
    
    
    % Submechanims, mobile base and arm
    for i=1:(size(w_T_arm,2)-1)
        iTip1=inv(w_T_arm{i})*w_T_arm{i+1};
        iPip1=iTip1(1:3,4)
        if(norm(iPip1)<0.001)% Links are coicident
            i
        else  % Links are not
            for j=1:numberSteps
                distance_along_link=j*(w_T_arm{i}(1:3,1:3)*iPip1)/numberSteps;
                cp=w_T_arm{i}(1:3,4)+distance_along_link;
                publishControlPoint(cp,frame_nbr);
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
                
                % To respect desired dangerfield constraints
                % This factor needs to be reduced by the number of mechanisms
                % as each mechanism is just below the allowable velocity
                % the sum will exceed it. This is conservative because
                % we assume each sub-mechanims is generate a danger
                % exceeding the threshold, whereas in reality it is not.
                % So to be clear suppose we've a desired danger=10
                % One sub-mechanims generates no danger. Second mechamnism
                % is free to gemnerate velocity up to 10
                
                
                eta=(dfdesired*norm(rdiff)^2)-norm(rdiff);
                E=[E;eta];
                eta_arm=checkEtaValue(n,JJ,Q_arm_init.V);
                eta_mb=checkEtaValue(n,JJ_mb,Q_mb_init.V);
                
                
                if(any(eta_arm>eta) && any(eta_mb>eta))
                    disp 'Both exceed'
                    A=[A;n'*JJ(1:3,:)];
                    B=[B;0.5*eta];
                    Amb=[Amb;n'*JJ_mb(1:3,:)];
                    Bmb=[Bmb;0.5*eta];
                    size(Amb)
                    size(Bmb)
                elseif(any(eta_arm>eta))
                    disp 'Arm exceeds'
                    A=[A;n'*JJ(1:3,:)];
                    B=[B;eta-max(eta_mb)];
                elseif(any(eta_mb>eta))
                    disp 'Mobile Base exceeds'
                    Amb=[Amb;n'*JJ_mb(1:3,:)];
                    Bmb=[Bmb;eta-max(eta_arm)];
                else
                    disp 'None exceed'
                end
            end
        end
    end
    
    % Composite mechanism
    for i=1:(size(w_T_mec,2)-1)
        iTip1=inv(w_T_mec{i})*w_T_mec{i+1};
        iPip1=iTip1(1:3,4);
        if(norm(iPip1)<0.02)% Links are coicident
            i;
        else  % Links are not
            for j=1:numberSteps
                distance_along_link=j*(w_T_mec{i}(1:3,1:3)*iPip1)/numberSteps;
                cp=w_T_mec{i}(1:3,4)+distance_along_link;
                publishControlPoint(cp,frame_nbr);
                Lhat=skew(distance_along_link);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ_mec=screwTransform*w_J_mec{i};
                rdiff=ObjectPositions(o,:)'-cp;
                n=(rdiff)/norm(rdiff);
                JJ_mec=[JJ_mec, zeros(6,total_frames-size(JJ_mec,2))];
                
                eta=(dfdesired*norm(rdiff)^2)-norm(rdiff);
                E2=[E2;eta];
                eta_mec=checkEtaValue(n,JJ_mec,Q_mec_init.V);
                if(any(eta_mec>eta))
                    disp 'Mobile Base exceeds'
                    Amec=[Amec;n'*JJ_mec(1:3,:)];
                    Bmec=[Bmec;eta];
                end
                
            end
        end
    end
    
    
    
    
end


%% plot Joint Polytope

% Arm Joint Polytope
Q_arm=Polyhedron(A(1:4,:),B(1:4));Q_arm.computeVRep;
% Arm Joint Polytope after object Deformation
Q_arm_obs=Polyhedron(A,B); Q_arm_obs.computeVRep;
% Arm Joint Polytope after joint limits
Q_arm_jl=Polyhedron(A(1:4,:),PHI*B(1:4)); Q_arm_jl.computeVRep;
% Joint Polytope and Joint limits
Q_arm_obs_jl=Polyhedron(A,[PHI*B(1:4);B(5:end)]); Q_arm_obs_jl.computeVRep;
%
Q_mb=Polyhedron(Amb(1:4,:),Bmb(1:4)); Q_mb.computeVRep;
Q_mb_obs=Polyhedron(Amb,Bmb); Q_mb_obs.computeVRep;
Q_mb_jl=Polyhedron(Amb(1:4,:),PHI_mb*Bmb(1:4)); Q_mb_jl.computeVRep;
Q_mb_obs_jl=Polyhedron(Amb,[PHI_mb*Bmb(1:4);Bmb(5:end)]); Q_mb_obs_jl.computeVRep;

Q_mec=Polyhedron(Amec(1:8,:),Bmec(1:8)); Q_mec.computeVRep;
Q_mec_obs=Polyhedron(Amec,Bmec); Q_mec_obs.computeVRep;
Q_mec_jl=Polyhedron(Amec(1:8,:),PHI_mec*Bmec(1:8)); Q_mec_jl.computeVRep;
Q_mec_obs_jl=Polyhedron(Amec,[PHI_mec*Bmec(1:8);Bmec(9:end)]); Q_mec_obs_jl.computeVRep;


Qjs_mink=Polyhedron(minkowskiSum([Q_mb_obs.V,zeros(size(Q_mb_obs.V,1),2)],[zeros(size(Q_arm_obs.V,1),2),Q_arm_obs.V]));
Qjs_mink.minVRep

% PQ.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
% hold on
% %PQ_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
% PQ_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
% axis([-1 1 -1 1])
% xlabel('$\dot{q}_{1} \ [s^{-1}]$','Interpreter','latex','FontSize',18)
% ylabel('$\dot{q}_{2} \ [s^{-1}]$','Interpreter','latex','FontSize',18)

%% plot Cartesian Polytope
figure(2)
%ellipsoidCreate(mb_J_arm_ee)
hold on


V_arm= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm);
V_arm_obs= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm_obs);
V_arm_jl= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm_jl);
V_arm_obs_jl= getCartesianPolytope(w_J_arm{end}(1:2,:),Q_arm_obs_jl);


V_mb= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb);
V_mb_obs= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb_obs);
V_mb_jl= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb_jl);
V_mb_obs_jl= getCartesianPolytope(w_J_mb{end}(1:2,1:2),Q_mb_obs_jl);

V_mec= getCartesianPolytope(w_J_mec{end}(1:2,:),Q_mec);
V_mec_obs= getCartesianPolytope(w_J_mec{end}(1:2,:),Q_mec_obs);
V_mec_jl= getCartesianPolytope(w_J_mec{end}(1:2,:),Q_mec_jl);
V_mec_obs_jl= getCartesianPolytope(w_J_mec{end}(1:2,:),Q_mec_obs_jl);

% Why is polytope minkowski sum contained in the mechanism
% polytope?
% It is smaller as it assumes the mechanisms can operate
% independantly whereas the composite mechanims encodes
% dependancies between joint velocity directions.

V_arm.plot('color',[1.0 0.0 0.0],'Alpha',0.01,'LineStyle',':','LineWidth',0.1,'Marker','s')
hold on
V_arm_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.01,'LineStyle',':','LineWidth',0.1)%,'Marker','x')
V_arm_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.01,'LineStyle',':','LineWidth',0.1)%,'Marker','o','LineStyle',':')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')
axis([-0.65 0.65,-1.75 1.75])

V_mb.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
hold on
V_mb_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
V_mb_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')
axis([-0.65 0.65,-1.75 1.75])

%% Tests minkowski sum
figure(3)
% Check 1-> ensure that minkowski sum of polytopes equals mechanism
% polytope [OK]
V_arm_mb=Polyhedron(minkowskiSum(V_arm.V,V_mb.V));

V_mec.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
hold on
V_arm_mb.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')

% Check 2 ensure that minkowski sum of polytopes with joint limit equals mechanism
% polytope [OK]
V_arm_mb_jl=Polyhedron(minkowskiSum(V_arm_jl.V,V_mb_jl.V));

V_arm_mb_jl.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x','LineStyle','--')
hold on
V_mec_jl.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')

% Check 3 ensure that minkowski sum of polytopes with all constraint is within mechanism
% polytope.

V_arm_mb_objs=Polyhedron(minkowskiSum(V_mb_obs.V,V_arm_obs.V));

V_arm_mb_objs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
hold on
V_mec_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')


V_arm_mb_objs_jl=Polyhedron(minkowskiSum(V_mb_obs_jl.V,V_arm_obs_jl.V));

V_arm_mb_objs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
hold on
V_mec_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')

%%

% Sanity Check: Ensuring all the reduced Polytopes respect the dangerfield values. Which
% they do.

for o=1: size(ObjectPositions,1)
    
    % Submechanims, mobile base and arm
    for i=1:(size(w_T_arm,2)-1)
        iTip1=inv(w_T_arm{i})*w_T_arm{i+1};
        iPip1=iTip1(1:3,4)
        if(norm(iPip1)<0.001)% Links are coicident
            i
        else  % Links are not
            for j=1:numberSteps
                distance_along_link=j*(w_T_arm{i}(1:3,1:3)*iPip1)/numberSteps;
                cp=w_T_arm{i}(1:3,4)+distance_along_link;
                publishControlPoint(cp,frame_nbr);
                Lhat=skew(distance_along_link);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ=screwTransform*mb_J_arm{i};
                rdiff=ObjectPositions(o,:)'-cp;
                n=(rdiff)/norm(rdiff);
                JJ=[JJ, zeros(6,length(qdot_arm_max)-size(JJ,2))];
                
                
                
                
                if(norm(rdiff)<0.001)
                    rdiff=0.001;
                end
                
                N_arm_pre=checkDangerValue(n,JJ(1:3,:), Q_arm.V,rdiff)
                N_arm_post=checkDangerValue(n,JJ(1:3,:), Q_arm_obs.V,rdiff)
                
                
                % Mobile base Constraints
                % we simply screw transform the jacobian to the control point.
                cp=mb_T_arm{i}(1:3,4)+distance_along_link;
                
                frame_nbr=frame_nbr+1;
                Lhat=skew(cp);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ=screwTransform*w_J_mb{2};
                JJ=[JJ, zeros(6,total_mb-size(JJ,2))]; % Ensure dimensionality consistency
                
                N_mb_pre=checkDangerValue(n,JJ(1:3,:), Q_mb.V,rdiff)
                N_mb_post=checkDangerValue(n,JJ(1:3,:), Q_mb_obs.V,rdiff)
                if(any(N_mb_post>6.001))
                    disp ' ERROR'
                    pause()
                end
                pause()
                
                disp '============================'
                
            end
        end
    end
    
    % Composite mechanism
    for i=1:(size(w_T_mec,2)-1)
        iTip1=inv(w_T_mec{i})*w_T_mec{i+1};
        iPip1=iTip1(1:3,4)
        if(norm(iPip1)<0.02)% Links are coicident
            disp ' Links coincident'
        else  % Links are not
            for j=1:numberSteps
                distance_along_link=j*(w_T_mec{i}(1:3,1:3)*iPip1)/numberSteps;
                cp=w_T_mec{i}(1:3,4)+distance_along_link;
                publishControlPoint(cp,frame_nbr);
                Lhat=skew(distance_along_link);
                screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
                JJ=screwTransform*w_J_mec{i};
                rdiff=ObjectPositions(o,:)'-cp;
                n=(rdiff)/norm(rdiff);
                JJ=[JJ, zeros(6,total_frames-size(JJ,2))];
                
                % get vertices and calculate actual dangervalue;
                
                if(norm(rdiff)<0.001)
                    rdiff=0.001;
                end
                
                
                N_mb_pre=checkDangerValue(n,JJ(1:3,:), Q_mec.V,rdiff)
                N_mb_post_mec=checkDangerValue(n,JJ(1:3,:), Q_mec_obs.V,rdiff)
                N_mb_post_mink=checkDangerValue(n,JJ(1:3,:), Qjs_mink.V,rdiff)
                N_mb_post_mb=checkDangerValue(n,JJ(1:3,1:2), Q_mb_obs.V,rdiff)
                N_mb_post_arm=checkDangerValue(n,JJ(1:3,3:4), Q_arm_obs.V,rdiff)
                disp '============================'
                pause()
            end
        end
    end
    
    
    
    
end

