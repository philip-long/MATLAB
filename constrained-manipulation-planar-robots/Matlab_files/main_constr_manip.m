%% main_v1
%
% roslaunch constrained_manipulation planar_humanoids_2018.launch
%  
%  This file obtains the constrained manipulability for a planar 
%  2 DOF robot. The current joint positions are read via the ros 
%  interface, even though the system is 4DOF we only consider 2DOF arm 
%  here. Nbr of objects and their pose can be configured from line 47

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
joint_names={'q2', 'q3'};
total_frames=length(joint_names);
q_pos=scandata.Position(contains(scandata.Name,joint_names));
%%

% Define joint limits
qp_upperlimits=ones(total_frames,1)*1.0;
qp_lowerlimits=qp_upperlimits*-1;
q_upper=[pi/2;pi/2];
q_lower=[-pi/2;-pi/2];
[phi_max,phi_min]=getJointLimitPenalty(q_pos,q_upper,q_lower,4);
PHI=eye(total_frames*2)-diag([phi_max;phi_min]);

n=length(qp_upperlimits);
T=getTransforms(q_pos); % Get all transforms
J=getJacobians(q_pos);  % Get all Jacobians
Threshold=0.13;               % set saftey threshold
TnE=eye(4); TnE(1,4)=0.55;   % terminal point
% Define Control point and publish frame
% This is necessary  since my chain ends with the last joint
[T0E,J0E]=getEEInfo(T{total_frames},J{total_frames},TnE,'world');
T{end+1}=T0E;
J{end+1}=J0E;
Qs=expandJointPolytope([1 2],qp_upperlimits,qp_lowerlimits);

%%  Object initialization
ObjectPositions=[]
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame

Obj_info={}; % is a cell (number of objects) of cells (number of links),

for i=1:size(ObjectPositions,1)
    LinkOrien=rot2Quat_xyzw(T{i}(1:3,1:3)); % used just to align object and link    
    pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
    marker=publishObjectInfoRviz(ObjectPositions(i,:),'world',i,LinkOrien);
    send(pub,marker) % oublish the marker
end

%% Get Jacobians at all control points
Jaclist={};
Linklist={};
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
            Linklist{end+1}=cp;
            Lhat=skew(distance_along_link);
            screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
            JJ=screwTransform*J{i};
            rdiff=ObjectPositions(o,:)'-cp;
            n=(rdiff)/norm(rdiff);
            JJ=[JJ, zeros(6,length(qp_upperlimits)-size(JJ,2))];
            Jaclist{end+1}=JJ;
            A=[A;n'*JJ(1:3,:)];
            
            if(norm(rdiff)<0.001) 
                rdiff=0.001;
            end     
            B=[B;(dfdesired*norm(rdiff)^2)-norm(rdiff)];
        end
    end
end
end


%% plot Joint Polytope

% Original Joint Polytope
PQ=Polyhedron(A(1:4,:),B(1:4));
% Joint Polytope after object Deformation
PQ_obs=Polyhedron(A,B); PQ_obs.computeVRep;
% Joint Polytope and Joint limits
PQ_obs_jl=Polyhedron(A,[PHI*B(1:4);B(5:end)]); PQ_obs_jl.computeVRep;


PQ.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
hold on
%PQ_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
PQ_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
axis([-1 1 -1 1])
xlabel('$\dot{q}_{1} \ [s^{-1}]$','Interpreter','latex','FontSize',18)
ylabel('$\dot{q}_{2} \ [s^{-1}]$','Interpreter','latex','FontSize',18)

%% plot Cartesian Polytope
h=figure(2)
%ellipsoidCreate(J0E)
hold on
V_obs=zeros(length(PQ_obs.V),2);
V_obs_jl=zeros(length(PQ_obs_jl.V),2);
V_orig=zeros(length(PQ.V),2);
for i=1:length(PQ_obs.V)
V_obs(i,:)=(J0E(1:2,:)*( PQ_obs.V(i,:))')';
end

for i=1:length(PQ_obs_jl.V)
V_obs_jl(i,:)=(J0E(1:2,:)*( PQ_obs_jl.V(i,:))')';
end

for i=1:length(PQ.V)
V_orig(i,:)=(J0E(1:2,:)*( PQ.V(i,:))')';
end

Pv=Polyhedron(V_orig);
Pv_obs=Polyhedron(V_obs);
Pv_obs_jl=Polyhedron(V_obs_jl);

Pv.plot('color',[1.0 0.0 0.0],'Alpha',0.2,'LineStyle','-','LineWidth',1.0,'Marker','s')
hold on
Pv_obs.plot('color',[0.0 1.0 0.0],'Alpha',0.4,'LineWidth',1.5,'Marker','x')
Pv_obs_jl.plot('color',[0.0 0.0 1.0],'Alpha',0.6,'LineWidth',3.0,'Marker','o','LineStyle',':')
xlabel('v_{x} [m s^{-1}]','Interpreter','Tex')
ylabel('v_{y} [m s^{-1}]','Interpreter','Tex')

%% 
Pv_obs_jl.volume
Pv.volume
ratio=Pv_obs_jl.volume/Pv.volume

