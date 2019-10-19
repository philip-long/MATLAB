% Attempt 1 to get analytical gradient

clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))

qdot_arm_max=2;
qdot_arm_min=-4;

% Capcity margin
PA_desired=[eye(3) ;-eye(3)];
PB_desired=[0.1 ; 0.1 ;0.1 ;0.4;0.3;0.2];
desired_twist=Polyhedron(PA_desired,PB_desired);

active_joint=1:3; % Only considering 4 joint for illustrative case


%%
G=[]
qpos=randRange(-pi,pi,7);
joint=randi(4);

step=0.01
Gamma_i=0.0;

Gamma_all=zeros(6,1);

qdot_max=ones(length(active_joint),1)*qdot_arm_max;
qdot_min=ones(length(active_joint),1)*qdot_arm_min;
deltaq=qdot_max-qdot_min;

A=[eye(length(active_joint)) ;-eye(length(active_joint))];
B=[ones(length(active_joint),1)*qdot_arm_max ;ones(length(active_joint),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;

for q3=0:step:pi
    
    
    % Checking the numerical gradient
    Gamma_last=Gamma_i;
    Gamma_all_last=Gamma_all;
    %
    
    qpos(joint)=q3;
    T=T70(qpos);
    J=J70(qpos);
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
    
    
    
    V_arm= getCartesianPolytope(JE(1:3,active_joint),Q_arm);

    
    %
    HE=getHessian(JE(:,active_joint));
    JE=JE(1:3,active_joint);
    [n,hp,hm,n_grad,hp_grad,hm_grad]=getHyperplanes(JE,HE,joint,deltaq,active_joint);
    %
    
    % Gradient of Gamma all CHECKED
    Gamma_plus=hp + n*JE*qdot_min -  (n * desired_twist.V(1,:)')
    Gamma_p_gradient=hp_grad + (n_grad*JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) - (n_grad*desired_twist.V(1,:)')
    
    Gamma_minus=hm + n*JE*qdot_min+(n * desired_twist.V(1,:)')
    Gamma_m_gradient=hm_grad + (n_grad*JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) + (n_grad*desired_twist.V(1,:)')
    
    

end

