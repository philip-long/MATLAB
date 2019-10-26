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

step=0.005


Gamma_plus=zeros(3,1);
Gamma_minus=zeros(3,1);


hp=zeros(3,1);
hm=zeros(3,1);

qdot_max=ones(length(active_joint),1)*qdot_arm_max;
qdot_min=ones(length(active_joint),1)*qdot_arm_min;
deltaq=qdot_max-qdot_min;

A=[eye(length(active_joint)) ;-eye(length(active_joint))];
B=[ones(length(active_joint),1)*qdot_arm_max ;ones(length(active_joint),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;

Gamma_p_gradient=cell(length(active_joint),1);
Gamma_m_gradient=cell(length(active_joint),1);
E=[]
Em=[]


for q3=0:step:pi
    
    
    % Checking the numerical gradient
    Gamma_last=Gamma_plus;
    Gamma_last_m=Gamma_minus;
    
    
    hp_last=hp;    
    hm_last=hm;
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
    [n,hp,hm,n_grad,hp_grad,hm_grad]=getHyperplanes(JE,HE,deltaq,active_joint);
    %
    
    
    % Gamma_plus is capacity margin where positive means vertex is inside
    % polytope and negative outside. Gamma_plus has as many rows as
    % hyperplanes in cartesian velocity polytope and as many columns as
    % vertices in desired set. One element of cell Gamma_p_gradient{joint} has the
    % same dimension as Gamma_plus indicating the change of Gamma_plus
    % w.r.t to that chosen joint
    
    
    for vertex=1:size(desired_twist.V,1)
        Gamma_plus(:,vertex)=hp + n*JE*qdot_min -  (n * desired_twist.V(vertex,:)');
        %  Gamma_p_gradient{joint}(:,vertex)=hp_grad{joint} + (n_grad{joint} *JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) - (n_grad{joint} *desired_twist.V(vertex,:)');
        Gamma_minus(:,vertex)=hm + n*JE*qdot_min+(n * desired_twist.V(vertex,:)');
        % Gamma_m_gradient{joint}(:,vertex)=hm_grad{joint}  + (n_grad{joint} *JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) + (n_grad{joint} *desired_twist.V(vertex,:)');
        for joint_np=1:size(JE,2)
            Gamma_p_gradient{joint_np}(:,vertex)=hp_grad{joint_np} + (n_grad{joint_np} *JE*qdot_min)  + (n*HE{joint_np}(1:3,:)*qdot_min) - (n_grad{joint_np} *desired_twist.V(vertex,:)');
            Gamma_m_gradient{joint_np}(:,vertex)=hm_grad{joint_np}  + (n_grad{joint_np} *JE*qdot_min)  + (n*HE{joint_np}(1:3,:)*qdot_min) + (n_grad{joint_np} *desired_twist.V(vertex,:)');
        end
    end
    
     
    numerical_grad_hp=(hp-hp_last)/step;
    numerical_grad_hm=(hm-hm_last)/step;
    
    
    numerical_grad_gamma=(Gamma_plus-Gamma_last)/step;
    numerical_grad_gamma_m=(Gamma_minus-Gamma_last_m)/step;
    
       Gamma_p_gradient{joint};
    Gamma_m_gradient{joint};
  %  E=[E;norm(numerical_grad_gamma-    Gamma_p_gradient{joint})];
  %  Em=[Em;norm(numerical_grad_gamma_m-    Gamma_m_gradient{joint})];
    
    E=[E;norm(numerical_grad_hp-hp_grad{joint})];
    Em=[Em;norm(numerical_grad_hm-hm_grad{joint})];
    
    
    %     numerical_grad_gamma_sum=(sum(sum(Gamma_plus))-sum(sum(Gamma_last)))/step
    %     sum(sum(Gamma_p_gradient{joint}))
    %pause()
end
plot(E(2:end))
figure(2)
plot(Em(2:end))