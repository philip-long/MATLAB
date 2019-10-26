% http://www.orocos.org/node/829
% I have checked the hessian symbolically and this is working!! :0
% Need to tidy this code up and functionalize it
clear all,clc,close all
load('Hessian_mats')
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))

qdot_arm_max=2;
qdot_arm_min=-4;

%
qpos=randRange(-pi,pi,7)



qpos=randRange(-pi,pi,7);
joint=randi(7);
%joint=1;
J=J70(qpos);
T=T70(qpos);
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;
joint
E=[];
step=0.0001;
for q3=0:step:pi
    Jlast=JE;
    
    qpos(joint)=q3;
    T=T70(qpos);
    J=J70(qpos);
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
    
    H=getHessian(JE);
    
    
    
    % [ dx/dq1dq1 dx/dq1dq2 dx/dq1dq3 ... dx/dq1dqn]
    % [ dx/dq2dq1 dx/dq2dq2 dx/dq2dq3 ... dx/dqdqn]
    numerical_gradient=(JE-Jlast)/step;
    rows=randi(6);
    cols=randi(7);
    numerical_gradient- H{joint};
    
    E=[E;norm(numerical_gradient- H{joint})];
    
    
end
plot(E(2:end))