
clear all,clc,close all
clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))
%%
q=randRange(-pi,pi,7);
qmax=ones(7,1)*pi;
qmin=ones(7,1)*-pi;
qdot_arm_max=4;
qdot_arm_min=2;
qdot_max=ones(7,1)*qdot_arm_max;
qdot_min=ones(7,1)*qdot_arm_min;
deltaq=qdot_max-qdot_min;
deltaq=[deltaq;(qmax-q')-(qmin-q')]

% Get kinematics
T=T70(q);
J=J70(q);
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;
% Get Hessian
HE=getHessian(JE);
JE=JE(1:3,:);
JE=[JE;eye(7)];

[n,hplus,hminus,d_n_dq,d_hplus_dq,d_hminus_dq] = getHyperplanes(JE,HE,deltaq,1:7);
 
A=[eye(length(q)) ;-eye(length(q))];
B=[ones(length(q),1)*qdot_arm_max ;ones(length(q),1)*-qdot_arm_min];

 vol_sol_no_gradient=plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,'r');
An=[n
    -n];
bn=[hplus;-hminus];
P=Polyhedron(An,bn);
vmin=JE*qdot_min;
hold on
P.plot('color','b')
P.volume
% Finally need to shift this back away from origin
Pshifted=Polyhedron(P.V+vmin');
Pshifted.volume
Pshifted.plot('color','g')
