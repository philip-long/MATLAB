%% Solving the inverse kinematic model using the gradient of GAMMA

clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager')) % https://www.tbxmanager.com/#

qdot_arm_max=2;
qdot_arm_min=-4;

% Capcity margin


active_joint=1:7; % Select considered joints

qposd=randRange(-pi,pi,7); % desired joint position
q=randRange(-pi,pi,7); % Current position

qdot_arm_max=2; % Joint velocity upper limit
qdot_arm_min=-4; % Joint velocity lower limit

qdot_max=ones(7,1)*qdot_arm_max;
qdot_min=ones(7,1)*qdot_arm_min;

PA_desired=[eye(3) ;-eye(3)]; % Desired polytope hyperplanes
PB_desired=[5 ; 0.2 ;0.2 ;5.0;0.2;0.2]; % Desired polytope normalized distance
desired_twist=Polyhedron(PA_desired,PB_desired); % desired twist polytope

Td=T70(qposd); % Desired Transformation Matrix
pos_des=Td(1:3,4); % Extract desired position
q0=rand(7,1); % initial Solution


%% Inverse Kinematic Model with Gamma as a constraint


% Inverse Kinematic Model with Gamma as a constraint using gradient

fmincon_options= optimoptions('fmincon','Algorithm','sqp','SpecifyConstraintGradient',true,'CheckGradient',true,'Plotfcn','optimplotfval');
nonlcon=@gammaConstraint; % non-linear constraint function with gradient
%nonlcon2=@manipConstraint
fun = @(q)norm(pos70(q)-pos_des)*10000;
error0=fun(q0)

ub=[]; % upper bounds on decision variables
lb=[]; % lower bounds on decision variables
tic
 [q,fval_g,out_g] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
    @(q)nonlcon(q,qdot_max,qdot_min,desired_twist.V),fmincon_options)
g_time=toc;


figure(2)

vol_sol_gradient=plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,'r');
vol=plotPolyForJointValue(q0,qdot_arm_max,qdot_arm_min,'g');
hold on
desired_twist.plot('color','b')
title('Using Gradient')
legend('Opt solution','Desired','Initial solution')

pause()
% Inverse Kinematic Model with Gamma as a constraint without using gradient
figure(3)
fmincon_options= optimoptions('fmincon','Algorithm','sqp','Plotfcn','optimplotfval');
nonlcon_nograd=@gammaConstraintNoGrad; % non-lineare constraint function without gradient

tic
 [q,fval_ng,out_ng] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
    @(q)nonlcon_nograd(q,qdot_max,qdot_min,desired_twist.V),fmincon_options)
ng_time=toc
figure(4)
vol_sol_no_gradient=plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,'r');
plotPolyForJointValue(q0,qdot_arm_max,qdot_arm_min,'g');
hold on
desired_twist.plot('color','b')
title('No Gradient ')
legend('Opt solution','Desired','Initial solution')
%[q,fval] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
%   @(q)nonlcon(q),fmincon_options);

%  [c,ceq,c_grad,ceq_grad]=nonlcon(q,qdot_max,qdot_min,desired_twist.V);
%  fval
%  
 
 
 
 
 