clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))

qdot_arm_max=2;
qdot_arm_min=-4;

% Capcity margin


active_joint=1:7; % Only considering 4 joint for illustrative case

qposd=randRange(-pi,pi,7);
q=randRange(-pi,pi,7);

qdot_arm_max=2;
qdot_arm_min=-4;

qdot_max=ones(7,1)*qdot_arm_max;
qdot_min=ones(7,1)*qdot_arm_min;

PA_desired=[eye(3) ;-eye(3)];
PB_desired=[5 ; 0.2 ;0.2 ;5.0;0.2;0.2];
desired_twist=Polyhedron(PA_desired,PB_desired);


joint=randi(4);

Td=T70(qposd);
pos_des=Td(1:3,4);

q0=rand(7,1);


%% Trying Gamma as a constraint


fmincon_options= optimoptions('fmincon','Algorithm','sqp','CheckGradients',true,'SpecifyConstraintGradient',true);
nonlcon=@gammaConstraint
%nonlcon2=@manipConstraint
fun = @(q)norm(pos70(q)-pos_des)*1000;
error0=fun(q0)

ub=[]
lb=[]
 [q,fval,out] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
    @(q)nonlcon(q,qdot_max,qdot_min,desired_twist.V),fmincon_options)

%  [q,fval] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
%     @(q)nonlcon2(q,qdot_max,qdot_min,desired_twist.V),fmincon_options);

plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,'r');
plotPolyForJointValue(q0,qdot_arm_max,qdot_arm_min,'g');




hold on
desired_twist.plot('color','b')

%[q,fval] = fmincon(fun,q0,[],[],[],[],lb,ub,... 
%   @(q)nonlcon(q),fmincon_options);

%  [c,ceq,c_grad,ceq_grad]=nonlcon(q,qdot_max,qdot_min,desired_twist.V);
%  fval
%  
 
 
 
 
 