clear all,clc,close all


[RobotChain,gains]=getRobotParameters()
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
addpath(genpath('/home/philip/tbxmanager'))
PARAMS
%%
%roslaunch numl_val_motion_planning val_model.launch 
%
try
joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
catch
    rosinit
joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
end
val=importrobot('valkyrie_sim.urdf');
val.DataFormat='column';
r=importrobot('valkyrie_sim.urdf');

%%
Q = receive(joint,5.0);
q_config=homeConfiguration(r);
q=zeros(32,1);
qnames=cell(32,1);

for i=1:length(q_config)    
    for j=1:length(Q.Name)
        if(all(strcmp( q_config(i).JointName,Q.Name(j))))
            q_config(i).JointPosition=Q.Position(j);
            q(i)=Q.Position(j);
            qnames(i)=Q.Name(j);
        end
    end    
end

%% Objects, plot world
lfTp=eye(4);lfTp(1,4)=0.55; lfTp(3,4)=-0.090;
objects=getObjectsLeftFootFrame(lfTp);
showObjects(objects);

op_P_d=[0.6,0.4,0.9]';
lf_P_d=lfTp*[op_P_d;1];
publishControlPoint(lf_P_d,1,'leftFoot');

%% STEP 1 Formulate an IK problem that takes joint variables and lf_T_opt

fmincon_options= optimoptions('fmincon','Algorithm','sqp');

x_op1=lfTp(1,4);
y_op1=lfTp(2,4);
theta_op1=0.0;
x0=[x_op1;y_op1;theta_op1;q]; % Decision variables are leftfoot pose and joint variables

fun = @costWp;
nonlcon=@nlonPose;
op_P_d=[0.6,0.4,0.9]';
A=[];
b=[];
Aeq=[];
beq=[];
ub=zeros(35,1);
lb=zeros(35,1);

lb(1:3)=[-2.0,-2.0,-pi];
ub(1:3)=[2.0,2.0,pi];
for i=1:length(qnames)
    for j=1:length(RobotChain.JOINT_NAMES)
        if(strcmp(qnames(i),RobotChain.JOINT_NAMES(j)))
            ub(3+i)=RobotChain.JOINT_POSITION_MAX(j);
            lb(3+i)=RobotChain.JOINT_POSITION_MIN(j);
        end
    end    
end

[x,fval,exitflag,output,lambda]=fmincon(@(x)fun(x,val,x0(4:end),qnames),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(op_P_d,x,val),fmincon_options);
figure(2)
x_lf=x(1:3);
x_joints=x(4:end);
show(val,x_joints)
all(x<=ub)
all(x>=lb)
[c,ceq]=bestIKconstraints(op_P_d,x,val);
%%
lfTp_res=constructOpFrame(x_lf)
inv(lfTp_res)*getTransform(val,x_joints,'leftPalm')*Tne;
lf_P_d=lfTp_res*[op_P_d;1];
publishControlPoint(lf_P_d,1,'leftFoot');
% Display rviz
lfTp_res=constructOpFrame(x_lf);
objects=getObjectsLeftFootFrame(lfTp_res);
showObjects(objects);
while(1)
showSolutioninRviz(x_joints,qnames,objects)
publishControlPoint(lf_P_d,1,'leftFoot');
pause(1.0)
end

