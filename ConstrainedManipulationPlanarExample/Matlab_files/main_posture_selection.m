%% %% main_posture_selection
%
% Testing posture selection algorithm
% For a given target point obtain the best posture
% i.e find the inverse geometric solution that 
% maximises
% Find joint configuration such that wp* is maximized. 

close all,clear all,
clc
addpath(genpath('~/catkin_ws/src/constrained_manipulation/tbxmanager'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
% test configurations q1=[0.8,-0.9], q2=[0.8,-1.53]
Q=[];
F=[];
for i=1:2
options = optimoptions('fmincon','Display','iter','TolCon',0.00001);
fun = @postureOptimization;
A=[];
b=[];
Aeq=[];
beq=[];
lb=[];
ub=[];
xd=0.6; yd=0.6;
x0arm=getIGM(xd,yd);
x0=[0 0 x0arm(1,1) x0arm(1,2) ]+0.5*(rand(1,4)-0.5);
%x0=;
poseConstraint(x0,xd,yd);
nonlcon = @(x)poseConstraint(x,xd,yd);
[x,fval,exitflag]=fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

if(exitflag==2)
    Q=[Q;x];
    F=[F;fval];
end
end

fvalout=postureOptimization(x0)
%%

for i=1:size(Q,1)
close all
q1=Q(i,:);
T=getTransforms4DOF(q1);
n_T_ee=eye(4); n_T_ee(1,4)=0.55;   % terminal point
T{end+1}=T{end}*n_T_ee;
plotPlanarRobot(T,2)
plot(xd,yd,'ro','MarkerSize',15,'MarkerFace',[0.0,0.8,0.8])

ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
ObjectPositions(3,:)=[0.6;0.3;0.0]; % in world frame
ObjectPositions(4,:)=[0.4;0.8;0.0]; % in world frame
ObjectPositions(5,:)=[0.3;0.6;0.0]; % in world frame
ObjectPositions(6,:)=[0.35;0.2;0.0]; % in world frame
ObjectPositions(7,:)=[0.75;0.65;0.0]; % in world frame
ObjectPositions(8,:)=[0.6;0.8;0.0]; % in world frame
ObjectPositions(9,:)=[0.2;0.4;0.0]; % in world frame
ObjectPositions(10,:)=[0.6;0.4;0.0]; % in world frame

plotObjects(ObjectPositions,2);
axis('equal')
F(i)
pause()
end
%% ROS PUBLISH OBJECT POSITIONS
 
publishControlPoint( [xd,yd,0],10,'world')
for i=1:size(ObjectPositions,1)
    pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
    marker=publishPostureObject(ObjectPositions(i,:),i);
    send(pub,marker) % oublish the marker
end




%%

T=getTransforms4DOF(ql);
%postureOptimization(ql)
n_T_ee=eye(4); n_T_ee(1,4)=0.55;   % terminal point
T{end+1}=T{end}*n_T_ee;
plotPlanarRobot(T)
plot(xd,yd,'ro','MarkerSize',15,'MarkerFace',[0.0,0.8,0.8])

ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
ObjectPositions(3,:)=[0.6;0.3;0.0]; % in world frame
ObjectPositions(4,:)=[0.4;0.8;0.0]; % in world frame


plotObjects(ObjectPositions);
axis('equal')
postureOptimization(ql)

