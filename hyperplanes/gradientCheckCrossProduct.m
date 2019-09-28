% Gradient of a cross product of two vectors
clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))

cv1v2=zeros(3,1);
v1=zeros(3,1);
v2=zeros(3,1);
step=0.005;
NUM=[]
ANALYTIC=[]
for q=-rand(1)*2:step:rand(1)*2
    
    cv1v2_last=cv1v2;
    v1_last=v1;
    v2_last=v2;
    
    v1=[q^2 + 6*q;-6*q^2 + 12*q; 1*q];
    dv1=[2*q+6;-12*q+12;1];
    v2=[q^3 + 2*q;-2*q; -8*q];
    dv2=[3*q^2+2;-2;-8];
    
    dc=-skew(v2)*dv1 + skew(v1)*dv2;
    
    cv1v2=cross(v1,v2);
    grad_cross=(cv1v2-cv1v2_last)/step;
    NUM=[NUM;grad_cross'];
    ANALYTIC=[ANALYTIC;dc'];
end
for i=1:3
    plot(NUM(2:end,i),'r');
    hold on
    plot(ANALYTIC(2:end,i),'b');
end


%% Gradient of cross product of two columns of Jacobian matirx

clear all,clc,close all

cv1v2=zeros(3,1);
v1=zeros(3,1);
v2=zeros(3,1);
step=0.005;
NUM=[];
ANALYTIC=[];
qpos=randRange(-pi,pi,7);
joint=randi(7);
%joint=1;
T=T70(qpos);
J=J70(qpos);
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;
v1=JE(1:3,1);
v2=JE(1:3,2);
joint;
E=[];
step=0.01;
for qjoint=0:step:pi
    
    cv1v2_last=cv1v2;
    v1_last=v1;
    v2_last=v2;
    
    qpos(joint)=qjoint;
    T=T70(qpos);
    J=J70(qpos);
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
    v1=JE(1:3,1);
    v2=JE(1:3,4);
    H=getHessian(JE);
    dv1=H{joint}(1:3,1);
    dv2=H{joint}(1:3,4);
    
    dc=-skew(v2)*dv1 + skew(v1)*dv2;
    
    cv1v2=cross(v1,v2);
    grad_cross=(cv1v2-cv1v2_last)/step;
    NUM=[NUM;grad_cross'];
    ANALYTIC=[ANALYTIC;dc'];
end


for i=1:3
    plot(NUM(2:end,i),'r');
    hold on
    plot(ANALYTIC(2:end,i),'b');
end