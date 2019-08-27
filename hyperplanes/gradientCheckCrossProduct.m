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