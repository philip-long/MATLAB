% Gradient of a sigmoid function check
clear all, clc,close all;
clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))

sig=0
step=0.001;
D=[];
N=[];
S=[]
a=100;
zmin=-5;
zmax=5
E=[]
for z=zmin:step:zmax
    siglast=sig;
    sig=sigmoid(z,a);
    grad_sig=sigmoidGradient(z,a);
    D=[D;grad_sig];
    num_grad=(sig-siglast)/step;
    N=[N;num_grad];
    S=[S;sig];
    E=[E;norm(num_grad- grad_sig)];    
end

plot(E)
% z=zmin:step:zmax;
% plot(z(2:end),N(2:end),'r');
% hold on
% plot(z(2:end),D(2:end),'b');
% 
% norm(N(3:end)-D(2:end-1))/length(N)
% A=[N,D]
%plot(z(2:end),S(2:end),'g')