% gradient of cross product test
clear all, clc,close all;
sig=0
step=0.01;
D=[];
N=[];
S=[]
a=100
for z=-5:0.01:5
    siglast=sig;
    sig=sigmoid(z,a);
    grad_sig=sigmoidGradient(z,a);
    D=[D;grad_sig];
    num_grad=(sig-siglast)/step;
    N=[N;num_grad];
    S=[S;sig]
end
z=-5:0.01:5;
plot(z(2:end),N(2:end),'r')
hold on
plot(z(2:end),D(2:end),'b')
%plot(z(2:end),S(2:end),'g')