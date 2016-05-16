
clear all,close all, clc
Avg=load('MovingAverageForce');
Fr=load('ResolvedForce');
Fm=load('MeasuredForce');


figure(1)
plot(Fr(:,1),'r');
hold on
plot(Fr(:,2),'b');
plot(Fr(:,3),'g');
legend("fx","fy","fz");

figure(2)
plot(Fr(:,4),'r');
hold on
plot(Fr(:,5),'b');
plot(Fr(:,6),'g');
legend("nx","ny","nz");

