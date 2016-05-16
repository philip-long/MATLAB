
clear all,close all, clc
Avg=load('MovingAverageForce');
Fr=load('ResolvedForce');
Fm=load('MeasuredForce');


for i=1:6
figure(i)
plot(Fr(:,i),'r');
hold on 
plot(Avg(:,i),'b');
plot(Fm(:,i)/1000000,'y');
legend('Resolved','Averaged','Measured');
end
