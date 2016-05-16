function showS()
clear all,close all,clc
data = load('bMt');
Fr = load('ResolvedForce');


Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);
%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title('P');

hold on;
plot(Px,'r');
hold on;
plot(Py,'b');
hold on;
plot(Pz,'g');
hold on;
plot(Fr(:,2),'k--')


%ylim([-500,500])

legend('Px','Py','Pz')
