function showS()
clear all,close all,clc
data = load('bMt');

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
%ylim([-500,500])

legend('Px','Py','Pz')
%%
for i=2:length(Px)
    plot(i,Px(i)-Px(i-1),'*')
    hold on
end
for i=2:length(Px)
    plot(i,Pz(i)-Pz(i-1),'*')
    hold on
end