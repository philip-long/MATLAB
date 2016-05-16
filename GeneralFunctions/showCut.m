function showCut()
clear all,close all,clc
data = load('CutData');

% 
%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1) 
title("Work Done");
hold on
plot(data(:,1),'r');
hold on;
legend('Work done by Cut Fc*Vc')

figure(4)
title("Work Done");
plot(data(:,2),'b');
hold on;
legend('Work done by Slice Fs*Vs');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ratio=data(:,1)./data(:,2);
figure(3)
plot(Ratio,'g');
hold on;
legend('Ratio Vc/Vs')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2)

plot(data(:,3),'g');
hold on;
plot(data(:,4),'y');
legend('Contact Width mm ','Crack Propogation mm')


figure(5)
title("Fracture Toughness");
plot(data(:,5),'r');
hold on;
legend('Fracture Toughness')

