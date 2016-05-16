% First plot for the graph is showing how the dots
% have deformed due to the pulling force and the
% weakening of the bonds
%% Loading data
close all,clear all, clc
% First things first decide where the cuts start
% and finish last datapoint of each cut 
LastDataPoint=[1,382,819,1304,1833,2379,2954,3562,4206,4865,5551,6259,7008,7756,8544];
for i=(1:length(LastDataPoint)-1)
    Prnge{i}=LastDataPoint(i):LastDataPoint(i+1);
end

Curvedata = load('Curve');
Uv=load('Uv'); % Transformation matrix as detected from vision

Pux=Uv(1:4:end,4);
Puy=Uv(2:4:end,4);
data = load('bMt');
Px=data(1:4:end,4);
Py=data(2:4:end,4);

%% 3 subplots compaing robot to vision system
R=Prnge{1};
P=polyfit(Pux(R),Puy(R),2)
subplot(1,3,1)
title('Passage 1')
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b','Linewidth',2.0)
xlim([-0.65 -0.56])
ylim([0.24 0.278])
xlabel('X(m')
ylabel('Y(m')
title('Passage 1')
 hold on
plot(Px(R),Py(R),'r*','MarkerSize',1.0)


R=Prnge{5};
P=polyfit(Pux(R),Puy(R),2)
subplot(1,3,2)

plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b','Linewidth',2.0)
xlim([-0.65 -0.56])
ylim([0.24 0.278])
xlabel('X(m)')
ylabel('Y(m')
title('Passage 5')
 hold on
plot(Px(R),Py(R),'r*','MarkerSize',1.0)

R=Prnge{11};
P=polyfit(Pux(R),Puy(R),2)
subplot(1,3,3)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b','Linewidth',2.0)
xlim([-0.65 -0.56])
ylim([0.24 0.278])
xlabel('X(m)')
ylabel('Y(m)')
title('Passage 11')
 hold on
plot(Px(R),Py(R),'r*','MarkerSize',1.0)
legend('Vision Extraction','Robot position')
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0
1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Robot trajectory following','HorizontalAlignment','center','VerticalAlignment', 'top')

%%

R=Prnge{9};
P=polyfit(Pux(R),Puy(R),2)
subplot(3,2,4)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b','Linewidth',2.0)
 hold on
plot(Px(R),Py(R),'b*','MarkerSize',1.0)

R=Prnge{11};
P=polyfit(Pux(R),Puy(R),2)
subplot(3,2,5)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b','Linewidth',2.0)
 hold on
plot(Px(R),Py(R),'b*','MarkerSize',1.0)

%%
R=Prnge{5};
P=polyfit(Pux(R),Puy(R),2)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'m-.','Linewidth',1.0)
plot(Px(R),Py(R),'ms','MarkerSize',2.0)

R=Prnge{11};
P=polyfit(Pux(R),Puy(R),2)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'k:','Linewidth',1.0)
plot(Px(R),Py(R),'kx','MarkerSize',1.0)

title('Robot Trajectory')
legend('Passage 1','Robot Position Passage 1','Passage 5','Robot Position Passage 5','Passage 14','Robot Position Passage 14')
xlabel('X(m)')
ylabel('Y(m)')