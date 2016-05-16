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

plot(Curvedata(7757:8544,1),Curvedata(7757:8544,3),'r','Linewidth',1.5)
 hold on
R=Prnge{1};
P=polyfit(Pux(R),Puy(R),2)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'b--','Linewidth',3.0)

R=Prnge{5};
P=polyfit(Pux(R),Puy(R),2)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'m-.','Linewidth',3.0)


R=Prnge{11};
P=polyfit(Pux(R),Puy(R),2)
plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'k:','Linewidth',3.0)

title('Cutting Trajectory Deformation')
legend('Offline Estimation','Passage 1','Passage 5','Passage 11')
xlabel('X(m)')
ylabel('Y(m)')