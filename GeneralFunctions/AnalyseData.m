% Script to analyse the data for the experiment

clear all, clc
% First things first decide where the cuts start
% and finish last datapoint of each cut 
LastDataPoint=[1,382,819,1304,1833,2379,2954,3562,4206,4865,5551,6259,7008,7756,8544];
for i=(1:length(LastDataPoint)-1)
    Prnge{i}=LastDataPoint(i):LastDataPoint(i+1);
end
%% Curve one showing deformation and robot position

Curvedata = load('Curve');
Uv=load('Uv'); % Transformation matrix as detected from vision
data = load('bMt');
Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);

Pux=Uv(1:4:end,4);
Puy=Uv(2:4:end,4);

plot(Curvedata(7757:8544,1),Curvedata(7757:8544,3),'b','Linewidth',2.0)

for i=1:14
    R=Prnge{i};
    plot(Pux(R),Puy(R),'r*')
     hold on
    P=polyfit(Pux(R),Puy(R),2)
    plot(min(Pux(R)):0.005:max(Pux(R)),polyval(P,min(Pux(R)):0.005:max(Pux(R))),'g')
    plot(Px(R),Py(R),'b>')
    pause()
    hold off
end
%%

%
R=Prnge{1};
plot(Pux(R),Puy(R),'r*')
hold on
R=Prnge{2};
plot(Pux(R),Puy(R),'g*')
R=Prnge{3};
plot(Pux(R),Puy(R),'b*')
R=Prnge{4};
plot(Pux(R),Puy(R),'c*')
R=Prnge{5};
plot(Pux(R),Puy(R),'y*')
R=Prnge{6};
plot(Pux(R),Puy(R),'m*')
R=Prnge{7};
plot(Pux(R),Puy(R),'k*')
R=Prnge{8};
plot(Pux(R),Puy(R),'rx')
R=Prnge{9};
plot(Pux(R),Puy(R),'gx')
R=Prnge{10};
plot(Pux(R),Puy(R),'bx')
R=Prnge{11};
plot(Pux(R),Puy(R),'cx')
R=Prnge{12};
plot(Pux(R),Puy(R),'yx')
R=Prnge{13};
plot(Pux(R),Puy(R),'mx')
R=Prnge{14};
plot(Pux(R),Puy(R),'b*')

%%
%
R=Prnge{1};
plot(Px(R),Py(R),'r*')
hold on
R=Prnge{2};
plot(Px(R),Py(R),'g*')
R=Prnge{3};
plot(Px(R),Py(R),'b*')
R=Prnge{4};
plot(Px(R),Py(R),'c*')
R=Prnge{5};
plot(Px(R),Py(R),'y*')
R=Prnge{6};
plot(Px(R),Py(R),'m*')
R=Prnge{7};
plot(Px(R),Py(R),'k*')
R=Prnge{8};
plot(Px(R),Py(R),'rx')
R=Prnge{9};
plot(Px(R),Py(R),'gx')
R=Prnge{10};
plot(Px(R),Py(R),'bx')
R=Prnge{11};
plot(Px(R),Py(R),'cx')
R=Prnge{12};
plot(Px(R),Py(R),'yx')
R=Prnge{13};
plot(Px(R),Py(R),'mx')
R=Prnge{14};
plot(Px(R),Py(R),'b*')

%plot(Pux(Prnge{8}),Puy(Prnge{8}),'k*')