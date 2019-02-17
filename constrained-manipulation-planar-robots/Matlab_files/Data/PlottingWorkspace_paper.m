
%% ELBOW DOWN
clear all,clc,close all
global ObjectPositions wkspace_lims_up wkspace_lims_dwn;
ObjectPositions=[];
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
ObjectPositions(3,:)=[0.8;-0.45;0.0]; % in world frame adding a third object
wkspace_lims_up=[0.216,-1.332;-0.396,0.432];  %wkspace_lims_up=[-0.54,0.828;0.198,-1.332];
wkspace_lims_dwn=[-0.396,-0.432;0.198,1.314]; %wkspace_lims_dwn=[0.198,1.314;-0.54, -0.828];
filename='WK_ext';
load(filename)

eta_type=1; wp_type=2;wp_star_type=3;c_ext_type=4;
elbow_down_config=1; elbow_up_config=2;

DATA=ELBOW_DOWN;
Xdesired=DATA(:,1);
Ydesired=DATA(:,2);
maxx=max(Xdesired);
minx=min(Xdesired);
step_si=Ydesired(10)-Ydesired(9);
total_len=length(minx:step_si:maxx)
eta_obst=DATA(:,6)./DATA(:,5);
eta_jl=DATA(:,7)./DATA(:,5);
eta_obs_jl=DATA(:,8)./DATA(:,5);
wp=DATA(:,9);
wp_obst=DATA(:,10);
wp_jl=DATA(:,11);
wp_obs_jl=DATA(:,12);
c_ext=DATA(:,13);


%% Plotting
figure(1)
plotSurface(Xdesired,Ydesired,eta_obs_jl,total_len,eta_type,elbow_down_config)
figure(2)
plotSurface(Xdesired,Ydesired,wp,total_len,wp_type,elbow_down_config)
figure(3)
plotSurface(Xdesired,Ydesired,wp_obs_jl,total_len,wp_star_type,elbow_down_config)
figure(4)
plotSurface(Xdesired,Ydesired,c_ext,total_len,c_ext_type,elbow_down_config)


%% ELBOW UP

clear all,clc,close all
global ObjectPositions wkspace_lims_up wkspace_lims_dwn;
ObjectPositions=[];
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame
ObjectPositions(3,:)=[0.8;-0.45;0.0]; % in world frame adding a third object
wkspace_lims_up=[0.216,-1.332;-0.396,0.432];  %wkspace_lims_up=[-0.54,0.828;0.198,-1.332];
wkspace_lims_dwn=[-0.396,-0.432;0.198,1.314]; %wkspace_lims_dwn=[0.198,1.314;-0.54, -0.828];
filename='WK_ext';
load(filename)

eta_type=1; wp_type=2;wp_star_type=3;c_ext_type=4;
elbow_down_config=1; elbow_up_config=2;

DATA=ELBOW_UP;
Xdesired=DATA(:,1);
Ydesired=DATA(:,2);
maxx=max(Xdesired);
minx=min(Xdesired);
step_si=Ydesired(10)-Ydesired(9);
total_len=length(minx:step_si:maxx)
eta_obst=DATA(:,6)./DATA(:,5);
eta_jl=DATA(:,7)./DATA(:,5);
eta_obs_jl=DATA(:,8)./DATA(:,5);
wp=DATA(:,9);
wp_obst=DATA(:,10);
wp_jl=DATA(:,11);
wp_obs_jl=DATA(:,12);
c_ext=DATA(:,13);

%% Plotting
figure(1)
plotSurface(Xdesired,Ydesired,eta_obs_jl,total_len,eta_type,elbow_up_config)
figure(2)
plotSurface(Xdesired,Ydesired,wp,total_len,wp_type,elbow_up_config)
figure(3)
plotSurface(Xdesired,Ydesired,wp_obs_jl,total_len,wp_star_type,elbow_up_config)
figure(4)
plotSurface(Xdesired,Ydesired,c_ext,total_len,c_ext_type,elbow_up_config)

%% REACHABILITY TESTS
clear all,clc,close all
filename='WK_ext';
load(filename)
%%
%                 ELBOW_DOWN=[ELBOW_DOWN; ...
%                     Xdesired,Ydesired,...
%                     q_pos,...
%                     Q_arm.volume,...
%                     Q_arm_obs.volume,...
%                     Q_arm_jl.volume,...
%                     Q_arm_obs_jl.volume,...
%                     V_arm.volume,...
%                     V_arm_obs.volume,...
%                     V_arm_jl.volume,...
%                     V_arm_obs_jl.volume];


eta_type=1; wp_type=2;wp_star_type=3;c_ext_type=4;
elbow_down_config=1; elbow_up_config=2;
DATA=ELBOW_DOWN;
Xdesired=DATA(:,1);
Ydesired=DATA(:,2);
maxx=max(Xdesired);
minx=min(Xdesired);
step_si=Ydesired(10)-Ydesired(9);
total_len=length(minx:step_si:maxx)
eta_obst=DATA(:,6)./DATA(:,5);
eta_jl=DATA(:,7)./DATA(:,5);
eta_obs_jl=DATA(:,8)./DATA(:,5);
wp=DATA(:,9);
wp_obst=DATA(:,10);
wp_jl=DATA(:,11);
wp_obs_jl=DATA(:,12);
c_ext=DATA(:,13);

DATA=ELBOW_UP;
Xdesired_up=DATA(:,1);
Ydesired_up=DATA(:,2);
maxx=max(Xdesired_up);
minx=min(Xdesired_up);
step_si=Ydesired(10)-Ydesired(9);
total_len_up=length(minx:step_si:maxx);
eta_obst_up=DATA(:,6)./DATA(:,5);
eta_jl_up=DATA(:,7)./DATA(:,5);
eta_obs_jl_up=DATA(:,8)./DATA(:,5);
wp_up=DATA(:,9);
wp_obst_up=DATA(:,10);
wp_jl_up=DATA(:,11);
wp_obs_jl_up=DATA(:,12);
c_ext_up=DATA(:,13);

figure(1)
plotSurface(Xdesired,Ydesired,max(eta_obs_jl,eta_obs_jl_up),total_len,eta_type,0)
figure(2)
plotSurface(Xdesired,Ydesired,max(wp,wp_up),total_len,wp_type,0)
figure(3)
plotSurface(Xdesired,Ydesired,max(wp_obs_jl_up,wp_obs_jl),total_len,wp_star_type,0)
figure(4)
plotSurface(Xdesired,Ydesired,max(c_ext,c_ext_up),total_len,c_ext_type,0)


%% Optimal work palcement pose


clear all,clc,close all
filename='REACHABILITY';
load(filename)

eta_type=1; wp_type=2;wp_star_type=3;c_ext_type=4;
elbow_down_config=1; elbow_up_config=2;
DATA=ELBOW_DOWN;
Xdesired=DATA(:,1);
Ydesired=DATA(:,2);
maxx=max(Xdesired);
minx=min(Xdesired);
step_si=Ydesired(10)-Ydesired(9);
total_len=length(minx:step_si:maxx)
eta_obst=DATA(:,6)./DATA(:,5);
eta_jl=DATA(:,7)./DATA(:,5);
eta_obs_jl=DATA(:,8)./DATA(:,5);
wp=DATA(:,9);
wp_obst=DATA(:,10);
wp_jl=DATA(:,11);
wp_obs_jl=DATA(:,12);


DATA=ELBOW_UP;
Xdesired_up=DATA(:,1);
Ydesired_up=DATA(:,2);
maxx=max(Xdesired_up);
minx=min(Xdesired_up);
step_si=Ydesired(10)-Ydesired(9);
total_len_up=length(minx:step_si:maxx);
eta_obst_up=DATA(:,6)./DATA(:,5);
eta_jl_up=DATA(:,7)./DATA(:,5);
eta_obs_jl_up=DATA(:,8)./DATA(:,5);
wp_up=DATA(:,9);
wp_obst_up=DATA(:,10);
wp_jl_up=DATA(:,11);
wp_obs_jl_up=DATA(:,12);

figure(1)
plotSurface(Xdesired,Ydesired,nanmax(eta_obs_jl,eta_obs_jl_up),total_len,eta_type,0)
figure(2)
plotSurface(Xdesired,Ydesired,nanmax(wp,wp_up),total_len,wp_type,0)
figure(3)
plotSurface(Xdesired,Ydesired,nanmax(wp_obs_jl_up,wp_obs_jl),total_len,wp_star_type,0)

figure(4)
plotSurface(Xdesired,Ydesired,wp_obs_jl,total_len,wp_star_type,0)
figure(5)
plotSurface(Xdesired,Ydesired,wp_obs_jl_up,total_len,wp_star_type,0)


qmb=[1.0458   -0.1944];
Q_configs=getIGM(0.18-qmb(2),0.036-qmb(1));
qpos=Q_configs(1,:);
T=getTransforms(qpos); % Get all transforms
% set saftey threshold
TnE=eye(4); TnE(1,4)=0.55;   % terminal point
T{end+1}=T{2}*TnE;
for i=1:length(T)
    T{i}(1,4)=T{i}(1,4)+qmb(2);
    T{i}(2,4)=T{i}(2,4)+qmb(1);
end
plotPlanarRobot(T)

plot3([0 0],[0 1],[2 2],'k:')
plot3([0 1],[1 1],[2 2],'k:')
plot3([1 1],[1 0],[2 2],'k:')
plot3([1 0],[0 0],[2 2],'k:')

% plot unit square