clear all,clc,close all 
filename='wsB__2Objects2';
load(filename)
%%


%DATA_1=[DATA_1;Xdesired,Ydesired,q_pos,q_vol,q_vold];   
% DATA_1=[DATA_1;Xdesired,Ydesired,q_pos,q_vol,q_vold,vols_saves]; 
config=1;
if(config==1)
    DATA=DATA_1;
else
    DATA=DATA_2;
end


Xdesired=DATA(:,1);
Ydesired=DATA(:,2);
maxx=max(Xdesired);
minx=min(Xdesired);
step_si=Ydesired(10)-Ydesired(9);
total_len=length(minx:step_si:maxx)
QRATIO=DATA(:,6)./DATA(:,5);
temp=(QRATIO==-1);
QRATIO2=QRATIO;
QRATIO2(temp)=NaN;
VOL=DATA(:,8);
temp=(QRATIO==-1);
VOL(temp)=NaN;
% tri = delaunay(Xdesired,Ydesired); %x,y,z column vectors
% trisurf(tri,Xdesired,Ydesired,QRATIO); 
% xlabel('X'); ylabel('Y'); zlabel('Q');

%% Qratio
x = reshape(Xdesired,total_len,total_len)';
y = reshape(Ydesired,total_len,total_len)';
z = reshape(QRATIO2,total_len,total_len)';

colormap jet
s=surf(x,y,z)
s.EdgeColor = 'none';
%alpha(s,'z')
hold on
plot3(ObjectPositions(1,1),...
    ObjectPositions(1,2),...
    2.0,...
    's',...
'MarkerSize',15,...
'MarkerEdgeColor','k',...
'MarkerFaceColor',[1.0,1.0,1.0])

plot3(ObjectPositions(2,1),...
    ObjectPositions(2,2),...
    2.0,...
    's',...
'MarkerSize',15,...
'MarkerEdgeColor','k',...
'MarkerFaceColor',[1.0,1.0,1.0])

%%
Qnew=IGM(0.924,0.456);
if(config==1)
   if( Qnew(1,2)<0)
    q= Qnew(1,:);
   else
    q= Qnew(2,:);
   end
else    
   if( Qnew(1,2)>0)
    q= Qnew(1,:);
   else
    q= Qnew(2,:);
   end
end

T=Transforms_world(q); % Get all transforms
% set saftey threshold
TnE=eye(4); TnE(1,4)=0.55;   % terminal point
T0E=T{2}*TnE;

plot3([0.0,T{1}(1,4)],[0.0,T{1}(2,4)],[1.2,1.2],'k-o','LineWidth',8.0);
hold on
plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[1.2,1.2],'k-o','LineWidth',5.0,'MarkerSize',5.0,'MarkerFaceColor', [0.5 0.5 0.5],'MarkerEdgeColor', [0.5 0.5 0.5]);
plot3([T{2}(1,4),T0E(1,4)],[T{2}(2,4),T0E(2,4)],[1.2,1.2],'k-o','LineWidth',5.0,'MarkerSize',5.0,'MarkerFaceColor', [0.5 0.5 0.5],'MarkerEdgeColor', [0.5 0.5 0.5]);
plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[1.3,1.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);
plot3([T{2}(1,4),T0E(1,4)],[T{2}(2,4),T0E(2,4)],[1.3,1.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);

xlabel(' x [m]','FontSize',20)
ylabel(' y [m]','FontSize',20)
xt = get(gca, 'XTick');
set(gca, 'FontSize', 16)
axis([0 1.35 0 1.35])
title('\eta_{q}','FontSize',20)

%%
%% Vol
x = reshape(Xdesired,total_len,total_len)';
y = reshape(Ydesired,total_len,total_len)';
z = reshape(VOL,total_len,total_len)';

colormap jet
s=surf(x,y,z)
s.EdgeColor = 'none';
%alpha(s,'z')
hold on
plot3(ObjectPositions(1,1),...
    ObjectPositions(1,2),...
    2.0,...
    's',...
'MarkerSize',15,...
'MarkerEdgeColor','k',...
'MarkerFaceColor',[1.0,1.0,1.0])

plot3(ObjectPositions(2,1),...
    ObjectPositions(2,2),...
    2.0,...
    's',...
'MarkerSize',15,...
'MarkerEdgeColor','k',...
'MarkerFaceColor',[1.0,1.0,1.0])
%%
Qnew=IGM(0.924,0.456);
if(config==1)
   if( Qnew(1,2)<0)
    q= Qnew(1,:);
   else
    q= Qnew(2,:);
   end
else    
   if( Qnew(1,2)>0)
    q= Qnew(1,:);
   else
    q= Qnew(2,:);
   end
end

T=Transforms_world(q); % Get all transforms
% set saftey threshold
TnE=eye(4); TnE(1,4)=0.55;   % terminal point
T0E=T{2}*TnE;

plot3([0.0,T{1}(1,4)],[0.0,T{1}(2,4)],[2.2,2.2],'k-d','LineWidth',8.0);
hold on
plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[2.2,2.2],'k-o','LineWidth',5.0,'MarkerSize',5.0,'MarkerFaceColor', [0.5 0.5 0.5],'MarkerEdgeColor', [0.5 0.5 0.5]);
plot3([T{2}(1,4),T0E(1,4)],[T{2}(2,4),T0E(2,4)],[2.2,2.2],'k-o','LineWidth',5.0,'MarkerSize',5.0,'MarkerFaceColor', [0.5 0.5 0.5],'MarkerEdgeColor', [0.5 0.5 0.5]);
plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[2.3,2.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);
plot3([T{2}(1,4),T0E(1,4)],[T{2}(2,4),T0E(2,4)],[2.3,2.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);

xlabel(' x [m]','FontSize',20)
ylabel(' y [m]','FontSize',20)
xt = get(gca, 'XTick');
set(gca, 'FontSize', 16)
axis([0 1.35 0 1.35])
title('\nu_{q}','FontSize',20)