function plotSurface(Xdesired,Ydesired,Z,total_len,type,elbow_config)

%% Qratio
global ObjectPositions wkspace_lims_dwn wkspace_lims_up
% ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
% ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame

x = reshape(Xdesired,total_len+1,total_len+1)';
y = reshape(Ydesired,total_len+1,total_len+1)';
%z = reshape(eta_obs_jl,total_len+1,total_len+1)';
z = reshape(Z,total_len+1,total_len+1)';
colormap jet
s=surf(x,y,z)
s.EdgeColor = 'none';
%alpha(s,'z')
hold on
for obj=1:size(ObjectPositions,1)
plot3(ObjectPositions(obj,1),...
    ObjectPositions(obj,2),...
    2.0,...
    's',...
    'MarkerSize',15,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[1.0,1.0,1.0])
% 
% plot3(ObjectPositions(2,1),...
%     ObjectPositions(2,2),...
%     2.0,...
%     's',...
%     'MarkerSize',15,...
%     'MarkerEdgeColor','k',...
%     'MarkerFaceColor',[1.0,1.0,1.0])
end

set(gca, 'FontSize', 16)
xlim([-1 1.35])
ylim([ -1.35 1.35])

if type==1 % eta
    title('\eta','FontSize',20)
    xlabel(' x [m]','FontSize',20)
    ylabel(' y [m]','FontSize',20)
elseif type==2
    xlabel(' x [m]','FontSize',20)
    ylabel(' y [m]','FontSize',20)
    title('w_p','FontSize',20)
elseif type==3
    xlabel(' x [m]','FontSize',20)
    ylabel(' y [m]','FontSize',20)
    title('w_{p}^{*}','FontSize',20)
elseif type==4
    xlabel(' x [m]','FontSize',20)
    ylabel(' y [m]','FontSize',20)
    title('c_{ext}','FontSize',20)    
end


if(elbow_config==1)
   % Qnew=getIGM(0.216,-1.332);
    Qnew=getIGM(wkspace_lims_up(1,1),wkspace_lims_up(1,2));
    % Elbow up is always second config
    qpos=Qnew(1,:);
    T=getTransforms(qpos); % Get all transforms
    % set saftey threshold
    TnE=eye(4); TnE(1,4)=0.55;   % terminal point
    T{end+1}=T{2}*TnE;
    plotPlanarRobot(T)
    
   % Qnew=getIGM(-0.396,0.432);
    Qnew=getIGM(wkspace_lims_up(2,1),wkspace_lims_up(2,2));
    qpos=Qnew(1,:);
    T=getTransforms(qpos); % Get all transforms
    % set saftey threshold
    TnE=eye(4); TnE(1,4)=0.55;   % terminal point
    T{end+1}=T{2}*TnE;
    plotPlanarRobot(T)
elseif(elbow_config==2)
  %  Qnew=getIGM(-0.396,-0.432);
    Qnew=getIGM(wkspace_lims_dwn(2,1),wkspace_lims_dwn(2,2));
    % Elbow up is always second config
    qpos=Qnew(2,:);
    T=getTransforms(qpos); % Get all transforms
    % set saftey threshold
    TnE=eye(4); TnE(1,4)=0.55;   % terminal point
    T{end+1}=T{2}*TnE;
    plotPlanarRobot(T)
    
 %   Qnew=getIGM(0.198,1.314);
    Qnew=getIGM(wkspace_lims_dwn(1,1),wkspace_lims_dwn(1,2));
    qpos=Qnew(2,:);
    T=getTransforms(qpos); % Get all transforms
    % set saftey threshold
    TnE=eye(4); TnE(1,4)=0.55;   % terminal point
    T{end+1}=T{2}*TnE;
    plotPlanarRobot(T)    
end

end