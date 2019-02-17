%% INITIALIZE DATA
close all,clear all,clc
rosshutdown
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/UtilityFunctions'))
addpath(genpath('/home/philip/tbxmanager'))

 try
     joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
 catch
     rosinit
     joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
 end

%% EXTRACT DATE FROM *.mat file

load('wpstar_t10obj_1_05-Feb-2019')
%load('wp_t10obj_1_05-Feb-2019')
%load('wp_t2obj_1_05-Feb-2019')

% Initial
init_T_op=constructOpFrame(tinit);
init_objects=getObjectsLeftFootFrame(init_T_op);
init_Pd=init_T_op*[op_P_d;1]; % get pose in left foot frame
%

color_valu=zeros(length(FVAL),1);
valid_costs=zeros(length(find(EXITFLAG==1)),1);
valid_dists=zeros(length(find(EXITFLAG==1)),1);
k=1;
for i=1:length(FVAL)
    if(EXITFLAG(i)==1)
        color_valu(i)=COST(i);
        valid_costs(k)=COST(i);
        valid_dists(k)=DIST(i);
        k=k+1;
    else
        color_valu(i)=-1;
    end
end

mean(WPSTAR)
mean(COST)


mean(DIST)
mean(valid_costs)
mean(valid_dists)
length(find(EXITFLAG==1))
%% Show configuration in RVIZ

lf_T_op=constructOpFrame(x_lf);
lf_Pd=lf_T_op*[op_P_d;1]; % get pose in left foot frame
lf_objects=getObjectsLeftFootFrame(lf_T_op);
lf_Desired_Vertices=Polyhedron(transformMesh(lf_T_op,Desired_Vertices.V));
showObjects(lf_objects);
publishControlPoint(lf_Pd,1,'leftFoot');

while(1)
    mkr_publisher = rospublisher('/visualization_marker','visualization_msgs/Marker');
    mkr_msg=getMeshRviz(lf_Desired_Vertices,'leftFoot',[0.0,0.0,0.5,0.1]);
    send(mkr_publisher,mkr_msg);
    
    
    
    showObjects(lf_objects);
    joint_publisher= rospublisher('/dummy_joint_states','sensor_msgs/JointState');
    joint_msg=rosmessage(joint_publisher);
    joint_msg.Position=x_joints;
    for i=1:length(qnames)
        if(contains(qnames(i),'Neck') || contains(qnames(i),'neck') )
            joint_msg.Position(i)=0.0;
        end
    end    
    joint_msg.Name=qnames;
    joint_msg.Velocity=zeros(size(x_joints));
    joint_msg.Effort=zeros(size(x_joints));
    joint_msg.Header.Seq=1;
    joint_msg.Header.Stamp=rostime('now');
    joint_msg.Header.FrameId='world';
    send(joint_publisher,joint_msg);    
    publishControlPoint(lf_Pd,1,'leftFoot');
    pause(1.0)
end

for j=1:length(Q_CONFIG)
        joint_publisher= rospublisher('/dummy_joint_states','sensor_msgs/JointState');
    joint_msg=rosmessage(joint_publisher);
    joint_msg.Position=Q_CONFIG{j};
    for i=1:length(qnames)
        if(contains(qnames(i),'Neck') || contains(qnames(i),'neck') )
            joint_msg.Position(i)=0.0;
        end
    end    
    joint_msg.Name=qnames;
    joint_msg.Velocity=zeros(size(x_joints));
    joint_msg.Effort=zeros(size(x_joints));
    joint_msg.Header.Seq=1;
    joint_msg.Header.Stamp=rostime('now');
    joint_msg.Header.FrameId='world';
    send(joint_publisher,joint_msg);    
    publishControlPoint(lf_Pd,1,'leftFoot');
    pause()
end




%% Plot Workspace Result
rgb = vals2colormap(color_valu, 'jet',[-1 75])
for i=1:length(FVAL)

 plot3(POS(i,1),POS(i,2),POS(i,3),'s','MarkerFaceColor',rgb(i,:))
 publishWkPoint(POS(i,:),i,'leftFoot',rgb(i,:));
 hold on
end
xlabel('x')
ylabel('y')
zlabel('z')

%%