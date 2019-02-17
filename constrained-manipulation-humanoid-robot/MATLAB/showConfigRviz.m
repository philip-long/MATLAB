function  showConfigRviz( x_joints,qnames)
%showSolutioninRviz Plot solution in rviz
%   Show robot current configuration and the object locations in rviz
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
while(joint_publisher.NumSubscribers<1)
    pause(.1)
end
send(joint_publisher,joint_msg);


end

