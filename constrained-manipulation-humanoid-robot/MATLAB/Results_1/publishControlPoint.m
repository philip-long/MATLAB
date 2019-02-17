function publishControlPoint( cp_position,i,frame )
%publishControlPoint publish point in rviz
%   Detailed explanation goes here

if(nargin==1)
   frame='world'; 
   i=0;
elseif(nargin==2)
    frame='world'; 
end

pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
marker=rosmessage(pub);
marker.Ns='ControlPoint';
marker.Action=marker.ADD;
marker.Type=marker.SPHERE;
marker.Id=i;
marker.Pose.Position.X = cp_position(1);
marker.Pose.Position.Y = cp_position(2);
marker.Pose.Position.Z = cp_position(3);
marker.Pose.Orientation.X = 0.0;
marker.Pose.Orientation.Y = 0.0;
marker.Pose.Orientation.Z = 0.0;
marker.Pose.Orientation.W = 1.0;
marker.Header.FrameId=frame;
marker.Lifetime = rosduration(0.0);
marker.Color.R=1.0;
marker.Color.G=0.4;
marker.Color.B=0.0;

marker.Color.A=0.8;
marker.Scale.X=0.05;
marker.Scale.Y=0.05;
marker.Scale.Z=0.05;

%pub = rospublisher('/visualization_marker4','sensor_msgs/JointState');
send(pub,marker) % oublish the marker
while(pub.NumSubscribers<1)
    pause(0.1)
end

end
