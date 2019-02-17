function publishWkPoint( cp_position,i,frame,color_rgb )
%publishWkPoint publish a cube in rviz
%   Detailed explanation goes here

if(nargin==1)
   frame='world'; 
   i=0;
elseif(nargin==2)
    frame='world'; 
end


marker=rosmessage('visualization_msgs/Marker');
marker.Ns='WorkspacePoint';
marker.Action=marker.ADD;
marker.Type=marker.CUBE;
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
marker.Color.R=color_rgb(1);
marker.Color.G=color_rgb(2);
marker.Color.B=color_rgb(3);

marker.Color.A=0.5;
marker.Scale.X=0.04;
marker.Scale.Y=0.04;
marker.Scale.Z=0.04;
pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
send(pub,marker) % oublish the marker
while(pub.NumSubscribers<1)
    pause(0.1)
end
end
