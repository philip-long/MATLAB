function publishControlPoint( cp_position,i,frame )
%publishControlPoint publish point in RVIZ
%  

if(nargin==1)
   frame='world'; 
   i=0;
elseif(nargin==2)
    frame='world'; 
end


marker=rosmessage('visualization_msgs/Marker');
marker.Ns='Object';
marker.Action=marker.ADD;
marker.Type=marker.SPHERE;
marker.Id=i;
marker.Pose.Position.X = cp_position(1);
marker.Pose.Position.Y = cp_position(2);
marker.Pose.Position.Z = 0.05;
marker.Pose.Orientation.X = 0.0;
marker.Pose.Orientation.Y = 0.0;
marker.Pose.Orientation.Z = 0.0;
marker.Pose.Orientation.W = 1.0;
marker.Header.FrameId=frame;
marker.Lifetime = rosduration(5.5);
marker.Color.R=1.0;
marker.Color.G=0.4;
marker.Color.B=0.0;

marker.Color.A=0.8;
marker.Scale.X=0.05;
marker.Scale.Y=0.05;
marker.Scale.Z=0.01;
pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
send(pub,marker) % oublish the marker

end

