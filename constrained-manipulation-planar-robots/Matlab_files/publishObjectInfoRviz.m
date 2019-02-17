function marker=publishObjectInfoRviz(ObjectPosition,frame,i,Q)
%publishObjectInfoRviz plot object in rviz
if(nargin==1)
   frame='torso'; 
   i=0;
elseif(nargin==2)
    i=0;
end



marker=rosmessage('visualization_msgs/Marker');
marker.Ns='Object';
marker.Action=marker.ADD;
marker.Type=marker.CUBE;
marker.Id=i;
marker.Pose.Position.X = ObjectPosition(1);
marker.Pose.Position.Y = ObjectPosition(2);
marker.Pose.Position.Z = ObjectPosition(3);
marker.Pose.Orientation.X = Q(1);
marker.Pose.Orientation.Y = Q(2);
marker.Pose.Orientation.Z = Q(3);
marker.Pose.Orientation.W = Q(4);
marker.Header.FrameId=frame;
marker.Lifetime = rosduration();
marker.Color.R=0.05;
marker.Color.G=0.05;
marker.Color.B=0.05;
marker.Color.A=1.0;
marker.Scale.X=0.05;
marker.Scale.Y=0.05;
marker.Scale.Z=0.01;



%
end

