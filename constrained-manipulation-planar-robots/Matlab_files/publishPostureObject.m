function marker=publishPostureObject(ObjectPosition,i)
%publishPostureObject plot object in rviz



marker=rosmessage('visualization_msgs/Marker');
marker.Ns='Object';
marker.Action=marker.ADD;
marker.Type=marker.SPHERE;
marker.Id=i;
marker.Pose.Position.X = ObjectPosition(1);
marker.Pose.Position.Y = ObjectPosition(2);
marker.Pose.Position.Z = ObjectPosition(3);
marker.Pose.Orientation.X = 0;
marker.Pose.Orientation.Y = 0;
marker.Pose.Orientation.Z = 0;
marker.Pose.Orientation.W = 1;
marker.Header.FrameId='world';
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

