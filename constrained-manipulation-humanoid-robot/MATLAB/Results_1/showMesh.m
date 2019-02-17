function [ output_args ] = showMesh( rviz_mesh)
%showMesh show mesh in rviz 

mkr_publisher = rospublisher('/visualization_marker','visualization_msgs/Marker');
while(mkr_publisher.NumSubscribers<1)
    pause(0.1)
end
send(mkr_publisher,rviz_mesh);
pause(0.05)

end

