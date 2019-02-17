function [ output_args ] = showObjects( objects)
%showObjects show objects in rviz and in Matlab
% objects is an array of Polyhedra

objects{1}.plot('alpha',0.1)
hold on
objects{2}.plot('alpha',0.1)
objects{3}.plot('alpha',0.1)
objects{4}.plot('alpha',0.1)

mkr_msg=getMeshRviz(objects{1},'leftFoot',[0.2,0.2,0.2]);
mkr_publisher = rospublisher('/visualization_marker','visualization_msgs/Marker');
while(mkr_publisher.NumSubscribers<1)
    pause(0.1)
end
send(mkr_publisher,mkr_msg);
mkr_msg=getMeshRviz(objects{2},'leftFoot',[0.1,0.5,0.1]);
send(mkr_publisher,mkr_msg);
pause(0.05)
mkr_msg=getMeshRviz(objects{3},'leftFoot',[0.1,0.5,0.8]);
send(mkr_publisher,mkr_msg);
pause(0.05)
mkr_msg=getMeshRviz(objects{4},'leftFoot',[0.8,0.2,0.3]);
send(mkr_publisher,mkr_msg);
pause(0.05)
end

