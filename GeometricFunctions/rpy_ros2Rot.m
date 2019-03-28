function [R] = rpy_ros2Rot(rpy)
%rpy_ros2Rot Convert ROS rpy to rotation matrix


x=[1;0;0];
y=[0;1;0];
z=[0;0;1];
R=expm(skew(z)*rpy(3)) * expm(skew(y)*rpy(2))*expm(skew(x)*rpy(1));

end

