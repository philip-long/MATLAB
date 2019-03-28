function [sol1] = rot2Euler_xyz(R)
%rot2Euler_xyz Useful for ROS
%   Detailed explanation goes here
% [                           cos(q2)*cos(q3),                          -cos(q2)*sin(q3),          sin(q2)]
% [ cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2), cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3), -cos(q2)*sin(q1)]
% [ sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2), cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3),  cos(q1)*cos(q2)]


phi=atan2(R(2,3),-R(3,3));
phi2=atan2(-R(2,3),R(3,3));

theta=atan2(R(1,3),-sin(phi)*R(2,3) + cos(phi)*R(3,3));
theta2=atan2(R(1,3),-sin(phi2)*R(2,3) + cos(phi2)*R(3,3));


psi=atan2( cos(phi)*R(2,1)+ sin(phi)* R(3,1), cos(phi)*R(2,2)+sin(phi)*R(3,2) );
psi2=atan2( cos(phi2)*R(2,1)+ sin(phi2)* R(3,1), cos(phi2)*R(2,2)+sin(phi2)*R(3,2) );

sol1=[phi;theta;psi];
sol2=[phi2;theta2;psi2]
end

