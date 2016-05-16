function [ Repdot ] = Omega_to_ZYXdot( u )
%Converts angualr Velocity to delta Euler angle

omega=u(1:3);
R=u(4:12);
R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end

Y=Rot_to_ZYX(Rinit);

phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

C=[cos(phi1)*tan(theta1) ,   sin(phi1)*tan(theta1), 1
    -sin(phi1)           ,   cos(phi1)      ,       0
    cos(phi1)/cos(theta1),sin(phi1)/cos(theta1),0];

Repdot=C*omega;