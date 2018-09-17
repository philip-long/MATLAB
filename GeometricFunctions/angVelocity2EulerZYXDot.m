function [ Repdot ] = angVelocity2EulerZXZDot(omega,R)
% Omega_to_AngleAxisdot Convert angular velocity to change in euler angles (Z Y X)

R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end

Y=zyx_euler2Rot(R);

phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

C=[cos(phi1)*tan(theta1) ,   sin(phi1)*tan(theta1), 1
    -sin(phi1)           ,   cos(phi1)      ,       0
    cos(phi1)/cos(theta1),sin(phi1)/cos(theta1),0];

Repdot=C*omega;
