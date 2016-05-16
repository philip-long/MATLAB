function [ Repdot ] = Omega_to_ZXZdot( u )
%Converts angualr Velocity to delta Euler angle

omega=u(1:3);
R=u(4:12);
R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end

Y=Rot_to_ZXZ(Rinit);
phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

C=[-sin(phi1)*cot(theta1) ,  cos(phi1)*cot(theta1) ,     1
        cos(phi1)             ,  sin(phi1), 0
        sin(phi1)/sin(theta1) ,  -cos(phi1)/sin(theta1)  ,    0];


Repdot=(C*omega);