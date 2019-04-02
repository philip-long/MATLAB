function [ Repdot ] = angVelocity2EulerZXZDot(omega,R)
% Omega_to_AngleAxisdot Convert angular velocity to change in euler angles (Z X Z)

R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end

Y=zxz_euler2Rot(Rinit);
phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

C=[-sin(phi1)*cot(theta1) ,  cos(phi1)*cot(theta1) ,     1
        cos(phi1)             ,  sin(phi1), 0
        sin(phi1)/sin(theta1) ,  -cos(phi1)/sin(theta1)  ,    0];


Repdot=(C*omega);
