function [ Repdot ] = angularVelocity2Quaterniondot(omega,R)
% angularVelocity2Quaterniondot Converts angular Velocity to change in quaternion 

omega=u(1:3);
Test=norm((R*R')-eye(3))
if Test>0.1
    disp 'Not a rotation matrix'
end

[Q1 Q2 Q3 Q4]=matrix2quaternion(Rinit); 
Q=[Q1; Q2; Q3; Q4];

 C=0.5*[  -Q(2) -Q(3) -Q(4)
        Q(1) Q(4) -Q(3)
        -Q(4) Q(1) Q(2)
        Q(3) -Q(2) Q(1)];

Repdot=C*omega;
