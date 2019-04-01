function [ Repdot ] = angVelocity2Quaterniondot(omega,R)
% angularVelocity2Quaterniondot Converts angular Velocity to change in quaternion 


Test=norm((R*R')-eye(3))
if Test>0.1
    disp 'Not a rotation matrix'
end

Q=rot2Quat_wxyz(R); 

C=0.5*[  -Q(2) -Q(3) -Q(4)
        Q(1) Q(4) -Q(3)
        -Q(4) Q(1) Q(2)
        Q(3) -Q(2) Q(1)];

Repdot=C*omega;
