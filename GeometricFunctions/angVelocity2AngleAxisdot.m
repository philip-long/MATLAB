function [ Repdot ] = angVelocity2AngleAxisdot(omega,R)
% Omega_to_AngleAxisdot Convert angular velocity to change in angle axis


Test=norm((R*R')-eye(3));

if Test>0.1
    disp 'Not a rotation matrix'
end

thetadot=(transpose(u))*omega;
udot=(0.5*(((eye(3)-u*transpose(u))*cot(theta/2))-skew(u)))*omega;
    


Repdot=[thetadot;udot];
