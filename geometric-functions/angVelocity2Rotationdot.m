function Repdot=angVelocity2Rotationdot(omega,R)
% Omega_to_AngleAxisdot Convert angular velocity to change in rotation matrix

R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end


Repdot=skew(omega)*Rt1;
    
