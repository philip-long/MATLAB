function [ Repdot ] = Omega_to_AngleAxisdot( u )


omega=u(1:3);
R=u(4:12);
R=reshape(R,3,3);
Test=norm((R*R')-eye(3));

if Test>0.1
    disp 'Not a rotation matrix'
end

thetadot=(transpose(u))*omega;
udot=(0.5*(((eye(3)-u*transpose(u))*cot(theta/2))-skew(u)))*omega;
    


Repdot=[thetadot;udot];