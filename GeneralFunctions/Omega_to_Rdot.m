function Repdot=Omega_to_Rdot(u)


omega=u(1:3);
R=u(4:12);
R=reshape(R,3,3);
Test=norm((R*R')-eye(3));
if Test>0.1
    disp 'Not a rotation matrix'
end


Repdot=skew(omega)*Rt1;
    
