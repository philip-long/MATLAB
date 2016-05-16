function R=ZYX_to_Rot(u)


phi1=u(1);
theta1=u(2);
if theta1==pi/2;
    disp 'Singular Configuration'
end
psi1=u(3);


R=TransMat(phi1,'z','rot')*TransMat(theta1,'y','rot')*TransMat(psi1,'x','rot');
