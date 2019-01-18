function R=zyx_euler2Rot(u)


phi1=u(1);
theta1=u(2);
if theta1==pi/2
    disp 'Singular Configuration'
end
psi1=u(3);

x=[1;0;0];
y=[0;1;0];
z=[0;0;1];
R=expm(skew(z)*phi1) * expm(skew(y)*theta1)*expm(skew(x)*psi1);
%R=TransMat(phi1,'z','rot')*TransMat(theta1,'y','rot')*TransMat(psi1,'x','rot');
