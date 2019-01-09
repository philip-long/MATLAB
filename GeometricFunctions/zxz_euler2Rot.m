function r=zxz_euler2Rot(u)
% for a given representation output dcm

phi=u(1);
theta=u(2);
if theta==0;
    disp 'singular'
end
psi=u(3);
x=[1;0;0];
y=[0;1;0];
z=[0;0;1];
r=expm(skew(z)*phi) * expm(skew(x)*theta)*expm(skew(z)*psi);
%r=TransMat(phi,'z','rot')*TransMat(theta,'x','rot')*TransMat(psi,'z','rot');


