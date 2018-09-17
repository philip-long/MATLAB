function r=ZXZ_to_Rot(u)
% for a given representation output dcm

phi=u(1);
theta=u(2);
if theta==0;
    disp 'singular'
end
psi=u(3);
r=TransMat(phi,'z','rot')*TransMat(theta,'x','rot')*TransMat(psi,'z','rot');


