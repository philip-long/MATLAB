function [r] = xyz_euler2Rot(u)
phi=u(1);
theta=u(2);
psi=u(3);

x=[1;0;0];
y=[0;1;0];
z=[0;0;1];
r=expm(skew(x)*phi) * expm(skew(y)*theta)*expm(skew(z)*psi);

end

