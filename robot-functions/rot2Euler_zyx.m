function y=rot2Euler_zyx(T)
%For a given DCM output a representation

if size(T,1)==4 && size(T,2)==4
    R = T(1:3, 1:3);   % Extract rotation part of T
elseif size(T,1)==1 && size(T,2)==9
    R=reshape(T,3,3);
elseif size(T,1)==9 && size(T,2)==1
    R=reshape(T,3,3);
else
    R=T;
end


sx=R(1,1);
sy=R(2,1);
sz=R(3,1);
nx=R(1,2);
ny=R(2,2);
nz=R(3,2);
ax=R(1,3);
ay=R(2,3);
az=R(3,3);




phi1=atan2(sy,sx);
phi2=phi1+pi;
theta1=atan2(-sz,((cos(phi1)*sx)+(sin(phi1)*sy)));
theta2=atan2(-sz,((cos(phi2)*sx)+(sin(phi2)*sy)));
psi1=atan2((sin(phi1)*ax)-(cos(phi1)*ay),(-sin(phi1)*nx)+(cos(phi1)*ny));
psi2=atan2((sin(phi2)*ax)-(cos(phi2)*ay),(-sin(phi2)*nx)+(cos(phi2)*ny));
y=[phi2;theta2;psi2];
y=[phi1;theta1;psi1];
