function y=Rot2Euler_zxz(R)
% For a given DCM output a representation
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




phi1=atan2(-ax,ay);
phi2=phi1+pi;
theta1=atan2((ax*sin(phi1))-(cos(phi1)*ay),az);
theta2=atan2((ax*sin(phi2))-(cos(phi2)*ay),az);
psi1=atan2((-cos(phi1)*nx)-sin(phi1)*ny,cos(phi1)*sx+sin(phi1)*sy);
psi2=atan2((-cos(phi2)*nx)-sin(phi2)*ny,cos(phi2)*sx+sin(phi2)*sy);
y1=[phi1;theta1;psi1]';
y2=[phi2;theta2;psi2]';

% My Convention is to select the smallest abs of first angle

if abs(phi1)>abs(phi2)
    y=y2;
else
    y=y1;
end
    
