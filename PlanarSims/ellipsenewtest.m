clear all, clc,close all

global MX3 MY3 MZ3 MX2 MY2 MZ2 MX1 MY1 MZ1
global XX3 XX2 YY3 YY2 ZZ3 ZZ2 XX1 YY1 ZZ1
global XY1 XY2 XY3 XZ1 XZ2 XZ3 YZ1 YZ2 YZ3 
global d1 d2 d3 M1 M2 M3
global IA1 IA2 IA3 G3

IA1=0.0; IA2=0.0; IA3=0.0;
M1=1.0; M2=1.0;M3=1.0;
MX3=0.5;MY3=0.0; MZ3=0.0;
MX2=0.0;MY2=0.0;MZ2=0.0;
MX1=0.0;MY1=0.0;MZ1=0.0; 
XX3=0.0; XX2=0.0;XX1=0.0; 
XY3=0.0; XY2=0.0;XY1=0.0;
XZ1=0.0;XZ2=0.0;XZ3=0.0; 
YY1=0.2;YY2=0.2;YY3=0.2;
YZ1=0.0;YZ2=0.0;YZ3=0.0; 
ZZ3=1.0;ZZ2=1.0; ZZ1=1.0;
G3=0.0;


links=3;
steps=[2,3,1];
d1=0.0;
d2=2.0;
d3=0.75;

q=[rand(3,1)*2];


qdot=[0.1;0.5;0];
qddot=[0.1;0.2;0.3];
T_01=T01(q);T_02=T02(q);T_03=T03(q);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
V=J_03*qdot;


A=Inertia_matrix(q);
M_inv=J_03(1:2,:)*inv(A)*transpose(J_03(1:2,:));

[s v]=eig(M_inv);

if(v(2,2)>v(1,1))
  ax2=s(:,1);
  l2=v(1,1);
  
  ax1=s(:,2);
  l1=v(2,2);
else
  ax1=s(:,1);
  l1=v(1,1);
  
  ax2=s(:,2);
  l2=v(2,2);
endif

phi=atan(ax1(2)/ax1(1));
h = ellipsedraw(l1,l2,0.0,0.0,phi,'r-');

set(h,'LineWidth',6);
hold on
plot([0.0,(s(1,1)*v(1,1))],[0.0,s(2,1)*v(1,1)])
plot([0.0,(s(1,1)*-v(1,1))],[0.0,s(2,1)*-v(1,1)])
plot([0.0,(s(1,2)*v(2,2))],[0.0,s(2,2)*v(2,2)])
plot([0.0,(s(1,2)*-v(2,2))],[0.0,s(2,2)*-v(2,2)])

u=[1;0];
plot([0.0,(u(1))],[0.0,u(2)])

t=atan( -l1/l2 * tan(phi))

 x = l1*cos(t);
 y = l2*sin(t);
% Coordinate transform 
%----------------------------------------
 X = cos(phi)*x - sin(phi)*y;
 Y = sin(phi)*x + cos(phi)*y;
 
plot(X,Y,"ko","MarkerSize",10.0)

u=
t=acos( (s(:,2)'*u)/ norm(s(:,2))*norm(u)  )
r= ( l1*l2 ) /( ((l2*cos(theta))^2) + ((l1*sin(theta))^2))^0.5




%%%%%%%%%%%%%%%% Finally C++ test %%%%%%%%%%%%%%%%%%%%%%%%%

close all,clc,clear all

M=[0.0537863 -0.020942
-0.020942 0.0711678];
r_diff=[ -2.74171  -2.78142  0  ];
major_axis=[0.0 1 ];
[s v]=eig(M);

if(v(2,2)>v(1,1))
  ax2=s(:,1);
  l2=v(1,1);
  
  ax1=s(:,2);
  l1=v(2,2);
else
  ax1=s(:,1);
  l1=v(1,1);
  
  ax2=s(:,2);
  l2=v(2,2);
endif

phi=atan(ax1(2)/ax1(1));
h = ellipsedraw(l1,l2,0.0,0.0,phi,'r-');
set(h,'LineWidth',6);
hold on
plot([0.0,(ax1(1)*l1)],[0.0,ax1(2)*l1])
plot([0.0,(ax1(1)*-l1)],[0.0,ax1(2)*-l1])
plot([0.0,(ax2(1)*l2)],[0.0,ax2(2)*l2])
plot([0.0,(ax2(1)*-l2)],[0.0,ax2(2)*-l2])

u=r_diff/norm(r_diff);
u=u(1:2)'
theta=acos( (ax1'*u)/ norm(ax1)*norm(u)  )
theta=acos( (major_axis*u)/ norm(major_axis)*norm(u)  )
hold on
plot([0.0,(u(1))],[0.0,u(2)])
r= ( l1*l2 ) /( ((l2*cos(theta))^2) + ((l1*sin(theta))^2))^0.5



