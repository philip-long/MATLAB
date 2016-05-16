% Script to find the interaction matrix 

% Camera Parameters:
px = 561.0859613;	 py = 561.8516144;
u0 = 353.0059416;	 v0 = 244.0586532;
kud = -0.3120862655; kdu = 0.3798145096;

% Define the relationship between and image point
% and the normalized coorindates

u=u0 + (px*x) + (kud*(r^2)*px*x);
v=v0 + (py*y) + (kud*(r^2)*py*y);

x1=(u1-u0)/px;
y1=(v1-u0)/py;
x2=(u2-u0)/px;
y2=(v2-v0)/py;

%% Interaction matrix for 2 points
% 2 Points 
clear all,clc
syms X Y Z u1 v1 u2 v2 Z1 Z2
syms fu fs u0 fv v0

xn=X/Z;
yn=Y/Z;
% Jacobian from 3D point to normalized image
mn=[xn,yn,1];
m=[X,Y,Z];
J=jacobian(mn,m);
% Colineation matrix
K=[fu fs u0;0 fv v0; 0 0 1];
% Screw Transform from velocity to point change
L=[ -eye(3) skew(m)];

Lw=K*J*L;
% Sub in expression of X Y Z in image points u v

X=Z1*((u1 - u0)/fu - (fs*(v1 - v0))/(fu*fv));
Y=Z1*((v1 - v0)/fv);
Z=Z1;

LP1=simplify(subs(Lw));
% For second image point
X=Z2*((u2 - u0)/fu - (fs*(v2 - v0))/(fu*fv));
Y=Z2*((v2 - v0)/fv);
Z=Z2;
LP2=simplify(subs(Lw));
% 
% Interaction matrix for two image points
L=[LP1(1:2,:);LP2(1:2,:)]
Mat2Cmatrix(L)
%fs=0; v0=0; u0=0;
%L=subs(L)


%% interaction matrix for segment defined by uc vc l
%and theta


syms uc vc l theta u1 u2 v1 v2
% u1=uc+(l/2)*cos(theta);
% u2=uc-(l/2)*cos(theta);
% v1=vc-(l/2)*sin(theta);
% v2=vc-(l/2)*sin(theta);

uc=(u1+u2)/2;
vc=(v1+v2)/2;
l=(((u1-u2)^2) + ((v1-v2)^2))^0.5;
theta=atan((v1-v2)/(u1-u2));

s1=[u1;v1;u2;v2];
s2=[uc;vc;l;theta];

ds2ds1=(jacobian(s2,s1));
Lseg=(ds2ds1*L);
clear uc vc l theta
syms uc vc l theta
% Final step sub in for u1 and u2
u1=uc+(l/2)*cos(theta);
u2=uc-(l/2)*cos(theta);
v1=vc+(l/2)*sin(theta);
v2=vc-(l/2)*sin(theta);

Lsegn=simple(subs(Lseg));



%% Last Part write a text file describing Lsegn as a C++ matrix 
Mat2Cmatrix(Lsegn)















