% Octave
% This gives the inertia at the centre of gravity of the link


%  CG{i} is with respect to the joint frame and is calculated as half the length of the cylinder 
%  I{i} is the inertia of a cylinder i where the axis is defined by its Z axis
%  R{i} is the rotation matrix between the Inertia reference frame and joint origin ^{j}R_{cgj}
%  M{i} is the mass of the link

function [inertia_link,mass_link,ms_link]=CalculateInertiaUR10(link)

M{1}=7.778; M{2}=12.93;M{3}=3.87;
M{4}=1.96;  M{5}=1.96; M{6}=0.202;M{7}=3.45;


Names{1}="ShoulderLink";
Names{2}="UpperArmLink";
Names{3}="ForeArmLink";
Names{4}="Wrist1Link";
Names{5}="Wrist2Link";
Names{6}="Wrist3Link";
Names{7}="VersaBall";


%I=cylinderinert(radius,length,mass)
I{1}=cylinderinert(0.075,0.178,M{1});
CG{1}=[0,0,0];
R{1}=eye(3);
% Cylinder 1 is along Z axis of joint 


I{2}=cylinderinert(0.075,0.612,M{2});
CG{2}=[0,0,0.612/2.0]; % COG is d3/2 along the -x axis
R{2}=TransMat(pi/2,'y','r');


I{3}=cylinderinert(0.075,0.5723,M{3});
CG{3}=[0,0,0.5723/2.0]; % COG is d4/2 along the -x axis
R{3}=TransMat(-pi/2,'y','r');


I{4}=cylinderinert(0.075,0.115,M{4});
CG{4}=[0,-.115/2,0]; % COG is r5/2 along the -y axis
R{4}=TransMat(-pi/2,'x','r');


I{5}=cylinderinert(0.075,0.12,M{5});
CG{5}=[0,0,0]; % COG is along the y axis
R{5}=eye(3);
% Cylinder 5 is along Z axis of joint 


I{6}=cylinderinert(0.075,0.12,M{6});
CG{6}=[0,0,0.0922/2];
R{6}=eye(3);
% Cylinder 6 is along Z axis  of joint r7/2

I{7}=cylinderinert(0.05,0.2,M{7});
CG{7}=[0,0,0];
R{7}=eye(3);
% Cylinder 6 is along Z axis

for i=1:7
	I_j{i}=R{i}*I{i}*transpose(R{i});
	MS{i}=M{i}*CG{i};
end

inertia_link=I_j{link};
mass_link=M{link};
ms_link=MS{link};


