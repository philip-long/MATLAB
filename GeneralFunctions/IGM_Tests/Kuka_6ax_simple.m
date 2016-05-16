
clear all
clc

% STANFORD DH parameters by Khailil & Dombre


global alpha d theta r sigma Joints r1 r3 r5 r7



global qlower qupper



% Joint limits not necessarily true
qlower=[-12 -120 -170  -120 -170 -130 ]*pi/180;
qupper=[120 120 170 120 170 130]*pi/180;


Joints=6; %No of joints
%qc=rand(Joints,1).*(qupper'-qlower')+qlower';
Ant = [0,1,2,3,4,5];
sigma= [0,0,0,0,0,0];
b= [0,0,0,0,0,0];
%Assign r
r= [0,0,0,0.39,0,0];
rcell=num2cell(r);
[r1 r2 r3 r4 r5 r6 ]=rcell{:};

%Assign d
d= [0,0,0.4,0,0,0];
dcell=num2cell(d);
[d1 d2 d3 d4 d5 d6]=dcell{:};

gamma= [0,0,0,0,0,0,0];
te=pi/4;
alpha= [0,pi/2,te,-pi/2,pi/2,-pi/2];

mu= [1,1,1,1,1,1,1];

theta= [0,0,0,0,0,0,0];


rotdisp=3; % Orientation Representation 
           % 1=Quaternions [Scalar; Vector] 
           % 2=Euler Angles
           % 3=Angle Axis [angle;axis]
           % 4=Roll Pitch Yaw
           % 5=DCM Matrix


% %%
% 
% %In order to find the Jacobian matrix at the tool frame rather than the last joints
% q=qinit
% T= T70(q);% Transformation matrix 
% J= J70(q); 
% P=TnE(1:3.4); %Distance to tool frame
% L=T(1:3.1:3)*P; %Change the reference frame to the world one  
% Lhat=skew(L); %Skew the matrix
% 
% J0e=[eye(3) -Lhat; zeros(3) eye(3) ]*J; % Change the point of the Jacobian
% 
% 