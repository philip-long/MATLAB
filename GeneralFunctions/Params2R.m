
clear all
clc

% Puma DH parameters by Khailil & Dombre


global alpha d theta r sigma Joints



global qlower qupper



% Joint limits not necessarily true
qlower=[-140 -150 -110 -80 -60 -130]*pi/180;
qupper=-qlower; 


Joints=4; %No of joints
Ant = [0,1];
sigma= [0,0,];
b= [0,0,0,0,0,0];
%Assign r
r= [0,10];
rcell=num2cell(r);
[r1 r2]=rcell{:};

%Assign d
d= [0,10 ];
dcell=num2cell(d);
[d1 d2]=dcell{:};

gamma= [0,0,0,0,0,0,0];

alpha= [0,pi/4];

mu= [1,1,1,1,];

theta= [0,0,0,0];


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