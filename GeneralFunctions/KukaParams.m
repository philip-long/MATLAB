
clear all
clc

%DynamicParams
%ImpedanceParams
% Parameter file for the kinematics of robot-- Well Known---


global alpha d theta r sigma Joints qinit
global r3 r5 r7 r1

global rotdisp
global qlower qupper
global Pinit RepInit


% Robot¨Parameters
qlower=[-170 -120 -170 -120 -170 -130 -170]*pi/180;
qupper=-qlower;
Joints=7; %No of joints
Ant = [0,1,2,3,4,5,6];
sigma= [0,0,0,0,0,0,0];
b= [0,0,0,0,0,0,0];
%Assign r
r= [0,0,0.4,0,0.39,0,0];
rcell=num2cell(r);
[r1 r2 r3 r4 r5 r6 r7]=rcell{:};
%Assign d
d= [0,0,0,0,0,0,0];
dcell=num2cell(d);
[d1 d2 d3 d4 d5 d6 d7]=dcell{:};
gamma= [0,0,0,0,0,0,0];
alpha= [0,pi/2,-pi/2,-pi/2,pi/2,pi/2,-pi/2];
mu= [1,1,1,1,1,1,1];
theta= [0,0,0,0,0,0,0];


rotdisp=3; % Orientation Representation 
           % 1=Quaternions [Scalar; Vector] 
           % 2=Euler Angles
           % 3=Angle Axis [angle;axis]
           % 4=Roll Pitch Yaw
           % 5=DCM Matrix

% Configuration Parameters
%qinit=rand(7,1);  
qinit=[0.4809    0.7292    0.9376    0.5173    0.9031    0.2182    0.8732];
Tinit=T70(qinit);
Pinit=(Tinit(1:3,4));
RepInit=reshape(Tinit(1:3,1:3),9,1);



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