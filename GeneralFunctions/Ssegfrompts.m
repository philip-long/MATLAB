function [ S ] = Ssegfrompts( u )
%Ssegfrompts: Generate segment in camera frame from  points
%
%   Inputs: u=[Pt1,Pt2;T] Outputs = S 
% 
% 1. Must use current vision location to find pts
% in camera frame
% 2. Using the pinhole camera model and some made
% up camera parameters to transfom to u1 v1 u2 v2
% 3. Find the segment parameters relating to u1 v1
% u2 v2, xp yp l and theta



global CameraParameters
fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

p01=[u(1:3);1]; %Pt 1 in homogenous format
p02=[u(4:3+3);1];%Pt2  in homogenous format

T=reshape(u(7:end),4,4);


% Collineation Matrix

K=[fu fs u0;0 fv v0; 0 0 1];


 
% 2. Find the location of the points in the vision robot's end
% effector frame
P1=T\p01;
P2=T\p02;


% 3. Normalise the image points
% Point 1
xn1=P1(1)/P1(3);yn1=P1(2)/P1(3);
% Point 2
xn2=P2(1)/P2(3);yn2=P2(2)/P2(3);

% Convert Image Points
Pim1=K*[xn1;yn1;1];
Pim2=K*[xn2;yn2;1];

% plot the images
u1=Pim1(1);v1=Pim1(2);u2=Pim2(1);v2=Pim2(2);


uc=(u1+u2)/2;
vc=(v1+v2)/2;
l=((u1-u2)^2 +(v1-v2)^2)^0.5;
theta=atan((v1-v2)/(u1-u2));

S=[uc;vc;l;theta];

end

















