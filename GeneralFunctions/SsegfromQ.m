function [ S ] = SsegfromQ( u )
%SsegfromQ: Generate segment from joint configs
%
%   Inputs: u=[q1;q2] Outputs = S
% 
% function "functions" by six easy steps:
% 1. From q1 we generate two knife points, p1  and p2
% in the world frame p01,  p02
% 2. Using q2 we tranforms this to the robot end
% effector frame DGM(q2)= 0Te pe1=inv(0Te)*p01
% 3. Using the pinhole camera model and some made
% up camera parameters to transfom to u1 v1 u2 v2
% 4. Find the segment parameters relating to u1 v1
% u2 v2, xp yp l and theta
% 

global qinit CameraParameters 

fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

q1=u(1:length(qinit)); % Cutting Robot
q2=u(length(qinit)+1:length(qinit)+7);% Vision Robot


% Collineation Matrix

K=[fu fs u0;0 fv v0; 0 0 1];


% 1. Why T70_B? Gives exactly the same as T70 expcet
% the tool offset is absent. Therefore we can
% take this as the point one of knife 
p01=T70_B(q1);p01=[p01(1:3,4);1];
p02=T70(q1);p02=[p02(1:3,4);1];
 
% 2. Find the location of the points in the vision robot's end
% effector frame
P1=(T70_0C(q2))\p01;
P2=(T70_0C(q2))\p02;


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

















