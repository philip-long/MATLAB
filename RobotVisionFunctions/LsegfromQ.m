function [ L] = LsegfromQ( u )
%LsegfromQ: Generate Ls from robots joints
%
%  This function generates the camera velocity
%  using the joint positions of both robots
%   Inputs: u=[q1;q2] Outputs = V
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
% 5. Generate the Lnteraction matrix  for L using
% the image points and also the known depth
 

global qinit  
CameraParameters=[560; 560; 0;  374.4020879	;  180.2622981 ];
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
Pim1=K*[xn1;yn1;1];Pim2=K*[xn2;yn2;1];

% plot the images
% 4. & 5. Generate the Interaction Matrix
L=Lseg(Pim1(1),Pim1(2),Pim2(1),Pim2(2),P1(3),P2(3));


end

















