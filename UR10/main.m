
clear all, clc, close all

initUR10_ros
q=[pi/5,1.0,-pi/2,-pi/2,pi/6,pi/4];
%%q=[0.0,0.0,0.0,0.0,0.0,0.0];
Axis=["x","y","z"];

[T,Ros] = UR10_DGM_ij (q); % Gives transformation from each frame to precedent frame
[Tw,Rosw] = UR10_DGM_0j (q); % Gives transformation from each frame to world frame

e=6; % target frame only joint frames
TjE=eye(4); % Transformation from joint frame to frame of interest 
%TjE=T{7};
%TjE=TransMat([0.0 -0.044 0.35])*RPY_to_Rot([-1.570796326 0.0 -1.57079632],"T");
w=0; % the desired base frame i.e. the frame on the chain from which the jacobian is measured and also in which in is represented
T0n=eye(4);


iter=1;
for k=(w+1):e
    a=(Tw{k}(1:3,find(rq{k}==Axis)));
    askew=skew(a);
    J0n(:,k)=[askew*(Tw{6}(1:3,4)-Tw{k}(1:3,4))
              a];           
    iter=iter+1;
end

J0n
% Transform the Jacobian to a fixed frame
j=6;
P=TjE(1:3,4);   % Distance to tool frame from joint j
L=Tw{j}(1:3,1:3)*P; % Change the reference frame to the world one by multiplying by transform
Lhat=skew(L);   % Skew the matrix
J0e=[eye(3) -Lhat; zeros(3) eye(3) ]*J0n; % Change the point of the Jacobian

% Check the Jacobian is correct numerically
% ToDo transform to base frame
% Ensure that a speed defined in a tool frame can be transform to base frame

