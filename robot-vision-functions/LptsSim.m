function [ L ] = LptsSim(Pim1,Pim2,depth1,depth2)
%LPTS Summary of this function goes here
%   Detailed explanation goes here


global CameraParameters
CameraParameters=[1000;1000;0;0;0]
fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

u1=Pim1(1);v1=Pim1(2);
u2=Pim2(1);v2=Pim2(2);
Z1=depth1;Z2=depth2;

L =[ -1/Z1,     0, u1/Z1,    u1*v1, - u1^2 - 1,  v1
         0, -1/Z1, v1/Z1, v1^2 + 1,     -u1*v1, -u1
  -1/Z2,     0, u2/Z2,    u2*v2, - u2^2 - 1,  v2
     0, -1/Z2, v2/Z2, v2^2 + 1,     -u2*v2, -u2];


end

