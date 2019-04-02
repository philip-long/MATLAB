function [ L ] = interactionMatrixPts(Pim1,Pim2,depth1,depth2)
% Compute the interaction matrix for two points
%   Compute the interaction matrix for two points Pim1 Pim2 of a given 
%   depth depth1 depth2 based on a pinhole camera model


global CameraParameters
fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

u1=Pim1(1);v1=Pim1(2);
u2=Pim2(1);v2=Pim2(2);
Z1=depth1;Z2=depth2;

L = [ -fu/Z1, -fs/Z1, -(u0 - u1)/Z1, fs + ((u0 - u1)*(v0 - v1))/fv, - fu - ((u0 - u1)*(fv*u0 - fv*u1 - fs*v0 + fs*v1))/(fu*fv), fs*((u0 - u1)/fu - (fs*(v0 - v1))/(fu*fv)) - (fu*(v0 - v1))/fv
      0, -fv/Z1, -(v0 - v1)/Z1,           fv + (v0 - v1)^2/fv,        (fs*(v0 - v1)^2)/(fu*fv) - ((u0 - u1)*(v0 - v1))/fu,                          (fv*(u0 - u1))/fu - (fs*(v0 - v1))/fu
 -fu/Z2, -fs/Z2, -(u0 - u2)/Z2, fs + ((u0 - u2)*(v0 - v2))/fv, - fu - ((u0 - u2)*(fv*u0 - fv*u2 - fs*v0 + fs*v2))/(fu*fv), fs*((u0 - u2)/fu - (fs*(v0 - v2))/(fu*fv)) - (fu*(v0 - v2))/fv
      0, -fv/Z2, -(v0 - v2)/Z2,           fv + (v0 - v2)^2/fv,        (fs*(v0 - v2)^2)/(fu*fv) - ((u0 - u2)*(v0 - v2))/fu,                          (fv*(u0 - u2))/fu - (fs*(v0 - v2))/fu];
 


end

