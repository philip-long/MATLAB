function [ L ] = Lseg(u1,v1,u2,v2,Z1,Z2)
%Lseg Calculate interaction matrix  
%
% matrix for a segment, based on a pinhole camera model
%
% [ L ] = InteractionMatrixSeg(u1,v1,u2,v2,Z1,Z2)
%
% Input: The image coordinates for the two points
% and their depth 
%
% Imagepoint1=(u1,v1) Depth = Z1
% Imagepoint2=(u1,v2) Depth = Z2
% 
% Output: The interaction matrix L a 4x6 matrix


global CameraParameters

fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

uc=(u1+u2)/2;
vc=(v1+v2)/2;
l=((u1-u2)^2 +((v1-v2)^2))^0.5;
theta=atan((v1-v2)/(u1-u2));

L =[ - fu/(2*Z1) - fu/(2*Z2),                      - fs/(2*Z1) - fs/(2*Z2),                                                                                                                                                 (uc - u0 + (l*cos(theta))/2)/(2*Z1) - (u0 - uc + (l*cos(theta))/2)/(2*Z2),                                                                                                            fs + ((sin(2*theta)*l^2)/8 + u0*v0 - u0*vc - uc*v0 + uc*vc)/fv,                                                                                                     ((fs*sin(2*theta)*l^2)/8 + fs*u0*v0 - fs*u0*vc - fs*uc*v0 + fs*uc*vc)/(fu*fv) - ((l^2*cos(theta)^2)/4 - 2*u0*uc + fu^2 + u0^2 + uc^2)/fu,                                                      (fs*(u0 - uc))/fu - ((v0 - vc)*(fs^2 + fu^2))/(fu*fv)
     0,                                           - fv/(2*Z1) - fv/(2*Z2),                                                                                                                                                 (vc - v0 + (l*sin(theta))/2)/(2*Z1) - (v0 - vc + (l*sin(theta))/2)/(2*Z2),                                                                                                                    fv + ((l^2*sin(theta)^2)/4 - 2*v0*vc + v0^2 + vc^2)/fv,                                                                                                               (fs*v0^2 + fs*vc^2 - 2*fs*v0*vc + (fs*l^2*sin(theta)^2)/4)/(fu*fv) - ((sin(2*theta)*l^2)/8 + u0*v0 - u0*vc - uc*v0 + uc*vc)/fu,                                                                         (fv*u0 - fv*uc - fs*v0 + fs*vc)/fu
(fu*l*cos(theta)*(Z1 - Z2))/(Z1*Z2*(l^2)^(1/2)),     (l*(Z1 - Z2)*(fs*cos(theta) + fv*sin(theta)))/(Z1*Z2*(l^2)^(1/2)), (l*(l - 2*u0*cos(theta) + 2*uc*cos(theta) - 2*v0*sin(theta) + 2*vc*sin(theta)))/(2*Z1*(l^2)^(1/2)) + (l*(Z1*l + 2*Z1*u0*cos(theta) - 2*Z1*uc*cos(theta) + 2*Z1*v0*sin(theta) - 2*Z1*vc*sin(theta)))/(2*Z1*Z2*(l^2)^(1/2)), -(2*v0*(l^2)^(1/2) - 2*vc*(l^2)^(1/2) + u0*sin(2*theta)*(l^2)^(1/2) - uc*sin(2*theta)*(l^2)^(1/2) + 2*v0*sin(theta)^2*(l^2)^(1/2) - 2*vc*sin(theta)^2*(l^2)^(1/2))/(2*fv), ((l^2)^(1/2)*(3*u0 - 3*uc + u0*cos(2*theta) - uc*cos(2*theta) + v0*sin(2*theta) - vc*sin(2*theta)))/(2*fu) - ((l^2)^(1/2)*(3*fs*v0 - 3*fs*vc - fs*v0*cos(2*theta) + fs*vc*cos(2*theta) + fs*u0*sin(2*theta) - fs*uc*sin(2*theta)))/(2*fu*fv), ((l^2)^(1/2)*(sin(2*theta)*fs^2 - 2*cos(2*theta)*fs*fv + sin(2*theta)*fu^2 - sin(2*theta)*fv^2))/(2*fu*fv)
-(fu*sin(theta)*(Z1 - Z2))/(Z1*Z2*l),             ((Z1 - Z2)*(fv*cos(theta) - fs*sin(theta)))/(Z1*Z2*l),                                                                                                                                     ((Z1 - Z2)*(v0*cos(theta) - vc*cos(theta) - u0*sin(theta) + uc*sin(theta)))/(Z1*Z2*l),                                                                                        (u0*sin(theta)^2 - uc*sin(theta)^2 - (v0*sin(2*theta))/2 + (vc*sin(2*theta))/2)/fv,                                           (v0 - vc - (u0*sin(2*theta))/2 + (uc*sin(2*theta))/2 - v0*sin(theta)^2 + vc*sin(theta)^2)/fu + (fs*u0*sin(theta)^2 - fs*uc*sin(theta)^2 - (fs*v0*sin(2*theta))/2 + (fs*vc*sin(2*theta))/2)/(fu*fv),        (fv*(sin(theta)^2 - 1))/fu - (fs^2*sin(theta)^2 + fu^2*sin(theta)^2)/(fu*fv) + (fs*sin(2*theta))/fu];


end

