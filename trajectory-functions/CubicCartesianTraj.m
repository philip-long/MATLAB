function [wTcurrent,twist,acceleration] = CubicCartesianTraj(wTinit,wTfinal,t,tfinal)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
wTcurrent=eye(4);
twist=zeros(6,1);
acceleration=zeros(6,1);

rt=(10*((t/(tfinal))^3))-(15*((t/(tfinal))^4))+(6*((t/(tfinal))^5));
rtdot =(30*t^2)/(tfinal)^3 - (60*t^3)/(tfinal)^4 + (30*t^4)/(tfinal)^5;
rtddot=((120*t^3)/(tfinal)^5 - (180*t^2)/(tfinal)^4 + (60*t)/(tfinal)^3);

D_position=wTfinal(1:3,4)-wTinit(1:3,4);  
RotuAlpha=wTfinal(1:3,1:3)*transpose(wTinit(1:3,1:3)) ;
alpha_u=rot2AngleAxis(RotuAlpha);
alpha=alpha_u(1);
u=alpha_u(2:4);
if(norm(alpha_u(1))<1e-12)
    RotuAlpha_rt=eye(3);
else
  nu=rt*alpha; %evolution of alpha in t
  RotuAlpha_rt=(u*(u').*(1-cos(nu))) + (eye(3)*cos(nu))+ (skew(u)*sin(nu)); 
end
wTcurrent(1:3,1:3)=RotuAlpha_rt*wTinit(1:3,1:3);

wTcurrent(1:3,4)=wTinit(1:3,4)+rt*D_position;
twist(1:3)=rtdot*D_position;
twist(4:6)=(u*rtdot*alpha);
acceleration(1:3)=rtddot*D_position;
acceleration(4:6)=u*rtddot*alpha;
end

