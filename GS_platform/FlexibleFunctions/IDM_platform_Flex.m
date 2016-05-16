function [ F ] = IDM_platform_Flex(u)
% IDM_platform Inverse Dynamic model (NE-equations) of rigid platform
%   This function calculates the inverse dynamic
%   model of the rigid platform and the centre of
%   mass. This means that the Force reaquried to
%   move the platform given its current state is
%   calculated from the (desired) acceleration,
%   the state and the mass properties
global G3 Mp MSP I_rr


V=u(1:6)';Vdot=u(7:12)';

% Inertia Matrix
JJp=[eye(3)*Mp -skew(MSP)
     skew(MSP) I_rr];
 
% The effect of gravity
Vdot= Vdot-[0;0;G3;0;0;0];

Hp=[cross(V(4:6),(cross(Vdot(4:6),MSP)))
    cross(Vdot(4:6),I_rr*Vdot(4:6))];
F=JJp*Vdot + Hp;
 
end

