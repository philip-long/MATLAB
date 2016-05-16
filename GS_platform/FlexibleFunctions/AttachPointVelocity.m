function [Vout] = AttachPointVelocity( u )
%IKM_RIGID This functions cacculates qdot from the platform velocity and the current configuration
%
% All calculations are computed in the world frame
%
global PM1 PM2 PM3 PM4 PM5 PM6 Phi_P Phi_1 Phi_2 Phi_3 Phi_4 Phi_5 Phi_6
%%
X=u(1:6);V=u(7:12);
q=u(13:30);
q_e=u(31:33)
qdot_e=u(34:36)


%%
T0M=[ZXZ_to_Rot(X(4:6)) X(1:3);0 0 0 1]; % This matrix changes to the fixed frame

R0M=eye(3);
% Location of all legs required for Jacobian
T01=T03_leg_GS(q(1:3),1); T02=T03_leg_GS(q(4:6),2); T03=T03_leg_GS(q(7:9),3);
T04=T03_leg_GS(q(10:12),4); T05=T03_leg_GS(q(13:15),5); T06=T03_leg_GS(q(16:18),6);

R0M=T0M(1:3,1:3);
% Distance between the platform and attachment points represented in the world frame

L01=R0M*PM1; L02=R0M*PM2; L03=R0M*PM3;
L04=R0M*PM4; L05=R0M*PM5; L06=R0M*PM6;

% Axis of prismatic joints
u01=T01(1:3,3); u02=T02(1:3,3); u03=T03(1:3,3);
u04=T04(1:3,3); u05=T05(1:3,3); u06=T06(1:3,3);

deltaP0_p=R0M*Phi_P(1:3,:)*q_e; % deltaP0_p is measured in the local body reference frame

V(1:3)=V(1:3)-(Phi_P(1:3,:)*qdot_e);

r1=L01-deltaP0_p;r2=L02-deltaP0_p;r3=L03-deltaP0_p;
r4=L04-deltaP0_p;r5=L05-deltaP0_p;r6=L06-deltaP0_p;

v1=[eye(3) -skew(r1) ( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;qdot_e]
%v1=[eye(3) -skew(r1) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;0;0;0]
v2=[eye(3) -skew(L01) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;0;0;0];
v3=[eye(3) -skew(r3) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;qdot_e];
v4=[eye(3) -skew(r4) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;qdot_e];
v5=[eye(3) -skew(r5) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;qdot_e];
v6=[eye(3) -skew(r6) R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))]*[V;qdot_e];

Vout=[v1;v2;v3;v4;v5;v6];
end

