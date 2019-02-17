function [ qdot ] = IKM_Flex( u )
%IKM_RIGID This functions cacculates qdot from the platform velocity and the current configuration
%
% All calculations are computed in the world frame
%
global PM1 PM2 PM3 PM4 PM5 PM6 Phi_P
%%
X=u(1:6);V=u(7:12);
q=u(13:30);
q_e=u(31:33);
qdot_e=u(34:36);


%%
T0M=[ZXZ_to_Rot(X(4:6)) X(1:3);0 0 0 1]; % This matrix changes to the fixed frame


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


r1=L01-deltaP0_p;r2=L02-deltaP0_p;r3=L03-deltaP0_p;
r4=L04-deltaP0_p;r5=L05-deltaP0_p;r6=L06-deltaP0_p;

% Inverse Jacobian Matrix
Jinv=[u01' -u01'*skew(r1)   u01'*R0M*( Phi_P(1:3,:)-skew(r1)*(Phi_P(4:6,:)))
      u02' -u02'*skew(r2)   u02'*R0M*( Phi_P(1:3,:)-skew(r2)*(Phi_P(4:6,:)))
      u03' -u03'*skew(r3)   u03'*R0M*( Phi_P(1:3,:)-skew(r3)*(Phi_P(4:6,:)))
      u04' -u04'*skew(r4)   u04'*R0M*( Phi_P(1:3,:)-skew(r4)*(Phi_P(4:6,:)))
      u05' -u05'*skew(r5)   u05'*R0M*( Phi_P(1:3,:)-skew(r5)*(Phi_P(4:6,:)))
      u06' -u06'*skew(r6)   u06'*R0M*( Phi_P(1:3,:)-skew(r6)*(Phi_P(4:6,:)))];
  
  qdot_e
qdot=Jinv*[V;qdot_e];  

end

