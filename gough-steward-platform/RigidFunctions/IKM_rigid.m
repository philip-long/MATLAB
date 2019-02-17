function [ qdot ] = IKM_rigid( u )
%IKM_RIGID This functions cacculates qdot from the platform velocity and the current configuration
global PM1 PM2 PM3 PM4 PM5 PM6

X=u(1:6);V=u(7:12);
q=u(13:30);
T0M=[ZXZ_to_Rot(X(4:6)) X(1:3);0 0 0 1];


% Location of all legs required for Jacobian
T01=T03_leg_GS(q(1:3),1); T02=T03_leg_GS(q(4:6),2); T03=T03_leg_GS(q(7:9),3);
T04=T03_leg_GS(q(10:12),4); T05=T03_leg_GS(q(13:15),5); T06=T03_leg_GS(q(16:18),6);

% Distance between the platform and attachment points represented in the world frame

L01=T0M(1:3,1:3)*PM1; L02=T0M(1:3,1:3)*PM2; L03=T0M(1:3,1:3)*PM3;
L04=T0M(1:3,1:3)*PM4; L05=T0M(1:3,1:3)*PM5; L06=T0M(1:3,1:3)*PM6;

% Axis of prismatic joints
u01=T01(1:3,3); u02=T02(1:3,3); u03=T03(1:3,3);
u04=T04(1:3,3); u05=T05(1:3,3); u06=T06(1:3,3);

% Inverse Jacobian Matrix
Jinv=[u01' -u01'*skew(L01(1:3))
      u02' -u02'*skew(L02(1:3))
      u03' -u03'*skew(L03(1:3))
      u04' -u04'*skew(L04(1:3))
      u05' -u05'*skew(L05(1:3))
      u06' -u06'*skew(L06(1:3))];
  
  
qdot=Jinv*V;  

end

