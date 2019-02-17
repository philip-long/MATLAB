function GAMMA = IDM_GS_rigid(u)
%IDM_GS gives the IDM for the rigid G-S robot
%   GAMMA_{a}=J^{T}*F_{p}+sum_{i=1}^{n} 
%  (\frac{\partial \dot{q}_{i}}{\partial\dot{q}_{a} } )^T
%  H_{i}                 

global PM1 PM2 PM3 PM4 PM5 PM6

% Extract the data
% [q1,q2,q3,q4,q5,q6] all the joints of the robot i.e [q11 q12 q13,q21 q22 q23...
% [qdot1,qdot2,qdot3,qdot4,qdot5,qdot6] all the joints of the robot
X=u(1:6);V=u(7:12);Vdot=u(13:18); 
q=u(19:36); qdot=u(37:54);qddot=u(55:72);



% Calculates the Platform forces at the platform
% origin
F= IDM_platform([V,Vdot]);


% Current location of Platform required for platform Jacobian
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
  
% Jacobian  
Jm=inv(Jinv);

% Platform Jacobians
J_v1=[eye(3) -skew(L01(1:3))];J_v2=[eye(3) -skew(L02(1:3))];J_v3=[eye(3) -skew(L03(1:3))];
J_v4=[eye(3) -skew(L04(1:3))];J_v5=[eye(3) -skew(L05(1:3))];J_v6=[eye(3) -skew(L06(1:3))];

% leg Jacobians
J_1=J_leg_GS(q(1:3),1);J_2=J_leg_GS(q(4:6),2);J_3=J_leg_GS(q(7:9),3);
J_4=J_leg_GS(q(10:12),4);J_5=J_leg_GS(q(13:15),5);J_6=J_leg_GS(q(16:18),6);

% Leg Dynamics
H1=IDM_leg_GS([q(1:3),qdot(1:3),qddot(1:3)]); H2=IDM_leg_GS([q(4:6),qdot(4:6),qddot(4:6)]);
H3=IDM_leg_GS([q(7:9),qdot(7:9),qddot(7:9)]); H4=IDM_leg_GS([q(10:12),qdot(10:12),qddot(10:12)]);
H5=IDM_leg_GS([q(13:15),qdot(13:15),qddot(13:15)]); H6=IDM_leg_GS([q(16:18),qdot(16:18),qddot(16:18)]);

% Leg dynamics to platform point
Hlegs1=(transpose(J_v1)*transpose(inv(J_1))*H1);
Hlegs2=(transpose(J_v2)*transpose(inv(J_2))*H2);
Hlegs3=(transpose(J_v3)*transpose(inv(J_3))*H3);
Hlegs4=(transpose(J_v4)*transpose(inv(J_4))*H4);
Hlegs5=(transpose(J_v5)*transpose(inv(J_5))*H5);
Hlegs6=(transpose(J_v6)*transpose(inv(J_6))*H6);
Hlegs=Hlegs1+Hlegs2+Hlegs3+Hlegs4+Hlegs5+Hlegs6;

% Gamma
GAMMA=transpose(Jm)*(F + Hlegs);

end

