function [J] = J_Plat(q)
%J_Plat This functions returns the jacobian matrix of the gough stewart robot, which relates the actued joint velocity to the platform velocity
%
%
%  J_plat=inv( [^{0} u_{1}^{T} ,  -^{0} u_{1}^{T}skew(P_{1}) ]
%              [^{0} u_{2}^{T} ,  -^{0} u_{2}^{T}skew(P_{2}) ]
%              [^{0} u_{3}^{T} ,  -^{0} u_{3}^{T}skew(P_{3}) ]
%              [^{0} u_{4}^{T} ,  -^{0} u_{4}^{T}skew(P_{4}) ]
%              [^{0} u_{5}^{T} ,  -^{0} u_{5}^{T}skew(P_{5}) ]
%              [^{0} u_{6}^{T} ,  -^{0} u_{6}^{T}skew(P_{6}) ])
global PM1 PM2 PM3 PM4 PM5 PM6 T0M_init
q1=q(1:3);
q2=q(4:6);
q3=q(7:9);
q4=q(10:12);
q5=q(13:15);
q6=q(16:18);
X=u(19:24);
T0M=GS_X_2_T(X);
% Attachment point location in world frame

T01=T03_leg_GS(q1,1); T02=T03_leg_GS(q2,2); T03=T03_leg_GS(q3,3);
T04=T03_leg_GS(q4,4); T05=T03_leg_GS(q5,5); T06=T03_leg_GS(q6,6);

L01=T0M(1:3,1:3)*PM1(1:3); L02=T0M(1:3,1:3)*PM2; L03=T0M(1:3,1:3)*PM3;
L04=T0M(1:3,1:3)*PM4; L05=T0M(1:3,1:3)*PM5; L06=T0M(1:3,1:3)*PM6;

u01=T01(1:3,3); u02=T02(1:3,3); u03=T03(1:3,3);
u04=T04(1:3,3); u05=T05(1:3,3); u06=T06(1:3,3);

Jinv=[u01' -u01'*skew(L01(1:3))
      u02' -u02'*skew(L02(1:3))
      u03' -u03'*skew(L03(1:3))
      u04' -u04'*skew(L04(1:3))
      u05' -u05'*skew(L05(1:3))
      u06' -u06'*skew(L06(1:3))];
Jm=inv(Jinv);

end

