function [Deformation] = DeformationFlex( u )
%IKM_RIGID This functions cacculates qdot from the platform velocity and the current configuration
%
% All calculations are computed in the world frame
%
global PM1 PM2 PM3 PM4 PM5 PM6 Phi_P Phi_1 Phi_4
%%
X=u(1:6);V=u(7:12);
q=u(13:30);
q_e=u(31:33);
qdot_e=u(34:36);

T0M=[ZXZ_to_Rot(X(4:6)) X(1:3);0 0 0 1];
R0M=T0M(1:3,1:3);

Deformation=[R0M*Phi_P(1:3,:)*q_e;R0M*Phi_P(4:6,:)*q_e;R0M*Phi_P(1:3,:)*qdot_e;R0M*Phi_P(4:6,:)*qdot_e];

end

