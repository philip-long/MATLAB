function [vol] = plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,c)
%plotPolyForJointValue Conveniece function for plotting
%   Given a joint configuration we plot the poltope using tbxmanager https://www.tbxmanager.com/#
end_effector_offset=[0.55;0.0;0.0];
T=T70(q);
J=J70(q); 
S=screwTransform(T(1:3,1:3)*end_effector_offset);
JE=S*J;
A=[eye(length(q)) ;-eye(length(q))];
B=[ones(length(q),1)*qdot_arm_max ;ones(length(q),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;
V_arm= getCartesianPolytope(JE(1:3,:),Q_arm);
vol=V_arm.volume;
V_arm.plot('color',c,'alpha',0.5);
hold on
end

