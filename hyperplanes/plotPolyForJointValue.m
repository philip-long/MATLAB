function [h] = plotPolyForJointValue(q,qdot_arm_max,qdot_arm_min,c)
%plotPolyForJointValue Conveniece function for plotting
%   Detailed explanation goes here

T=T70(q);
J=J70(q); 
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;
A=[eye(length(q)) ;-eye(length(q))];
B=[ones(length(q),1)*qdot_arm_max ;ones(length(q),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;
V_arm= getCartesianPolytope(JE(1:3,:),Q_arm);
disp 'volume', V_arm.volume
h=V_arm.plot('color',c,'alpha',0.5);
hold on
end

