function [c,ceq] = manipConstraint(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

qdot_arm_max=2;
qdot_arm_min=-4;
    T=T70(q);
    J=J70(q); 
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
A=[eye(length(q)) ;-eye(length(q))];
B=[ones(length(q),1)*qdot_arm_max ;ones(length(q),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;

V_arm= getCartesianPolytope(JE(1:3,:),Q_arm);
c=0.200-V_arm.volume;
ceq=[]
end

