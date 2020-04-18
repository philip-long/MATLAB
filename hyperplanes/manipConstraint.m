function [c,ceq] = manipConstraint(q)
%manipConstraint ensures the volume of the polytope is greater than a min
%volume

minimum_allowable_volume=0.200;
end_effector_offset=[0.55;0.0;0.0];

qdot_arm_min=-4;
    T=T70(q);
    J=J70(q); 
    S=screwTransform(T(1:3,1:3)*end_effector_offset);
    JE=S*J;
A=[eye(length(q)) ;-eye(length(q))];
B=[ones(length(q),1)*qdot_arm_max ;ones(length(q),1)*-qdot_arm_min];
Q_arm=Polyhedron(A,B);Q_arm.computeVRep;

V_arm= getCartesianPolytope(JE(1:3,:),Q_arm);
c=minimum_allowable_volume-V_arm.volume;
ceq=[]
end

