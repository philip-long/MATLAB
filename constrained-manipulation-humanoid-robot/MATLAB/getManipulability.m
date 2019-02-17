function [ wp ] = getManipulability( r2,q3,qnames,RobotChain )
%getManipulability get volume of standard manipulability polytope


Jj=geometricJacobian(r2,q3,'leftPalm'); % 
Tj=getTransform(r2,q3,'leftPalm');
ind=getIndices(qnames, RobotChain.LEFT_ARM );
J=[Jj(4:6,ind);Jj(1:3,ind)];
Lhat=skew(Tj(1:3,1:3)*RobotChain.Tne(1:3,4));
eSn=[eye(3) -Lhat; zeros(3) eye(3) ];
Jjj=eSn*J;

A=[eye(7);-eye(7)];
B=[RobotChain.LEFT_ARM_MAX_VELOCITY_LIMITS;-RobotChain.LEFT_ARM_MIN_VELOCITY_LIMITS];
Q=Polyhedron(A,B);
V=getCartesianPolytope(Jjj,Q);
wp=V.volume;
end

