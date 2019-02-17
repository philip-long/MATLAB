function J=getJacobians(q)
%getJacobians Returns all Jacobians for planar robot
q11=q(1);
q12=q(2);
J_world_L11(1,1)=0;
J_world_L11(2,1)=0;
J_world_L11(3,1)=0;
J_world_L11(4,1)=0;
J_world_L11(5,1)=0;
J_world_L11(6,1)=1.00000000000000;

J_world_EE(1,1)=-0.8*sin(q11);
J_world_EE(1,2)=0;
J_world_EE(2,1)=0.8*cos(q11);
J_world_EE(2,2)=0;
J_world_EE(3,1)=0;
J_world_EE(3,2)=0;
J_world_EE(4,1)=0;
J_world_EE(4,2)=0;
J_world_EE(5,1)=0;
J_world_EE(5,2)=0;
J_world_EE(6,1)=1.00000000000000;
J_world_EE(6,2)=1.00000000000000;


J={J_world_L11,J_world_EE};
