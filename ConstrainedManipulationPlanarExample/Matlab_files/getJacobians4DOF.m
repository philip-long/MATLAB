function J=getJacobians4DOF(q) 
%getJacobians Returns all Jacobians for planar robot with mobile base
 
q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
J_world_L0(1,1)=0;
J_world_L0(2,1)=1.00000000000000;
J_world_L0(3,1)=0;
J_world_L0(4,1)=0;
J_world_L0(5,1)=0;
J_world_L0(6,1)=0;

J_world_L1(1,1)=0;
J_world_L1(1,2)=1.00000000000000;
J_world_L1(2,1)=1.00000000000000;
J_world_L1(2,2)=0;
J_world_L1(3,1)=0;
J_world_L1(3,2)=0;
J_world_L1(4,1)=0;
J_world_L1(4,2)=0;
J_world_L1(5,1)=0;
J_world_L1(5,2)=0;
J_world_L1(6,1)=0;
J_world_L1(6,2)=0;

J_world_L2(1,1)=0;
J_world_L2(1,2)=1.00000000000000;
J_world_L2(1,3)=0;
J_world_L2(2,1)=1.00000000000000;
J_world_L2(2,2)=0;
J_world_L2(2,3)=0;
J_world_L2(3,1)=0;
J_world_L2(3,2)=0;
J_world_L2(3,3)=0;
J_world_L2(4,1)=0;
J_world_L2(4,2)=0;
J_world_L2(4,3)=0;
J_world_L2(5,1)=0;
J_world_L2(5,2)=0;
J_world_L2(5,3)=0;
J_world_L2(6,1)=0;
J_world_L2(6,2)=0;
J_world_L2(6,3)=1.00000000000000;

J_world_EE(1,1)=0;
J_world_EE(1,2)=1.00000000000000;
J_world_EE(1,3)=-0.8*sin(q2);
J_world_EE(1,4)=0;
J_world_EE(2,1)=1.00000000000000;
J_world_EE(2,2)=0;
J_world_EE(2,3)=0.8*cos(q2);
J_world_EE(2,4)=0;
J_world_EE(3,1)=0;
J_world_EE(3,2)=0;
J_world_EE(3,3)=0;
J_world_EE(3,4)=0;
J_world_EE(4,1)=0;
J_world_EE(4,2)=0;
J_world_EE(4,3)=0;
J_world_EE(4,4)=0;
J_world_EE(5,1)=0;
J_world_EE(5,2)=0;
J_world_EE(5,3)=0;
J_world_EE(5,4)=0;
J_world_EE(6,1)=0;
J_world_EE(6,2)=0;
J_world_EE(6,3)=1.00000000000000;
J_world_EE(6,4)=1.00000000000000;

J={J_world_L0,J_world_L1,J_world_L2,J_world_EE};
end

