function T=getTransforms4DOF(q) 
%getTransforms Returns all Transforms for planar robot with mobile base 
q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
T_world_L0(1,1)=1.00000000000000;
T_world_L0(1,2)=0;
T_world_L0(1,3)=0;
T_world_L0(1,4)=0;
T_world_L0(2,1)=0;
T_world_L0(2,2)=1.00000000000000;
T_world_L0(2,3)=0;
T_world_L0(2,4)=1.0*q0;
T_world_L0(3,1)=0;
T_world_L0(3,2)=0;
T_world_L0(3,3)=1.00000000000000;
T_world_L0(3,4)=0;
T_world_L0(4,1)=0;
T_world_L0(4,2)=0;
T_world_L0(4,3)=0;
T_world_L0(4,4)=1.00000000000000;

T_world_L1(1,1)=1.00000000000000;
T_world_L1(1,2)=0;
T_world_L1(1,3)=0;
T_world_L1(1,4)=1.0*q1;
T_world_L1(2,1)=0;
T_world_L1(2,2)=1.00000000000000;
T_world_L1(2,3)=0;
T_world_L1(2,4)=1.0*q0;
T_world_L1(3,1)=0;
T_world_L1(3,2)=0;
T_world_L1(3,3)=1.00000000000000;
T_world_L1(3,4)=0;
T_world_L1(4,1)=0;
T_world_L1(4,2)=0;
T_world_L1(4,3)=0;
T_world_L1(4,4)=1.00000000000000;

T_world_L2(1,1)=1.0*cos(q2);
T_world_L2(1,2)=-1.0*sin(q2);
T_world_L2(1,3)=0;
T_world_L2(1,4)=1.0*q1;
T_world_L2(2,1)=1.0*sin(q2);
T_world_L2(2,2)=1.0*cos(q2);
T_world_L2(2,3)=0;
T_world_L2(2,4)=1.0*q0;
T_world_L2(3,1)=0;
T_world_L2(3,2)=0;
T_world_L2(3,3)=1.00000000000000;
T_world_L2(3,4)=0;
T_world_L2(4,1)=0;
T_world_L2(4,2)=0;
T_world_L2(4,3)=0;
T_world_L2(4,4)=1.00000000000000;

T_world_EE(1,1)=1.0*cos(q2 + q3);
T_world_EE(1,2)=-1.0*sin(q2 + q3);
T_world_EE(1,3)=0;
T_world_EE(1,4)=1.0*q1 + 0.8*cos(q2);
T_world_EE(2,1)=1.0*sin(q2 + q3);
T_world_EE(2,2)=1.0*cos(q2 + q3);
T_world_EE(2,3)=0;
T_world_EE(2,4)=1.0*q0 + 0.8*sin(q2);
T_world_EE(3,1)=0;
T_world_EE(3,2)=0;
T_world_EE(3,3)=1.00000000000000;
T_world_EE(3,4)=0;
T_world_EE(4,1)=0;
T_world_EE(4,2)=0;
T_world_EE(4,3)=0;
T_world_EE(4,4)=1.00000000000000;

T={T_world_L0,T_world_L1,T_world_L2,T_world_EE};
end