function T=getTransforms(q)


q11=q(1);
q12=q(2);
T_world_L11(1,1)=1.0*cos(q11);
T_world_L11(1,2)=-1.0*sin(q11);
T_world_L11(1,3)=0;
T_world_L11(1,4)=0;
T_world_L11(2,1)=1.0*sin(q11);
T_world_L11(2,2)=1.0*cos(q11);
T_world_L11(2,3)=0;
T_world_L11(2,4)=0;
T_world_L11(3,1)=0;
T_world_L11(3,2)=0;
T_world_L11(3,3)=1.00000000000000;
T_world_L11(3,4)=0;
T_world_L11(4,1)=0;
T_world_L11(4,2)=0;
T_world_L11(4,3)=0;
T_world_L11(4,4)=1.00000000000000;

T_world_EE(1,1)=1.0*cos(q11 + q12);
T_world_EE(1,2)=-1.0*sin(q11 + q12);
T_world_EE(1,3)=0;
T_world_EE(1,4)=0.8*cos(q11);
T_world_EE(2,1)=1.0*sin(q11 + q12);
T_world_EE(2,2)=1.0*cos(q11 + q12);
T_world_EE(2,3)=0;
T_world_EE(2,4)=0.8*sin(q11);
T_world_EE(3,1)=0;
T_world_EE(3,2)=0;
T_world_EE(3,3)=1.00000000000000;
T_world_EE(3,4)=0;
T_world_EE(4,1)=0;
T_world_EE(4,2)=0;
T_world_EE(4,3)=0;
T_world_EE(4,4)=1.00000000000000;

T={T_world_L11,T_world_EE};
