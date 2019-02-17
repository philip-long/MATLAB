clear all, clc

global MX3 MY3 MZ3 MX2 MY2 MZ2 MX1 MY1 MZ1
global XX3 XX2 YY3 YY2 ZZ3 ZZ2 XX1 YY1 ZZ1
global XY1 XY2 XY3 XZ1 XZ2 XZ3 YZ1 YZ2 YZ3 
global d1 d2 d3 M1 M2 M3
global IA1 IA2 IA3 G3

IA1=0.05; IA2=0.0; IA3=0.0;
M1=0.1; M2=0.2;M3=0.15;
MX3=0.5;MY3=0.0; MZ3=0.0;
MX2=0.0;MY2=0.0;MZ2=0.0;
MX1=0.0;MY1=0.0;MZ1=0.0; 
XX3=0.0; XX2=0.0;XX1=0.0; 
XY3=0.0; XY2=0.0;XY1=0.0;
XZ1=0.0;XZ2=0.0;XZ3=0.0; 
YY1=0.0;YY2=0.0;YY3=0.0;
YZ1=0.0;YZ2=0.0;YZ3=0.0; 
ZZ3=0.2;ZZ2=0.0; ZZ1=0.0;
G3=0.0;


links=3;
steps=[2,3,1];
d1=0.0;
d2=2.0;
d3=0.75;

q=[0;pi+0.1;0.0];


qdot=[0.1;0.5;0];
qddot=[0.1;0.2;0.3];
T_01=T01(q);T_02=T02(q);T_03=T03(q);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
V=J_03*qdot;


A=Inertia_matrix(q);


% Effective Mass investigations
Generalized_Momentum=A*qdot
Transform_to_Cartesian_Momentum=inv(J_03')*Generalized_Momentum

%M=inv(transpose(J_03))*A*inv(J_03);
M_inv=J_03*inv(A)*transpose(J_03)
%CartesianMomentum=M*V; OK


u=V/norm(V);
% This is Khatib's method
Effective_Mass_inverse=(u'*M_inv*u);
Effective_Mass=inv(Effective_Mass_inverse);
Khatib_momentum=Effective_Mass*norm(V)

V2=J_02*qdot(1:2);
u=V2/norm(V2);
M_inv=[J_02 , zeros(3,1)]*inv(A)*transpose([J_02 , zeros(3,1)])
Effective_Mass_inverse=(u'*M_inv*u);
Effective_Mass=inv(Effective_Mass_inverse);
Khatib_momentum=Effective_Mass*norm(V2)

% Sub matrixing does not do the same thing
M_inv=J_02*inv(A(1:2,1:2))*transpose(J_02)
Effective_Mass_inverse=(u'*M_inv*u);
Effective_Mass=inv(Effective_Mass_inverse);
Khatib_momentum=Effective_Mass*norm(V2)


% This is Strambolis method
% v_ih is a scalar quantity defined along the line of impact
% m_ri is configuration dependent mass along line of impact
% This can define the effective kinetic enegery
% we use inertia matrix of whole manipultor 
% and can note that in fact compliance may decouple the inertia
J_red=u'*J_03;
v_u=(J_red)*qdot


m_r=inv((J_red*inv(A)*J_red'))

% testing
J_red=u'*J_02;
m_r=inv(([J_red 0]*inv(A)*[J_red 0]'))


kinetic_energy_stramboli=0.5*m_r*v_u*v_u
momentum=m_r*v_u

%==============================
F=[1;0;0];
tau=transpose(J_03)*F





%=============================================%



