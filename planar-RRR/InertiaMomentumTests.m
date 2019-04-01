clear all, clc, close all

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

q=[0;pi;0.0];


qdot=[0;0.0;0];

T_01=T01(q);T_02=T02(q);T_03=T03(q);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
V=J_03*qdot;


plot_robot(T_01,T_02,T_03);

q=[0;pi-0.1;0];
qdot=[0.1;0.0;0];
qddot=[0.1;0.2;0.3];
A=inertiaMatrix(q);
H=CCG(q,qdot);
Tau=A*qddot;

q=[0;0.1;0];
A=inertiaMatrix(q);
H=CCG(q,qdot);
Tau2=A*qddot;
%
%% Posture investigations to see if this makes sense 
%m_inv=[]
%for i=1:10
%step=i;
%steps=10;
%q=[0;pi-0.1;0];
%qdot=[0.1;0.0;0];
%qddot=[0.1;0.2;0.3];
%T_01=T01(q);T_02=T02(q);T_03=T03(q);
%T12=inv(T_01)*T_02;
%P12=T12(1:3,4);
%J_01=J01(q);J_02=J02(q);J_03=J03(q);
%A=Inertia_matrix(q);
%H=Centrefugal(q,qdot);
%Tau=A*qddot;
%
%
%
%jPp=(step-1)*(P12/steps);     
%jTp=eye(4);
%jTp(1:3,4)=jPp;
%T_0P=T_01*jTp;
%L= T_01(1:3,1:3)*(step-1)*(P12/steps);
%Lhat=skew(L);    
%ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
%Screw2=ScrewReduced(:,[1,2,6]);
%J=Screw2*J_01; % Change the point of the Jacobian
%J=J([1,2,6],:);   
%V=J*qdot(1);
%u=V/norm(V);
%
%% Now I need to transform the jaocbian 
%T_P3=inv(T_0P)*T_03;
%L= T_0P(1:3,1:3)*T_P3(1:3,4);
%Lhat=skew(L);    
%ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
%Screw2=ScrewReduced(:,[1,2,6]);
%J_03_p=Screw2*J_03; % Change the point of the Jacobian
%J_03_p=J_03_p([1,2,6],:); 
%M_inv=J_03_p*inv(A)*transpose(J_03_p);
%Effective_Mass_inverse=(u'*M_inv*u);
%Effective_Mass=inv(Effective_Mass_inverse);
%m_inv=[m_inv Effective_Mass];
%endfor
%plot_robot(T_01,T_02,T_03);
%figure(2)
%plot(1:10,m_inv,"rx","MarkerSize",5.0)


% Posture investigations to see if this makes sense 
m_inv=[]

for i=1:10
step=i;
steps=10;
q=[0;-0.1;0];
qdot=[0.1;0.0;0];
qddot=[0.1;0.2;0.3];
T_01=T01(q);T_02=T02(q);T_03=T03(q);
T12=inv(T_01)*T_02;
P12=T12(1:3,4);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
A=inertiaMatrix(q);




jPp=(step-1)*(P12/steps);     
jTp=eye(4);
jTp(1:3,4)=jPp;
T_0P=T_01*jTp;
L= T_01(1:3,1:3)*(step-1)*(P12/steps);
Lhat=skew(L);    
ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
Screw2=ScrewReduced(:,[1,2,6]);
J=Screw2*J_01; % Change the point of the Jacobian
J=J([1,2,6],:);   
V=J*qdot(1);
if(norm(V(1:2))==0)
  m_inv=[m_inv 0.0];
else
  nv=norm(V(1:2));
  u=(V(1:2)/nv);
M_inv=J(1:2)*inv(A(1))*transpose(J(1:2));
Effective_Mass_inverse=(u'*M_inv*u);
Effective_Mass=inv(Effective_Mass_inverse);
m_inv=[m_inv Effective_Mass];
J_red=u'*J(1:2);
m_r=inv((J_red*inv(A(1))*J_red'))
end


end
plot_robot(T_01,T_02,T_03);
figure(2)
plot(1:10,m_inv,"rx","MarkerSize",5.0)





% Posture investigations to see if this makes sense 
m_inv=[]

for i=1:10
step=i;
steps=10;
q=[0;-0.1;0];
qdot=[0.1;0.0;0];
qddot=[0.1;0.2;0.3];
T_01=T01(q);T_02=T02(q);T_03=T03(q);
T23=inv(T_02)*T_03;
P23=T23(1:3,4);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
A=inertiaMatrix(q);

Tij=T_02;
Pij=P23;
Jij=J_02;

jPp=(step-1)*(Pij/steps);     
jTp=eye(4);
jTp(1:3,4)=jPp;
T_0P=T_01*jTp;
L= Tij(1:3,1:3)*(step-1)*(Pij/steps);
Lhat=skew(L);    
ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
Screw2=ScrewReduced(:,[1,2,6]);
J=Screw2*Jij; % Change the point of the Jacobian
J=J([1,2,6],:);   
V=J*qdot(1:2);

if(norm(V(1:2))==0)
  m_inv=[m_inv 0.0];
else
  nv=norm(V(1:2));
  u=(V(1:2)/nv);
M_inv=J(1:2,:)*inv(A(1:2,1:2))*transpose(J(1:2,:));
Effective_Mass_inverse=(u'*M_inv*u);
Effective_Mass=inv(Effective_Mass_inverse);
m_inv=[m_inv Effective_Mass];
J_red=u'*J(1:2,:);
m_r=inv((J_red*inv(A(1:2,1:2))*J_red'))
end


end
plot_robot(T_01,T_02,T_03);
figure(2)
plot(1:10,m_inv,"rx","MarkerSize",5.0)