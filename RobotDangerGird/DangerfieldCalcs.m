
clear all, clc
global d1 d2 d3
links=3;
steps=[2,3,1];
d1=0.0;
d2=1.0;
d3=0.75;
q=[pi/4;pi/4;0];


qdot=rand(3,1);

T_01=T01(q);
T_02=T02(q);
T_03=T03(q);

J_01=J01(q);
J_02=J02(q);
J_03=J03(q);


T12=inv(T_01)*T_02;
P12=T12(1:3,4);
T23=(inv(T_02)*T_03);
P23=T23(1:3,4);


Discrete_Pos{1}= P12;
Discrete_Pos{2}= P23;
Discrete_Pos{3}= [0;0;0];

Positions{1}=T_01;
Positions{2}=T_02;
Positions{3}=T_03;

Jac{1}=J_01;
Jac{2}=J_02;
Jac{3}=J_03;

Xdim=-2:0.1:2;
Ydim=-2:0.1:2;

k1=1.0;
k2=1.0;
gamma=4.0;
V=[1;0;0]
qdot=inv(Jac{3})*V;
total_danger_field=zeros(length(Xdim),length(Ydim));

% I can calculate the danger field from here
jTp=eye(4);

for cell_i=1:length(Xdim)
  for cell_j=1:length(Ydim)
    cell_position=[Xdim(cell_i);Ydim(cell_j)];   
    static_danger_field_j=0.0;
    kinematic_danger_field_j=0.0;    
    for link_i=1:links      
      for step_link_j=1:steps(link_i);      
       jPp=(step_link_j-1)*(Discrete_Pos{link_i}/steps(link_i));     
       jTp(1:3,4)=jPp;
       Position_robot_point=Positions{link_i}*jTp;     
       L= Positions{link_i}(1:3,1:3)*(step_link_j-1)*(Discrete_Pos{link_i}/steps(link_i));
       Lhat=skew(L);    
       ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
       Screw2=ScrewReduced(:,[1,2,6]);
       J=Screw2*Jac{link_i}; % Change the point of the Jacobian
       J=J([1,2,6],:); 
       Velocity_robot_point=J*qdot(1:link_i);             
       r_diff=cell_position-Position_robot_point(1:2,4);       
       if(norm(r_diff)<0.01)
        norm_r_diff=0.01;
        cos_angle=0.0;
       else
        norm_r_diff=norm(r_diff);
        cos_angle=(r_diff'*Velocity_robot_point(1:2))/norm_r_diff*norm(Velocity_robot_point(1:2));
       endif     
       static_danger_field_j=static_danger_field_j+k1/norm_r_diff;   
       velocity_contribution=((k2*norm(Velocity_robot_point(1:2))*gamma*cos_angle))/(norm_r_diff*norm_r_diff);     
       kinematic_danger_field_j=kinematic_danger_field_j+velocity_contribution;                   
       
       if(abs(velocity_contribution)>10.0)       
        velocity_contribution
       endif
      endfor             
    endfor
    total_danger_field(cell_i,cell_j)=static_danger_field_j+kinematic_danger_field_j;    
  endfor
endfor


colormap('jet');   % set colormap
%imagesc([-1,1],[-1,1],total_danger_field)
imagesc(Xdim,Ydim,total_danger_field')
colorbar
hold on
plot_robot(T_01,T_02,T_03);
