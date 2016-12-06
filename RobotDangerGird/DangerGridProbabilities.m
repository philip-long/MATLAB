% Suppose a grid 8*8
clear all, clc,close all
global MX3 MY3 MZ3 MX2 MY2 MZ2 MX1 MY1 MZ1
global XX3 XX2 YY3 YY2 ZZ3 ZZ2 XX1 YY1 ZZ1
global XY1 XY2 XY3 XZ1 XZ2 XZ3 YZ1 YZ2 YZ3 
global d1 d2 d3 M1 M2 M3 links
global IA1 IA2 IA3 G3

steps=[3,4,1];
DeclareGlobals;
min_value=-2.0;
max_value=3.0;
resolution=0.2;
q=[-0.1;0.4;0.0];
qdot=[0.1;0.3;0];
qddot=[0.1;0.2;0.3];
jTp=eye(4);
k1=1.0;
k2=1.0;
gamma=4.0;

[Xdim,Ydim,total_danger_field]=create_grid (min_value, max_value,resolution);
[Px,Py]=get_coordinates(Xdim,Ydim,0.0,0.0);
[Discrete_Link_Points,Transforms,Jacobians ]=get_kinematic_information(q,qdot);


% Position 
prob_position_field=zeros(length(total_danger_field));
dmin=0.2;
dmax=2.0;
lambda=0.1;
%

for pos_x=min_value:0.1:max_value
  for pos_y=min_value:0.1:max_value
    
    dp=0; % probablity position
    
    static_danger_field_j=0.0;
    kinematic_danger_field_j=0.0;    
    
    % Get danger due to each part of the link
    for link_i=1:links      
      for step_link_j=1:steps(link_i);      
       jPp=(step_link_j-1)*(Discrete_Link_Points{link_i}/steps(link_i));     
       jTp(1:3,4)=jPp;
       Position_robot_point=Transforms{link_i}*jTp;     
             
       J=screw_transform_jacobian_matrix( Transforms{link_i}(1:3,1:3),
                                        (step_link_j-1)*(Discrete_Link_Points{link_i}/steps(link_i)),
                                        Jacobians{link_i});       
       Velocity_robot_point=J*qdot(1:link_i);             
       r_diff=[pos_x;pos_y]-Position_robot_point(1:2,4);       
       
       if(norm(r_diff)<0.01)
        norm_r_diff=0.01;
        cos_angle=0.0;
       else
        norm_r_diff=norm(r_diff);
        cos_angle=(r_diff'*Velocity_robot_point(1:2))/norm_r_diff*norm(Velocity_robot_point(1:2));
       endif     
       
       
       if(norm(r_diff)<dmin)
        d=dmin;
       elseif(norm(r_diff)>dmax)
        d=dmax;
       else
        d=norm(r_diff);
       endif              
       dpj=1/d^lambda;
       dpj=normalise_value(dpj,1/dmax^lambda,1/dmin^lambda);
       dp=max(dp,dpj);
       % Normalise
       
       
       
     %  static_danger_field_j=static_danger_field_j+k1/norm_r_diff;   
     %  velocity_contribution=((k2*norm(Velocity_robot_point(1:2))*gamma*cos_angle))/(norm_r_diff*norm_r_diff);     
     %  kinematic_danger_field_j=kinematic_danger_field_j+velocity_contribution;                          
       
       
      endfor     
    endfor

    
    [Px,Py]=get_coordinates(Xdim,Ydim,pos_x,pos_y);
    % total_danger_field(Px,Py)=static_danger_field_j+kinematic_danger_field_j;           
    prob_position_field(Px,Py)=dp;    
  endfor
endfor





plot_danger_field(Xdim,Ydim,prob_position_field);
% Plotting
plot_robot(Transforms{1},Transforms{2},Transforms{3});
grid on
%grid minor on