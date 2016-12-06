InitParams;

q=rand(6,1);
A=A_ur10(q);
A=A(1:6,1:6);
J=J10_ur_jac(q);
J4=J40_ur_jac(q);

Minv=[J zeros(6,5)] * inv(A) * [J zeros(6,5)]'



P=[0.0;0.0];
Pc=[-1.0;0.2];
Velocity_robot_point=[0.0;0.33];
RDIFF=[];
DANGER=[];
 plot(P(1),P(2),'bo',"MarkerSize",5.0,"LineWidth",5.0)
    hold on
    max_speed=0.25
    step=0.1
for i=1:1:20
  Pc(1)=Pc(1)+step;
   r_diff=Pc-P;     
   norm_r_diff=norm(r_diff);
   u=r_diff/norm(r_diff);
   Minv1=[J(1:2,:) zeros(2,5)] * inv(A) * [J(1:2,:) zeros(2,5)]';
   Minv1=[J4(1:2,:) zeros(2,3)] * inv(A) * [J4(1:2,:) zeros(2,3)]';
   m_effe_inv=u'*Minv1*u;
   
    if(m_effe_inv<0.0001)
   DangerInertia=0.0
   else
   DangerInertia=(1.0/m_effe_inv);
    endif
    

  
   
   RDIFF=[RDIFF r_diff];
   DANGER=[DANGER DangerInertia];
   plot(r_diff(1),r_diff(2),'rx',"MarkerSize",5.0,"LineWidth",5.0)

endfor

%DANGERp= ( DANGER-min(DANGER) ) ./  (max_speed-min(DANGER) ) ;

figure(2)
plot((-1.0+step):step:Pc(1),DANGER)
























