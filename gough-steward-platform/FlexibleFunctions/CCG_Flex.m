function H = CCG_Flex(u)


global M MS_r b_k Beta_k lambda_ik I_re I_er I_ee m_ee
global ModesNo I_rr

q_e=u(1:3);
qdot_e=u(4:6);
omega_r=u(7:9);

sumbk=([0;0;0]);
sumbkdot=([0;0;0]);
for i=1:ModesNo
    sumbk=sumbk+(b_k{i}.*q_e(i));
    sumbkdot=sumbkdot+(b_k{i}.*qdot_e(i));
end

r1_fin=MS_r+sumbk;
R1_in=cross(omega_r,r1_fin);
F_in= cross(omega_r,R1_in)+cross(2*omega_r,sumbkdot)

R1_cin=[0;0;0];
R2_cin=[0;0;0];
for i=1:ModesNo
    R1_cin=R1_cin+(I_re{i}*omega_r*qdot_e(i));
    for j=1:ModesNo
       R2_cin=R2_cin+I_ee{i,j}*omega_r*q_e(i)*qdot_e(j);
    end
end

C_in=cross(omega_r,I_rr*omega_r)+2*R1_cin+2*R2_cin

Q=zeros(ModesNo,1);
for i=1:ModesNo
    R1_Qin=zeros(3,1);
    R2_Qin=zeros(3,1);
    for j=1:ModesNo
        R1_Qin=R1_Qin+(I_ee{i,j}*omega_r*q_e(j));
        R2_Qin=R2_Qin+lambda_ik{i,j}*qdot_e(j);
    end
    Q(i)=(transpose(omega_r)*(I_er{j}*omega_r))-(transpose(omega_r)*R1_Qin) + 2*(transpose(omega_r)*R2_Qin)
end


H=[F_in;C_in;Q]