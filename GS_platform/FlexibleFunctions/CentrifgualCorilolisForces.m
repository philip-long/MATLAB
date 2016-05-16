%% Now I have to calculate the resultant forces for the centrigual and Coriolis stuff
% This could be difficult Double check this and 

% For a given 
omega_r=rand(3,1);
qdot_e=rand(3,1);
q_e=rand(3,1);
% First from Boyer then try to reconcile it with adams
sumbk=([0;0;0]);
sumbkdot=([0;0;0]);
for i=1:ModesNo
    sumbk=sumbk+(b_k{i}.*q_e(i));
    sumbkdot=sumbkdot+(b_k{i}.*qdot_e(i));
end

r1_fin=MS_r+sumbk;
R1_in=cross(omega_r,r1_fin);
F_in= cross(omega_r,R1_in)+cross(2*omega_r,sumbkdot);

% the torque compnent 

R1_cin=[0;0;0];
R2_cin=[0;0;0];
for i=1:ModesNo
    R1_cin=R1_cin+(I_8{i}*omega_r*qdot_e(i));
    for j=1:ModesNo
       R2_cin=R2_cin+I_9{i,j}*omega_r*q_e(i)*qdot_e(k);
    end
end

C_in=cross(omega_r,I_rr*omega_r)+2*R1_cin+2*R2_cin;

% Elasticised Forces

Q=zeros(3,ModesNo);
for i=1:ModesNo
    R1_Qin=zeros(3,1);
    R2_Qin=zeros(3,1);
    for j=1:ModesNo
        R1_Qin=R1_Qin+(I_9{i,j}*omega_r*q_e(j));
        R2_Qin=R2_Qin+lambda_ik{i,j}*qdot_e(j);
    end
    Q(:,i)=(transpose(omega_r)*(I_er{j}*omega_r))-(transpose(omega_r)*R1_Qin) + 2*(transpose(omega_r)*R2_Qin
end

%% Adams Equivalenece in Boyer noation
% 
% Its quite complicated so lets just compute the following elements for FIN
% Mdot11 Mdot12 Mdot12
% dM11dx dM12dx dM11dx

Mdot11=0;
% M12=int(r*dm)+int(Phi)dm*qe
% M12dot=wdot X MS_r + wdot*b*qe + b*qedot
% M13= int(Phi)dm

