Td =[0.4896   -0.6669    0.5617    0.2638
   -0.1671    0.5606    0.8111    0.6322
   -0.8558   -0.4910    0.1631   -0.0549
         0         0         0    1.0000];
     
Q=Kuka_6ax_simple_pieper(Td)
%T=GENDGM(Q{1})
%ans{6}

qr=[Q{1}(1:2) te Q{1}(3:end)];
% qr=[qc(1:2) te qc(3:end)];
J=J70_simple(qr)
T=T70_simple(qr)

% Jacobian decomposition stuff from Christine
Jm=[J(:,1:2) J(:,4:end)];
C1=inv(Jm)*J(:,3);
C=[eye(6) C1];
Cplus=[eye(6);zeros(1,6)]-([C1;-eye(1)]*inv(1+C1'*C1)*C1')





% q=[Q{1}(1:2) te Q{1}(3:end)]'+(eye(7)-pinv(J)*J)*rand(7,1)
% q=pinv(Cplus)*[Q{1}(1:2) Q{1}(3:end)]'
% Tnew=GENDGM(q)
% Tnew{6}


% qm=qp-C1*qc
qm=Q{1}'-(C1*rand(1)*10)
GENDGM(qm)
ans{6}