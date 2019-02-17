function tau=IFM(u)


global qinit

%Extract data from input
q=u(1:length(qinit));
dF=u(length(qinit)+1:end);

%Find Jacobian at Tool Frame
% T7=T70(q);
 J7=J70(q);
% P=TnE(1:3,4); %Distance to tool frame
% L=T7(1:3,1:3)*P; %Change the reference frame to the world one  
% Lhat=skew(L); %Skew the matrix
% JE=[eye(3) -Lhat; zeros(3) eye(3) ]*J7; % Change the point of the Jacobian

tau=transpose(J7)*dF;


