function dq=IKM(u)


global qinit
%Extract data from input
q=u(1:length(qinit));
dX=u(length(qinit)+1:length(qinit)+6);
 J7=J70(q);
try % If we have some secondary criteria

Z=u(length(qinit)+7:length(qinit)+13);
P1=eye(7)-(pinv(J7)*J7);
 dq=(pinv(J7)*dX)+P1*Z;
catch % If there is no secondary criteria do the follow
   
   dq=pinv(J7)*dX;     
 
end

%Find Jacobian at Tool Frame
% T7=T70(q);


%%  Maximum distance from Joint limits
%======================================================================
% qavg=0.5*(qupper+qlower);
% Delta_q=(qupper-qlower);
% Gain=-0.5;
% Z=zeros(length(qinit),1);
% for i=1:length(qinit)
%     
%     Num=anglediff(q(i),qinitB(i));
%    % Num=(Gain*(q(i)-qavg(i)));
%     %denom=(Delta_q(i)^2);   
% %     if denom==0
% %         denom=0.00001;
% %     end
% 
%     Z(i,:)=Num;%/denom;
% end
% 
% Manipulability=det(J7*transpose(J7));
% if rank(J7)<6
%     disp 'In singular Configuration'
% end

%Calculate dq

%dq=pinv(J7)*dX;




