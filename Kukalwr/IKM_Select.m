function dq=IKM(u)


global qinit qupper qlower
% Copyright (c) 2012 Philip Long
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.
%Extract data from input
q=u(1:length(qinit)); % Joint positions
dX=u(length(qinit)+1:length(qinit)+6); %Cartesian Velocities
SelectDirections=u(length(qinit)+7:length(qinit)+12);%Selector matrix components
S=eye(6);
for i=1:6
    for j=1:6
        if i==j
        S(i,j)=SelectDirections(i);
      
        end
    end
end

%Find Jacobian at Tool Frame
% T7=T70(q);
 J7=J70(q);
% P=TnE(1:3,4); %Distance to tool frame
% L=T7(1:3,1:3)*P; %Change the reference frame to the world one  
% Lhat=skew(L); %Skew the matrix
% JE=[eye(3) -Lhat; zeros(3) eye(3) ]*J7; % Change the point of the Jacobian


%Calculate dq
dq=pinv(S*J7)*dX;
%dq=(pinv(J7)*dX)+P1*Z;


