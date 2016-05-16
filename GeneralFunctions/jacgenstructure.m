function [Jack Qdot]=jacgenstructure(antc,mu,gamma,b,sigma,alpha,d,theta,r,q,k)
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
TiTj=eye(4);

j=k;
Counter=1;
while j~=0
     J_ord(Counter)=j;
    j=antc(j);
    Counter=Counter+1;
end%preprocessiong J
 J_ord=fliplr( J_ord);

Counter=1;
while j~=k
    
    j= J_ord(Counter);
    
    if sigma(j)==0
        theta(j)=q(j);
    elseif sigma(j)==1
        r(j)=q(j);
    end
    
    ajTj=[cos(gamma(j))*cos(theta(j))-sin(gamma(j))*cos(alpha(j))*sin(theta(j)), -cos(gamma(j))*sin(theta(j))-sin(gamma(j))*cos(alpha(j))*cos(theta(j)), sin(gamma(j))*sin(alpha(j)),d(j)*cos(gamma(j))+r(j)*sin(gamma(j))*sin(alpha(j));
        sin(gamma(j))*cos(theta(j))+cos(gamma(j))*cos(alpha(j))*sin(theta(j)), -sin(gamma(j))*sin(theta(j))+cos(gamma(j))*cos(alpha(j))*cos(theta(j)), -cos(gamma(j))*sin(alpha(j)),d(j)*sin(gamma(j))-r(j)*cos(gamma(j))*sin(alpha(j));
        sin(alpha(j))*sin(theta(j)),      sin(alpha(j))*cos(theta(j)),cos(alpha(j)),r(j)*cos(alpha(j))+b(j);
        0 , 0 , 0, 1];
    
    TiTj=TiTj*ajTj;
           
    p=TiTj(1:3,4);
    a=TiTj(1:3,3);
    
    AJ(:,Counter)=a;
    PJ(:,Counter)=p;
    
    jmax=Counter;
    
    Counter=Counter+1;
end

j=0;
Counter=1;

while j~=k
    
        j= J_ord(Counter);
    if sigma(j)~=2
        askew=[0        -AJ(3,Counter) AJ(2,Counter)
            AJ(3,Counter)   0        -AJ(1,Counter)
            -AJ(2,Counter)  AJ(1,Counter)       0];

        J0n(:,Counter)=[sigma(j)*AJ(:,Counter)+(1-sigma(j))*askew*(PJ(:,jmax)-PJ(:,Counter))
                                    (1-sigma(j))*AJ(:,Counter)];
        
        Qdot(Counter)=q(j);
        
        Counter=Counter+1;
                
    end
end

Jack=J0n
Qdot=transpose(Qdot)






















