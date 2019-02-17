%========JACOBIAN FOR GENERAL SERIAL ROBOT==========
function [T]=GENJAC(sigma,alpha,d,theta,r,q,n)

%n is number of frames

T=cell([5 1]);
T0n=eye(4);
for j=1:n
    if sigma(j)==1
        r(j)=q(j)+r(j);
        theta=theta(j);
    else
        theta(j)=q(j)+theta(j);
        r(j)=r(j);
    end
    Tj=[cos(theta(j)) -sin(theta(j)) 0 d(j)
        cos(alpha(j))*sin(theta(j)) cos(alpha(j))*cos(theta(j)) -sin(alpha(j)) -r(j)*sin(alpha(j))
        sin(alpha(j))*sin(theta(j)) sin(alpha(j))*cos(theta(j)) cos(alpha(j)) r(j)*cos(alpha(j))
        0 0 0 1];
    T0n=T0n*Tj;
    T{j}=T0n;
end
%T0Tn=T06;  

