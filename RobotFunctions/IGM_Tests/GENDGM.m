%========JACOBIAN FOR GENERAL SERIAL ROBOT==========
function [T]=GENDGM(q)


global alpha d theta r sigma Joints
n=Joints;
%n is number of frames

T=cell([n 1]);
T0n=eye(4);
for j=1:n
    if sigma(j)==1
        rr(j)=q(j)+r(j);
        t(j)=theta(j);
    else
        t(j)=q(j)+theta(j);
        rr(j)=r(j);
    end
    Tj=[cos(t(j)) -sin(t(j)) 0 d(j)
        cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
        sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
        0 0 0 1];
    T0n=T0n*Tj;
    T{j}=T0n;
end
%T0Tn=T06;  

