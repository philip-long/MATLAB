function dq=IVM(u)


global CameraParameters
fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);

VP1=[u(1:3)]; %Pt 1 in homogenous format
VP2=[u(4:6)];%Pt2  in homogenous format
q=u(7:13);% Vision Robot
ds=u(14:17); 
Ls=Lsegfrompts([VP1;VP2;q]);
T0C1=T70_C(q);
T0C2=[T0C1(1:3,1:3)  zeros(3); zeros(3) T0C1(1:3,1:3)];
dq=pinv(Ls*inv(T0C2)*J70_C(q))*ds;




