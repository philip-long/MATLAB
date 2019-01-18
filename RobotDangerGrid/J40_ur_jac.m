%Jacobian matrix for frame 4
%Projection frame 0, intermediate frame 4
%
%Geometric parameters
%j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
%1       0       0       1       0       0       0       0       th1     r1      
%2       1       0       1       0       0       pi/2    0       th2     0       
%3       2       0       1       0       0       0       -d3     th3     0.0     
%4       3       0       1       0       0       0       -d4     th4     r4      
%5       4       0       1       0       0       pi/2    0       th5     r5      
%6       5       0       1       0       0       -pi/2   0       th6     0       
%7       6       2       0       0       0       0       0       0       r7      
%
%Equations:


function J=J40_ur_jac(q)


global r1 r4 r7 r5 d4 d3
global XX XY XZ YZ YY ZZ M MX MY MZ G3


th1=q(1);
th2=q(2);
th3=q(3);

J(1,1) = d3*sin(th1)*cos(th2) + d4*sin(th1)*cos(th2 + th3) + r4*cos(th1);
J(2,1) = -d3*cos(th1)*cos(th2) - d4*cos(th1)*cos(th2 + th3) + r4*sin(th1);
J(3,1) = 0;
J(4,1) = 0;
J(5,1) = 0;
J(6,1) = 1;
J(1,2) = (d3*sin(th2) + d4*sin(th2 + th3))*cos(th1);
J(2,2) = (d3*sin(th2) + d4*sin(th2 + th3))*sin(th1);
J(3,2) = -d3*cos(th2) - d4*cos(th2 + th3);
J(4,2) = sin(th1);
J(5,2) = -cos(th1);
J(6,2) = 0;
J(1,3) = d4*sin(th2 + th3)*cos(th1);
J(2,3)= d4*sin(th1)*sin(th2 + th3);
J(3,3) = -d4*cos(th2 + th3);
J(4,3) = sin(th1);
J(5,3) = -cos(th1);
J(6,3) = 0;
endfunction


%J14 = 0;
%J24 = 0;
%J34 = 0;
%J44 = sin(th1);
%J54 = -cos(th1);
%J64 = 0;
%L11 = 0;
%L21 = 0;
%L31 = 0;
%L12 = 0;
%L22 = 0;
%L32 = 0;
%L13 = 0;
%L23 = 0;
%L33 = 0;
%*=*
