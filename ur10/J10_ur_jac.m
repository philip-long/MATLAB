%Jacobian matrix for frame 1
%Projection frame 0, intermediate frame 1
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
%Equations


function J=J10_ur_jac(q)

global r1 r4 r7 r5 d4 d3
global XX XY XZ YZ YY ZZ M MX MY MZ G3

J(1,1) = 0;
J(2,1) = 0;
J(3,1) = 0;
J(4,1) = 0;
J(5,1) = 0;
J(6,1) = 1;
endfunction
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
