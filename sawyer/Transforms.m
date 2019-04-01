function [T]= Transforms(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


global r1 r2 r3 r4 r5 r6 r7 d2
q1=q(1);
q2=q(2)+pi/2;
q3=q(3);
q4=q(4);
th5=q(5);
th6=q(6);
th7=q(7);

T0T1=eye(4);
T0T2=eye(4);
T0T3=eye(4);
T0T4=eye(4);
T0T5=eye(4);
T0T6=eye(4);
T0T7=eye(4);

T0T1(1,1) =  cos(q1);
T0T1(2,1) =  sin(q1);
T0T1(3,1) =  0;
T0T1(1,2) =  -sin(q1);
T0T1(2,2) =  cos(q1);
T0T1(3,2) =  0;
T0T1(1,3) =  0;
T0T1(2,3) =  0;
T0T1(3,3) =  1;
T0T1(1,4) =  0;
T0T1(2,4) =  0;
T0T1(3,4) =  r1;

%Tramsformation matrix 0 T 7
T0T7(1,1) =  -((((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*cos(th6) + (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*sin(th6))*cos(th7) - (-((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5))*sin(th7);
T0T7(2,1) =  -((((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*cos(th6) + (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*sin(th6))*cos(th7) - (-((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5))*sin(th7);
T0T7(3,1) =  -(((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*cos(th6) + (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*sin(th6))*cos(th7) - (-(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5))*sin(th7);
T0T7(1,2) =  ((((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*cos(th6) + (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*sin(th6))*sin(th7) - (-((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5))*cos(th7);
T0T7(2,2) =  ((((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*cos(th6) + (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*sin(th6))*sin(th7) - (-((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5))*cos(th7);
T0T7(3,2) =  (((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*cos(th6) + (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*sin(th6))*sin(th7) - (-(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5))*cos(th7);
T0T7(1,3) =  (((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*sin(th6) - (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*cos(th6);
T0T7(2,3) =  (((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*sin(th6) - (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*cos(th6);
T0T7(3,3) =  ((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*sin(th6) - (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*cos(th6);
T0T7(1,4) =  d2*cos(q1) - r2*sin(q1) + r3*sin(q2)*cos(q1) + r4*(-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2)) + r5*((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) + sin(q2)*cos(q1)*cos(q4)) + r6*(-((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5)) + r7*((((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*sin(th6) - (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*cos(th6));
T0T7(2,4) =  d2*sin(q1) + r2*cos(q1) + r3*sin(q1)*sin(q2) + r4*(-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3)) + r5*((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*sin(q2)*cos(q4)) + r6*(-((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5)) + r7*((((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*sin(th6) - (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*cos(th6));
T0T7(3,4) =  r1 + r3*cos(q2) + r4*sin(q2)*sin(q3) + r5*(-sin(q2)*sin(q4)*cos(q3) + cos(q2)*cos(q4)) + r6*(-(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5)) + r7*(((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*sin(th6) - (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*cos(th6));

%Tramsformation matrix 0 T 6
T0T6(1,1) =  (((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*cos(th6) + (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*sin(th6);
T0T6(2,1) =  (((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*cos(th6) + (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*sin(th6);
T0T6(3,1) =  ((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*cos(th6) + (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*sin(th6);
T0T6(1,2) =  -(((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5))*sin(th6) + (-(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4))*cos(th6);
T0T6(2,2) =  -(((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5))*sin(th6) + (-(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4))*cos(th6);
T0T6(3,2) =  -((-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5))*sin(th6) + (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*cos(th6);
T0T6(1,3) =  -((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5);
T0T6(2,3) =  -((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5);
T0T6(3,3) =  -(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5);
T0T6(1,4) =  d2*cos(q1) - r2*sin(q1) + r3*sin(q2)*cos(q1) + r4*(-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2)) + r5*((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) + sin(q2)*cos(q1)*cos(q4)) + r6*(-((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5));
T0T6(2,4) =  d2*sin(q1) + r2*cos(q1) + r3*sin(q1)*sin(q2) + r4*(-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3)) + r5*((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*sin(q2)*cos(q4)) + r6*(-((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5));
T0T6(3,4) =  r1 + r3*cos(q2) + r4*sin(q2)*sin(q3) + r5*(-sin(q2)*sin(q4)*cos(q3) + cos(q2)*cos(q4)) + r6*(-(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5));

%Tramsformation matrix 0 T 5
T0T5(1,1) =  ((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*cos(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(th5);
T0T5(2,1) =  ((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*cos(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*sin(th5);
T0T5(3,1) =  (-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*cos(th5) + sin(q2)*sin(q3)*sin(th5);
T0T5(1,2) =  -((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1))*sin(th5) + (-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*cos(th5);
T0T5(2,2) =  -((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4))*sin(th5) + (-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3))*cos(th5);
T0T5(3,2) =  -(-sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2))*sin(th5) + sin(q2)*sin(q3)*cos(th5);
T0T5(1,3) =  (-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) + sin(q2)*cos(q1)*cos(q4);
T0T5(2,3) =  (sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*sin(q2)*cos(q4);
T0T5(3,3) =  -sin(q2)*sin(q4)*cos(q3) + cos(q2)*cos(q4);
T0T5(1,4) =  d2*cos(q1) - r2*sin(q1) + r3*sin(q2)*cos(q1) + r4*(-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2)) + r5*((-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) + sin(q2)*cos(q1)*cos(q4));
T0T5(2,4) =  d2*sin(q1) + r2*cos(q1) + r3*sin(q1)*sin(q2) + r4*(-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3)) + r5*((sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*sin(q2)*cos(q4));
T0T5(3,4) =  r1 + r3*cos(q2) + r4*sin(q2)*sin(q3) + r5*(-sin(q2)*sin(q4)*cos(q3) + cos(q2)*cos(q4));

%Tramsformation matrix 0 T 4
T0T4(1,1) =  (-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*cos(q4) - sin(q2)*sin(q4)*cos(q1);
T0T4(2,1) =  (sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q2)*sin(q4);
T0T4(3,1) =  -sin(q2)*cos(q3)*cos(q4) - sin(q4)*cos(q2);
T0T4(1,2) =  -(-sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3))*sin(q4) - sin(q2)*cos(q1)*cos(q4);
T0T4(2,2) =  -(sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*sin(q2)*cos(q4);
T0T4(3,2) =  sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4);
T0T4(1,3) =  -sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2);
T0T4(2,3) =  -sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3);
T0T4(3,3) =  sin(q2)*sin(q3);
T0T4(1,4) =  d2*cos(q1) - r2*sin(q1) + r3*sin(q2)*cos(q1) + r4*(-sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2));
T0T4(2,4) =  d2*sin(q1) + r2*cos(q1) + r3*sin(q1)*sin(q2) + r4*(-sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3));
T0T4(3,4) =  r1 + r3*cos(q2) + r4*sin(q2)*sin(q3);

%Tramsformation matrix 0 T 3
T0T3(1,1) =  -sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3);
T0T3(2,1) =  sin(q1)*cos(q2)*cos(q3) + sin(q3)*cos(q1);
T0T3(3,1) =  -sin(q2)*cos(q3);
T0T3(1,2) =  -sin(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2);
T0T3(2,2) =  -sin(q1)*sin(q3)*cos(q2) + cos(q1)*cos(q3);
T0T3(3,2) =  sin(q2)*sin(q3);
T0T3(1,3) =  sin(q2)*cos(q1);
T0T3(2,3) =  sin(q1)*sin(q2);
T0T3(3,3) =  cos(q2);
T0T3(1,4) =  d2*cos(q1) - r2*sin(q1) + r3*sin(q2)*cos(q1);
T0T3(2,4) =  d2*sin(q1) + r2*cos(q1) + r3*sin(q1)*sin(q2);
T0T3(3,4) =  r1 + r3*cos(q2);


T0T2(1,1) =  cos(q1)*cos(q2);
T0T2(2,1) =  sin(q1)*cos(q2);
T0T2(3,1) =  -sin(q2);
T0T2(1,2) =  -sin(q2)*cos(q1);
T0T2(2,2) =  -sin(q1)*sin(q2);
T0T2(3,2) =  -cos(q2);
T0T2(1,3) =  -sin(q1);
T0T2(2,3) =  cos(q1);
T0T2(3,3) =  0;
T0T2(1,4) =  d2*cos(q1) - r2*sin(q1);
T0T2(2,4) =  d2*sin(q1) + r2*cos(q1);
T0T2(3,4) =  r1;


T={T0T1,...
T0T2,...
T0T3,...
T0T4,...
T0T5,...
T0T6,...
T0T7};
end

