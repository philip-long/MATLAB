%% Define the parameters
Px = Td(1,4);
Py = Td(2,4);
Pz = Td(3,4);
sx = Td(1,1);
sy = Td(2,1);
sz = Td(3,1);
nx =Td(1,2);
ny = Td(2,2);
nz = Td(3,2);
ax = Td(1,3);
ay = Td(2,3);
az = Td(3,3);
R2 = r(2);
e = 1;



%R3: 2 solution depending and e1

c=-Px^2 - Py^2 - Pz^2 + R2^2;
R3 = (-c)^0.5*e

%t2: 2 solutions depending on R3
e=-1
SQ = (e*(-Pz^2 + R3^2)^0.5)/R3;
CQ = Pz/R3;
t2 = atan2( SQ , CQ )


%t1:  2 Solutions depending on R3

c=-(R3*sin(t2));
X=Px^2 + Py^2;
Y=-(c*Py) - Px*R2;
X1=-Px^2 - Py^2;
Y1=c*Px - Py*R2;
t1 = atan2(Y/X , Y1/X1 )


U1T311=cos(t1)*cos(t2);
U1T312=cos(t2)*sin(t1);
U1T331=cos(t1)*sin(t2);
U1T332=sin(t1)*sin(t2);
SNA11=sx*U1T311 + sy*U1T312 - sz*sin(t2);
SNA12=nx*U1T311 + ny*U1T312 - nz*sin(t2);
SNA13=ax*U1T311 + ay*U1T312 - az*sin(t2);
SNA21=sy*cos(t1) - sx*sin(t1);
SNA22=ny*cos(t1) - nx*sin(t1);
SNA23=ay*cos(t1) - ax*sin(t1);
SNA31=sx*U1T331 + sy*U1T332 + sz*cos(t2);
SNA32=nx*U1T331 + ny*U1T332 + nz*cos(t2);
SNA33=ax*U1T331 + ay*U1T332 + az*cos(t2);


%t4: 4 solution depending on t1 and SNA23 SNA13 definition
%Equation type 2


t4 = atan2(-SNA23,-SNA13)
 % t4 = atan2(SNA23,SNA13 ) 

%t5: 4 solution depending on t1 and t4
B10=-(SNA13*cos(t4)) - SNA23*sin(t4);
t5 = atan2(-B10 , SNA33 )

%t6: 4 solution depending on t1 and t4
B10=-(SNA21*cos(t4)) + SNA11*sin(t4);
B20=-(SNA22*cos(t4)) + SNA12*sin(t4);

t6 = atan2(-B10 , -B20 )