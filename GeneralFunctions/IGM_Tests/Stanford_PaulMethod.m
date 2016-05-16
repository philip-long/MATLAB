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




%t1:  2 Solutions e=1 or -1 
e = -1;
SQ = (-(Px*R2) + e*Py*(Px^2 + Py^2 - R2^2)^0.5)/(Px^2 + Py^2);
CQ = (Py*R2 + e*Px*(Px^2 + Py^2 - R2^2)^0.5)/(Px^2 + Py^2);
t1 = atan2( SQ , CQ )

%t2: 2 solutions depending on t1
Y=-(Px*cos(t1)) - Py*sin(t1);

%R3: 4 solution depending on t1 and e
t2 = atan2(-Y , Pz )
e=1;
R3 = e * (Pz^2 + Y^2)^0.5



%t4: 4 solution depending on t1 and X Y definition
%Equation type 2
X=ay*cos(t1) - ax*sin(t1);
Y=-(ax*cos(t1)*cos(t2)) - ay*cos(t2)*sin(t1) + az*sin(t2);


t4 = atan2(-X,Y )
%t4 = atan2(X,-Y )

%t5: 4 solution depending on t1 and t4
Y=-(cos(t4)*(ax*cos(t1)*cos(t2) + ay*cos(t2)*sin(t1) - az*sin(t2))) - (ay*cos(t1) - ax*sin(t1))*sin(t4);
Y1=-(az*cos(t2)) - ax*cos(t1)*sin(t2) - ay*sin(t1)*sin(t2);

%Solution
t5 = atan2(-Y , -Y1 )



%t6: 4 solution depending on t1 and t4
Y=-(cos(t4)*(sy*cos(t1) - sx*sin(t1))) + (sx*cos(t1)*cos(t2) + sy*cos(t2)*sin(t1) - sz*sin(t2))*sin(t4);
Y1=-(cos(t4)*(ny*cos(t1) - nx*sin(t1))) + (nx*cos(t1)*cos(t2) + ny*cos(t2)*sin(t1) - nz*sin(t2))*sin(t4);

%Solution
t6 = atan2(-Y , -Y1 )