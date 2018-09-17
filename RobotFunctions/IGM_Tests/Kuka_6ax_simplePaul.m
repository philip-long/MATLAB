%% Define the parameters
global alpha
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
te=alpha(3);









%t1:  2 Solutions e=1 or -1 
% Equating T124 = -0.39*Cos[t3]*Sin[te]
% and      U124 = Py*Cos[t1] - Px*Sin[t1]


e = -1;
X=-Px;
Y=Py;
Z=-0.39*cos(t3)*sin(te);


SQ = ((X*Z) + e*Y*(X^2 + Y^2 - Z^2)^0.5)/(X^2 + Y^2);
CQ = (Y*Z - e*X*(X^2 + Y^2 - Z^2)^0.5)/(X^2 + Y^2);
t1 = atan2( SQ , CQ );

