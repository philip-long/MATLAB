run SymScrew
%%


%% Preparation
syms x y z
Pt=[x; y ;z]

Sleft=AxisOfJoints;%Screw axes
Pleft=VectorToOrigin;%Points on said axes

n2=cross(Sleft{3},Sleft{4});% Normal to P1ane 2
P2=Pleft{3};% Point on Plane 2
r2=transpose(n2)*(Pt-P2);%Equation of plane 2

n1=cross(Sleft{1},Sleft{2});%Normal to Plane 1
P1=Pleft{1};% Point on Plane 1
r1=transpose(n1)*(Pt-P1);%Equation of plane 2


%% Find Equation of Line intersecting both planes

%Normal to the normal of both planes, hence direction vector to desired line
n3=cross(n1,n2);

% %Find P3=intersection of r1 r2
x=solve(r1,x); %Solve x in terms of z

y=0;%y can be anything so choose 0

%Substitute and solve for z
r2n=subs(r2,x);r2n=subs(r2n,y);z=solve(r2n,z); 
%Finally sub z back in
x=subs(x)

P3=simplify([x;y;z])% A point on the line is given as P3
% %Eqn of Line=P3+k*n3



%% Checks - pick anyvalue of k and check if point lies on r1 and r2
k=10
AnyPoint=P3+k*n3
x=AnyPoint(1);
y=AnyPoint(2);
z=AnyPoint(3);
subs(r1)% =0 
subs(r2)% =0 so for any k resulting point falls on line of intersection


%% Find values of joints at actuation singularity

% The line is equal to point P3 which lies on it and n3 its direction
%multiplied by any scalar
L=P3+1*n3

% Pr is the location of the point on right that needs to intersect
% L for an actuation singularity

Pr=Pright{3}
EE=simplify(Pr-P3)

q2=0
subs(n3)
subs(EE)















