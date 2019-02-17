% This script is just checking linearly dependance of lines that intersect
% at points to help me understand the actuatioon singualrity condition of
% the lower mobility cooperative manipulator NAO

clear all
clc

syms x1 y1 z1 x2 y2 z2 
P1=[x1; y1 ;z1];P2=[x2 ;y2 ;z2];P3=3*[x2 ;y2 ;z2];
P1=[0; 0 ;0];P2=[1 ;0.5;0];P3=[4 ;2 ;0];
%ARM ONE directions 2 actuated and 1 constraint
va1=[0.7071; 0 ;0.7071]; %Parallel to s2 therefore no y componet
%va1=[0.5789 ; 0.5769  ; 0.5763]; %A random axis
va2=[0 ;1; 0]; % Parallel to s1 therefore no x z componet
%va2=[0.4717 ;   0.8601 ;   0.1942];%A random axis
vc1=[0.1646   ; 0.8232 ;   0.5433];  % I think I can make this any Direction a random axis


%ARM TWO directions 2 actuated and 1 constraint
va6=[0.5; 0; 0.8660]; %Parallel to s7 therefore no y componet
%va6=[0.0283 ;0.6724; 0.7396];%A random axis
va7=[0 ;1; 0];% Parallel to s1 therefore no x z componet
%va7=[0 ;cos(pi/12); sin(pi/12)];%A random axis
vc2=[0.9683 ;   0.2033  ;  0.1452];%A random axis

%P1=rand(3,1);
%P2=rand(3,1);

% Vector Construction Pure forces (Lines)
VA1=[va1;cross(P1,va1)]; %Pure force passing through point P1 with direction va1
VA2=[va2;cross(P1,va2)];
VC1=[vc1;cross(P1,vc1)];
VA6=[va6;cross(P2,va6)];
VA7=[va7;cross(P2,va7)];
VC2=[vc2;cross(P3,vc2)];

T=[VA1 VA2 VC1 VA6 VA7 VC2]; % Global Wrenchg
rank(T)
TA=[VA1 VA2 VA6 VA7];% Actuation wrench
rank(TA)
TC=[VC1 VC2];%Constraint Wrench
rank(TC)


% Conclusion the six $0 screws have rank 5 not because of any "parallelity" 
% but because they all intersect a common line
% Proof: Still degenerates if rand numbers are substituted for the parallel
% vectors
%va7=[0 ;cos(pi/12); sin(pi/12)];
%va6=[0.0283 ;0.6724; 0.7396];


%% Planar example with moments 
f1=[1;0;0]; P1=[ 1 ; 2;0];
f2=[0;1;0]; P2=[ 1 ; 2;0];
M1=[0;0;1];

S1=[f1;cross(f1,P1)];
S2=[f2;cross(f2,P2)];
S3=[0;0;0;M1]

S=[S1 S2 S3]
rank(S)
S=[S1 S2 ]
rank(S)

% Conlusion if two forces are parallel and the plane created by them
% contains the moment their linearly dependent

%% 6 DOF example
clc
f1=[1;0;0]; P1=[ 1 ; 1;1];
f2=[1;0;0]; P2=[ 1 ;2;2];
M=[0;-1;1];


S1=[f1;cross(f1,P1)];
S2=[f2;cross(f2,P2)];
S3=[0;0;0;M];

S=[S1 S2 S3]
rank(S)

%% Finding Reciporcy to two non parallel non intersecting screws
clear all
clc
f1=[1;0;0]; P1=[1 ; 0; 0]; %Two non parallel non intersecting forces
f2=[0;1;0]; P2=[ 0;1;1];


plot3([P1(1);P1(1)-f1(1)],[P1(2);P1(2)-f1(2)],[P1(3);P1(3)-f1(3)],'--b','LineWidth',2)% f1
hold on
plot3([P1(1);P1(1)+f1(1)],[P1(2);P1(2)+f1(2)],[P1(3);P1(3)+f1(3)],'--g','LineWidth',2)% f1

plot3([P2(1);P2(1)-f2(1)],[P2(2);P2(2)-f2(2)],[P2(3);P2(3)-f2(3)],'--r','LineWidth',2)% f1
plot3([P2(1);P2(1)+f2(1)],[P2(2);P2(2)+f2(2)],[P2(3);P2(3)+f2(3)],'--k','LineWidth',2)% f1



S1=[f1;cross(f1,P1)]; %Screws of said forces
S2=[f2;cross(f2,P2)];
M1=[zeros(3) eye(3)
    eye(3) zeros(3)];



S=[S1 S2];
for i=1:2% Check forces
sr=S(1:3,i);

srnorm=sqrt(sum((sr.^2)));

srN=sr/srnorm;

transpose(srN)*S(4:6,i)%Check if its pure rotation if zero then lambda=0 and its pure force


end


A=(M1*S)';
T=null(A); % Twists reciprocal to said forces
A*T
% Check if twists are pure rotation
for i=1:4
sr=T(1:3,i);

srnorm=sqrt(sum((sr.^2)));

srN=sr/srnorm;

transpose(srN)*T(4:6,i)%Check if its pure rotation if zero then lambda=0 and its pure force
pause()

end





