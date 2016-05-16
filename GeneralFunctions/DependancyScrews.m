% This script is used to invesitgate the linear dependancy of screws, as a
% sort of a check for my sanity while investigating actuation singualrities


% Using plucker lines to describe the screw lines, plucker lines are
% described by a direction vector $v$ and a point on the line $r$. 
% From euclideanspace.com:
% "Use the direction of the line and the vector direction from the origin to
% the nearest point on the line to the origin. A related way is to start with 
% an arbitrary point on the line and take its cross product with the direction vector. 
% This cross product is independent of the point chosen and uniquely defines the 
% line which is what we are looking for."


%I can use these to describe zero pitch screws rotations or
% forces
% $=(v ; r X v)
clear all,clc

%%          Objective 1: Show that if six lines are intersected by a
%%          single line the rank of the system is five

% Choose two points ( always an intersecting line)
P1=rand(3,1)*10;
P2=rand(3,1)*10;
P3=rand(3,1)*10;
% Choose six random vectors
v1=rand(3,1)*10;
v2=rand(3,1)*10;
v3=rand(3,1)*10;
v4=rand(3,1)*10;
v5=rand(3,1)*10;
v6=rand(3,1)*10;

% Create Screws
S1=[v1;cross(P1,v1)];
S2=[v2;cross(P1,v2)];
S3=[v3;cross(P1,v3)];
S4=[v4;cross(P2,v4)];
S5=[v5;cross(P2,v5)];
S6=[v6;cross(P2,v6)];

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S);


%%          Objective 2: Check the rank of a system that is composed of 5
%%          lines intersected by one line and another line parallel to
%%          this. SOLN: RANK5 since parallel line intersects at infinity

P1=rand(3,1)*10;
P2=rand(3,1)*10;
P3=rand(3,1)*10;
P4=rand(3,1)*10;


% Choose five random direction vectors
v1=rand(3,1)*10;
v2=rand(3,1)*10;
v3=rand(3,1)*10;
v4=rand(3,1)*10;
v5=rand(3,1)*10;

%Sixth vector is parallel to insecting line
v6=P1-P2;


% Create Screws
S1=[v1;cross(P1,v1)];
S2=[v2;cross(P1,v2)];
S3=[v3;cross(P1,v3)];
S4=[v4;cross(P2,v4)];
S5=[v5;cross(P2,v5)];
S6=[v6;cross(P3,v6)];

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S);

%%          Objective 3: Check the rank of a system that is composed of 4 
%%          lines in one plane


% Four points in the same plane, x-y plane
P1=[rand(1) ; rand(1);0]*10;% 
P2=[rand(1) ; rand(1);0]*10;%  
P3=[rand(1) ; rand(1);0]*10;% 
P4=[rand(1) ; rand(1);0]*10;% 
P5=rand(3,1)*10;
P6=rand(3,1)*10;

% Two random points

% Choose 4 vectors in this plane 
v1=[rand(1) ; rand(1);0]*10;% 
v2=[rand(1) ; rand(1);0]*10;% 
v3=[rand(1) ; rand(1);0]*10;% 
v4=[rand(1) ; rand(1);0]*10;% 

% Two random vectors
v5=rand(3,1)*10;
v6=rand(3,1)*10;

%Sixth vector is parallel to insecting line



% Create Screws
S1=[v1;cross(P1,v1)];
S2=[v2;cross(P2,v2)];
S3=[v3;cross(P3,v3)];
S4=[v4;cross(P4,v4)];
S5=[v5;cross(P5,v5)];
S6=[v6;cross(P6,v6)];

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S)












%%          Objective 3: Put into Notation of Screw Analysis for mobility
%%          check

A=rand(3,1)*10; % Intersection of U joint {q1,q2}
B=rand(3,1)*10; % Intersection of U joint {q6,q7}
C=rand(3,1)*10; % Intersection of S joint {q3,q4,q5}
D=rand(3,1)*10; % Intersection of S joint {q8,q9,q10}

v1=C-A; % Direction of Contraint wrench 1
v2=D-B; % Direction of Contraint wrench 2

v3=rand(3,1)*10; % Actuation wrench joint 3
v4=rand(3,1)*10; % Actuation wrench joint 4
v5=rand(3,1)*10; % Actuation wrench joint 8

%Sixth vector is parallel to insecting line
v6=B-A; % Actuation wrench joint 2
v6=rand(3,1)*10; %Actuation wrench joint 2

% Create Screws

S1=[v1;cross(A,v1)];  % Contraint wrenches
S2=[v2;cross(B,v2)]; 

S3=[v3;cross(A,v3)];  % Actuation wrenches
S4=[v4;cross(A,v4)];
S5=[v5;cross(B,v5)];
S6=[v6;cross(C,v6)];

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S);



%%          Objective 4: Checking for 2,3,4,5
%%          check

A=rand(3,1)*10; % Intersection of U joint {q1,q2}
B=rand(3,1)*10; % Intersection of U joint {q6,q7}
C=rand(3,1)*10; % Intersection of S joint {q3,q4,q5}
D=rand(3,1)*10; % Intersection of S joint {q8,q9,q10}

v1=C-A; % Direction of Contraint wrench 1
v2=D-B; % Direction of Contraint wrench 2

v3=rand(3,1)*10; % Actuation wrench joint 3
v4=rand(3,1)*10; % Actuation wrench joint 4
v5=rand(3,1)*10; % Actuation wrench joint 5

%Sixth vector is parallel to insecting line
v6=B-A; % Actuation wrench joint 2



% Create Screws

S1=[v1;cross(A,v1)];  % Constraint wrenches arm 1
S2=[v2;cross(B,v2)];  % Constraint wrenches arm 2               

S3=[v6;cross(C,v6)];  % Actuation wrenches joint 2
S4=[v3;cross(A,v3)];  % Actuation wrenches joint 3
S5=[v5;cross(A,v5)];  % Actuation wrenches joint 4
S6=[v4;cross(A,v4)];  % Actuation wrenches joint 5

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S);


%%          Objective 5: Put into Notation of Screw Analysis for mobility
%%          checking for 2,3,4,7

A=rand(3,1)*10; % Intersection of U joint {q1,q2}
B=rand(3,1)*10; % Intersection of U joint {q6,q7}
C=rand(3,1)*10; % Intersection of S joint {q3,q4,q5}
D=rand(3,1)*10; % Intersection of S joint {q8,q9,q10}

v1=C-A; % Direction of Contraint wrench 1
v2=D-B; % Direction of Contraint wrench 2

v3=rand(3,1)*10; % Actuation wrench joint 3
v4=rand(3,1)*10; % Actuation wrench joint 4
v5=rand(3,1)*10; % Actuation wrench joint 8

%Sixth vector is parallel to insecting line
v6=B-A; % Actuation wrench joint 2


% Create Screws

S1=[v1;cross(A,v1)];  % Constraint wrenches arm 1
S2=[v2;cross(B,v2)];  % Constraint wrenches arm 2               

S3=[v6;cross(C,v6)];  % Actuation wrenches joint 2
S4=[v3;cross(A,v3)];  % Actuation wrenches joint 3
S5=[v5;cross(A,v5)];  % Actuation wrenches joint 4
S6=[v6;cross(D,v6)];  % Actuation wrenches joint 7

% System
S=[S1 S2 S3 S4 S5 S6];
rank(S);


%%          Objective 6: Show 2 parallel forces are 2R2T

% Choose two points ( always an intersecting line)
P1=[0; 1; 0];%rand(3,1)*10;
P2=[0 ;1 ;0];%rand(3,1)*10;


% Choose six random vectors
v1=rand(3,1)*10;
v2=rand(3,1)*10;

% Parallel Vectors
v3=[rand(1);0; 0];
v4=[rand(1);0;0];

% Intersecting Vectors
v5=[rand(1);rand(1); rand(1)];
v6=[rand(1);0;0];

% Create zero pitch Screws
S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[v3/norm(v3);cross(P3,v3)];
S4=[v4/norm(v4);cross(P4,v4)];
S5=[v5/norm(v5);cross(P5,v5)];
S6=[v6/norm(v6);cross(P6,v6)];

% System
S=[S1 S2];
%Matrix which is postmultpied by twist 
PreMul=[zeros(3) eye(3);eye(3) zeros(3)];

Sn=[(PreMul*S1)';(PreMul*S2)'];
null(Sn)
rank(S);

%%          Objective 6.1: Check rank of system contain two parallel forces and two parallel moments
% Here this corresponds to Wei Ye Struture Synthesis and Kinematics of A class of 2R2T...
% P1 and P2 are points of last revolute joints of
% each leg which are said to be coincident


% Only get pure rotations if we chose object
% frame!!!!!
% Choose two points ( always an intersecting line)
P1=[0; 0; 0];%rand(3,1)*10; 
P2=[-5 ;0 ;0];%rand(3,1)*10;


% Two parallel forces
v1=[0; 1 ;0]; % is a pure force along y axis leg 1
v2=v1;% is a pure force along y axis leg 2

% Two parallel moment along z axis
v3=[0; 0; 1]; 
v4=[0; 0; 1];


% Create zero pitch Screws
S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[0;0;0;v3];
S4=[0;0;0;v4];


S=[S1 S2 S3 S4];
rank(S)

PreMul=[zeros(3) eye(3);eye(3) zeros(3)];

Sn=[(PreMul*S)';(PreMul*S)'];
null(Sn)

%%          Objective 6.2: Check rank of system contain two parallel forces and two parallel moments
%% Here this corresponds to Wei Ye Struture Synthesis and Kinematics of A class of 2R2T...
% P1 and P2 are points of last revolute joints of
% each leg which are said to be coincident but
% trying parallel


% If coinicdent then P1 and P2 lie on the same
% axis i.E. the x axis
% If parallel then P1 and P2 simplfiy lie in the
% x0y plane
%

% Choose two points ( always an intersecting line)
P1=[0; 0; 0];%rand(3,1)*10; 
P2=[10 ;-500;0];%rand(3,1)*10;


% Two parallel forces
v1=[0; 1 ;0]; % is a pure force along y axis leg 1
v2=v1;% is a pure force along y axis leg 2

% Two parallel moment along z axis
v3=[0; 0; 1]; 
v4=[0; 0; 2];


% Create zero pitch Screws
S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[0;0;0;v3];
S4=[0;0;0;v4];


S=[S1 S2 S3 S4];
rank(S)

PreMul=[zeros(3) eye(3);eye(3) zeros(3)];

Sn=[(PreMul*S)';(PreMul*S)'];
null(Sn)
plotscrew(S1)
hold on
plotscrew(S2)
plotscrew(S3)
plotscrew(S4)


%%          Objective 6.3: Check rank of system contain two parallel forces and One moment
%% Here this corresponds to Wei Ye Struture Synthesis and Kinematics of A class of 2R2T...
% adding in the the rotational or prisamtic joint
% to eliminate redundant constraints




% Only get pure rotations if we chose object
% frame!!!!!
% Choose two points ( always an intersecting line)
P1=[0; 0; 0];%rand(3,1)*10; 
P2=[-5 ;0 ;0];%rand(3,1)*10;


% Two parallel forces
v1=[0; 1 ;0]; % is a pure force along y axis leg 1
v2=v1;% is a pure force along y axis leg 2

% Two parallel moment along z axis
v3=[0; 0; 1]; 



% Create zero pitch Screws
S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[0;0;0;v3];



S=[S1 S2 S3];
rank(S)

PreMul=[zeros(3) eye(3);eye(3) zeros(3)];

Sn=[(PreMul*S)';(PreMul*S)'];
null(Sn)
plotscrew(S1)
hold on
plotscrew(S2)
plotscrew(S3)
plotscrew(S4)


%%          Objective 6.3: Check rank of system contain two parallel forces and One moment normal to both of them
% and the line between them i.e prependicur to the
% plane formed by the two pure forces
% Rank SYSTEM=2
% Check two parallel forces and a ifnite pitch
% moment perpendicular to both




% Only get pure rotations if we chose object
% frame!!!!!
% Choose two points ( always an intersecting line)
P1=rand(3,1)*10; 
P2=rand(3,1)*10;


% Two parallel forces
v1=rand(3,1); % is a pure force along y axis leg 1
v2=v1;% is a pure force along y axis leg 2

% Moment along z axis
v3=cross(v1,(P2-P1));



% Create zero pitch Screws
S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[0;0;0;v3/norm(v3)];



S=[S1 S2 S3];
rank(S)

PreMul=[zeros(3) eye(3);eye(3) zeros(3)];

Sn=[(PreMul*S)';(PreMul*S)'];
null(Sn)
plotscrew(S1)
hold on
plotscrew(S2)
plotscrew(S3)


%% Check the dependency of Gough stewart problem
%
% Write the six twists
%
%

% The actuation wrench of each leg is the pure
% force along the leg 
P1=[0,0,0]';
P2=[0,0,0]';
P3=[0,0,0]';
P4=[0,0,3]';
P5=[0,2,0]';
P6=[5,1,1]';


% Choose six random vectors
v1=rand(3,1)*10;
v2=rand(3,1)*10;
% Parallel Vectors
v3=rand(3,1)*10;
v4=rand(3,1)*10;
% Intersecting Vectors
v5=rand(3,1)*10;
v6=rand(3,1)*10;

S1=[v1/norm(v1);cross(P1,v1)];
S2=[v2/norm(v2);cross(P2,v2)];
S3=[v3/norm(v3);cross(P3,v3)];
S4=[v4/norm(v4);cross(P4,v4)];
S5=[v5/norm(v5);cross(P5,v5)];
S6=[v6/norm(v6);cross(P6,v6)];

S=[S1 S2 S3 S4 S5 S6];
rank(S)