
clear all,clc
%% Find Point 1 and Point 2
 
%l1 = 4; % AE en [m]
%l2 = 4; % BE en [m] 
alpha = sym(pi)/6; %[rad]
syms l1
l2=l1
theta=pi-(2*alpha);
l3 = (l1^2 +l2^2 - 2*l1*l2*cos(theta))^0.5;

rhoA = 2;

%rhoA = sqrt(l3^2/sin(alpha)^2)

% Expression of rhoB
rhoB1 = rhoA*cos(2*alpha)-sqrt(l3^2-rhoA^2*sin(2*alpha)^2);
rhoB2 = rhoA*cos(2*alpha)+sqrt(l3^2-rhoA^2*sin(2*alpha)^2);

u1=[cos(alpha); -sin(alpha)]; u1xk=[-sin(alpha) ;-cos(alpha)];
u2=[cos(alpha); sin(alpha)]; u2xk=[sin(alpha); -cos(alpha)];

abold = rhoA*[cos(alpha) -sin(alpha)]';
bbold = rhoB2*[cos(alpha) sin(alpha)]';

% Computation of beta [angle(AB,AE)]
beta = acos((l1^2+l3^2-l2^2)/(2*l1*l3));

% ub : unit vector of AB
ABvec = bbold-abold;
ABvecnorm = norm(ABvec,2);
ub = ABvec/ABvecnorm;

% Vb : unit vector normal to AB (Rot(pi/2))
Eb = [0 -1 ; 1 0];
vb = Eb*ub;

% Cartesian coordinates of point E
ebold = abold + l1*cos(beta)*ub - l1*sin(beta)*vb;

S=inv([-u1xk u2xk])*(abold-bbold);
P1=simplify(abold+ S(1).*u1xk);
P2=simplify(bbold+ S(2).*u2xk);


%% %% Plotting


%% Geometric parameters
syms l4 l5 theta2
%l4 = 3; % CE2 en [m]
%l5 = 4; % DE2 en [m] 
%alpha = pi/4; %[rad]
theta2=pi-(2*alpha);
l6 = (l4^2 +l5^2 - 2*l4*l5*cos(theta2))^0.5;

rhoC = 2;

%rhoA = sqrt(l3^2/sin(alpha)^2)

% Expression of rhoD
rhoD1 = rhoC*cos(2*alpha)-sqrt(l6^2-rhoC^2*sin(2*alpha)^2);
rhoD2 = rhoC*cos(2*alpha)+sqrt(l6^2-rhoC^2*sin(2*alpha)^2);


% Cartesian coordinates of point C
cbold = rhoC*[cos(alpha) sin(alpha)]';

u1=[cos(alpha); -sin(alpha)]; u1xk=[-sin(alpha) ;-cos(alpha)];
u2=[cos(alpha); sin(alpha)]; u2xk=[sin(alpha); -cos(alpha)];

% Cartesian coordinates of point D
% NB: Here, let's choose rhoD2
dbold = rhoD2*[cos(alpha) -sin(alpha)]';

beta2 = acos((l5^2+l6^2-l4^2)/(2*l5*l6));

% ub2 : unit vector of DC
DCvec = cbold-dbold;
DCvecnorm = norm(DCvec,2);
ub2 = DCvec/DCvecnorm;

% Vb : unit vector normal to AB (Rot(pi/2))
Eb = [0 -1 ; 1 0];
vb2 = Eb*ub2;

% Cartesian coordinates of point E2
e2bold = dbold + l5*cos(beta2)*ub2 - l5*sin(beta2)*vb2;

S=inv([-u2xk u1xk])*(cbold-dbold);
P3=cbold+ S(1).*u2xk;
P4=dbold+ S(2).*u1xk;
%% Check intersection between 
% 0 if they intersect

% by determinant
P=det([ones(3,1) [P1';P3';e2bold'] ]) 
% by slope 
(P3(1)-P1(1))/(P3(2)-P1(2));
(P3(1)-e2bold(1))/(P3(2)-e2bold(2));

%%
figure(1)
clf
figure(1)
%grid on
hold on
plot(0,0,'kd')
plot(abold(1),abold(2),'bo')
plot(bbold(1),bbold(2),'go')
plot(ebold(1),ebold(2),'ro')

plot(P1(1),P1(2),'ko')
plot(P2(1),P2(2),'g*')

sidecone = l1+l2;
endside1 = sidecone*[cos(alpha) -sin(alpha)]';
endside2 = sidecone*[cos(alpha) sin(alpha)]';
line([0 endside1(1)],[0 endside1(2)],'Color',[.8 .8 .8])
line([0 endside2(1)],[0 endside2(2)],'Color',[.8 .8 .8])
% segment AB
line([abold(1) bbold(1)],[abold(2) bbold(2)],'Color',[0 0 0])

% segment Intersection points
line([abold(1) P1(1)],[abold(2) P1(2)],'Color',[0 1 0])
line([bbold(1) P1(1)],[bbold(2) P1(2)],'Color',[0 1 0])

line([abold(1) ebold(1)],[abold(2) ebold(2)],'Color',[0 0 0])
% segment BE
line([bbold(1) ebold(1)],[bbold(2) ebold(2)],'Color',[0 0 0])

hold off
%axis square
axis equal
xlabel('x [m]')
ylabel('y [m]')

figure(1)
%grid on
hold on
plot(cbold(1),cbold(2),'bs')
plot(dbold(1),dbold(2),'gs')
plot(e2bold(1),e2bold(2),'rs')

plot(P3(1),P3(2),'mo')
plot(P4(1),P4(2),'m*')

% sidecone = l1+l2;
% endside1 = sidecone*[cos(alpha) -sin(alpha)]';
% endside2 = sidecone*[cos(alpha) sin(alpha)]';
% line([0 endside1(1)],[0 endside1(2)],'Color',[.8 .8 .8])
% line([0 endside2(1)],[0 endside2(2)],'Color',[.8 .8 .8])
% segment CD
line([cbold(1) dbold(1)],[cbold(2) dbold(2)],'Color',[1 0 1])

% segment Intersection points
line([cbold(1) P3(1)],[cbold(2) P3(2)],'Color',[0.5 0 0.5])
line([dbold(1) P3(1)],[dbold(2) P3(2)],'Color',[0.5 0 0.5])

line([cbold(1) e2bold(1)],[cbold(2) e2bold(2)],'Color',[1 0 1])
% segment DE
line([dbold(1) e2bold(1)],[dbold(2) e2bold(2)],'Color',[1 0 1])


line([P1(1) P3(1)],[P1(2) P3(2)],'Color',[0 0 1])