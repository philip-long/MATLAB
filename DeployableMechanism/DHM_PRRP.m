%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% October 3, 2016
% Philip LONG et St�phane CARO
% 
% Subject : Direct Geometric Model of the PRRP mechanism
% The PRRP mechanism is a subset of the scissor mechanism (deployale
% mechanism)
%
% Geometric parameters: l1, l2, l3, alpha
% Input variable: rhoA
% Output variable : xE, yE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% M�canisme 1 : PRRP

% function [xE,yE] = fun(rhoA,param) 
% 
% % Geometric parameters
% l1 = param(1); %[m]
% l2 = param(2); %[m] 
% l3 = param(3); %[m]
% alpha = param(4); %[rad]

clc
clear all
% % Geometric parameters
% l1 = 3; % AE en [m]
% l2 = 2; % BE en [m] 
% l3 = 4; % AB en [m]
% alpha = pi/4; %[rad]

% Geometric parameters
l1 = 4; % AE en [m]
l2 = 3; % BE en [m] 
alpha = pi/4; %[rad]
theta=pi-(2*alpha);
l3 = (l1^2 +l2^2 - 2*l1*l2*cos(theta))^0.5;



rhoA = 2;

%rhoA = sqrt(l3^2/sin(alpha)^2)

% Expression of rhoB
rhoB1 = rhoA*cos(2*alpha)-sqrt(l3^2-rhoA^2*sin(2*alpha)^2);
rhoB2 = rhoA*cos(2*alpha)+sqrt(l3^2-rhoA^2*sin(2*alpha)^2);

% Cartesian coordinates of point A
abold = rhoA*[cos(alpha) -sin(alpha)]';

% Cartesian coordinates of point B
% NB: Here, let's choose rhoB2
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

%% Plotting
figure(1)
clf
figure(1)
%grid on
hold on
plot(0,0,'kd')
plot(abold(1),abold(2),'bo')
plot(bbold(1),bbold(2),'go')
plot(ebold(1),ebold(2),'ro')

sidecone = l1+l2;
endside1 = sidecone*[cos(alpha) -sin(alpha)]';
endside2 = sidecone*[cos(alpha) sin(alpha)]';
line([0 endside1(1)],[0 endside1(2)],'Color',[.8 .8 .8])
line([0 endside2(1)],[0 endside2(2)],'Color',[.8 .8 .8])
% segment AB
line([abold(1) bbold(1)],[abold(2) bbold(2)],'Color',[0 0 0])
% segment AE
line([abold(1) ebold(1)],[abold(2) ebold(2)],'Color',[0 0 0])
% segment BE
line([bbold(1) ebold(1)],[bbold(2) ebold(2)],'Color',[0 0 0])
hold off
%axis square
axis equal
xlabel('x [m]')
ylabel('y [m]')

%% Check
norm((ebold-abold),2)
norm((ebold-bbold),2)
norm((bbold-abold),2)
% norm(abold,2)
% norm(bbold,2)


%% Point E Loci
reso = 100;
rhoAmin = 0;
rhoAmax = l3;

for rhoA = rhoAmin : (rhoAmax-rhoAmin)/reso : rhoAmax
    
%rhoA = 3;

% Expression of rhoB
rhoB1 = rhoA*cos(2*alpha)-sqrt(l3^2-rhoA^2*sin(2*alpha)^2);
rhoB2 = rhoA*cos(2*alpha)+sqrt(l3^2-rhoA^2*sin(2*alpha)^2);

% Cartesian coordinates of point A
abold = rhoA*[cos(alpha) -sin(alpha)]';

% Cartesian coordinates of point B
% NB: Here, let's choose rhoB2
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
ebold = abold + l1*cos(beta)*ub - l1*sin(beta)*vb

figure(1)
hold on
plot(ebold(1),ebold(2),'r+')
hold off
    
end



%% M�canisme 2 : PRRP

% % Geometric parameters
% l4 = 3; % CE2 en [m]
% l5 = 2; % DE2 en [m] 
% l6 = 4; % DC en [m]
% alpha = pi/4; %[rad]

% Geometric parameters
l4 = l2; % CE2 en [m]
l5 = l1; % DE2 en [m] 
alpha = pi/4; %[rad]
theta2=pi-(2*alpha);
l6 = (l4^2 +l5^2 - 2*l4*l5*cos(theta2))^0.5

rhoC = 2;

%rhoA = sqrt(l3^2/sin(alpha)^2)

% Expression of rhoD
rhoD1 = rhoC*cos(2*alpha)-sqrt(l6^2-rhoC^2*sin(2*alpha)^2);
rhoD2 = rhoC*cos(2*alpha)+sqrt(l6^2-rhoC^2*sin(2*alpha)^2);

% Cartesian coordinates of point C
cbold = rhoC*[cos(alpha) sin(alpha)]';

% Cartesian coordinates of point D
% NB: Here, let's choose rhoD2
dbold = rhoD2*[cos(alpha) -sin(alpha)]';

% Computation of beta [angle(AB,AE)]
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

%% Plotting
figure(1)
%grid on
hold on
plot(cbold(1),cbold(2),'bs')
plot(dbold(1),dbold(2),'gs')
plot(e2bold(1),e2bold(2),'rs')

% sidecone = l1+l2;
% endside1 = sidecone*[cos(alpha) -sin(alpha)]';
% endside2 = sidecone*[cos(alpha) sin(alpha)]';
% line([0 endside1(1)],[0 endside1(2)],'Color',[.8 .8 .8])
% line([0 endside2(1)],[0 endside2(2)],'Color',[.8 .8 .8])
% segment CD
line([cbold(1) dbold(1)],[cbold(2) dbold(2)],'Color',[1 0 1])
% segment CE
line([cbold(1) e2bold(1)],[cbold(2) e2bold(2)],'Color',[1 0 1])
% segment DE
line([dbold(1) e2bold(1)],[dbold(2) e2bold(2)],'Color',[1 0 1])
hold off
%axis square
axis equal
xlabel('x [m]')
ylabel('y [m]')

%% Check
norm((e2bold-cbold),2)
norm((e2bold-dbold),2)
norm((dbold-cbold),2)
% norm(abold,2)
% norm(bbold,2)


%% Point E2 Loci
reso = 100;
rhoCmin = 0;
rhoCmax = l6;

for rhoC = rhoCmin : (rhoCmax-rhoCmin)/reso : rhoCmax

% Expression of rhoD
rhoD1 = rhoC*cos(2*alpha)-sqrt(l6^2-rhoC^2*sin(2*alpha)^2);
rhoD2 = rhoC*cos(2*alpha)+sqrt(l6^2-rhoC^2*sin(2*alpha)^2);

% Cartesian coordinates of point C
cbold = rhoC*[cos(alpha) sin(alpha)]';

% Cartesian coordinates of point D
% NB: Here, let's choose rhoD2
dbold = rhoD2*[cos(alpha) -sin(alpha)]';

% Computation of beta [angle(AB,AE)]
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

figure(1)
hold on
plot(e2bold(1),e2bold(2),'m*')
hold off
    
end



