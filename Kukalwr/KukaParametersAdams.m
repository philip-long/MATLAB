%% A chart organsiging the kuka parameters for the MSC ADAMS SIMULATION

% It gives the location of each joint relative to the base in the candle position 
% the location is in the format of (a) the marker in Adams orientation given in Z-X-Z(b)
% Transformation matrix in matlab

% Using the Identified parameters of MX, MY , MZ the centre of mass of each
% link is found. The inertia is then asigned to each link at the link frame
% i.e at the joint coordidinates. Mass is assigned at 2.5kg since unknown

% Units are in metres radians kg 
T=GENDGM(zeros(7,1),'Notool');
%% JOINT 1
T10=T{1}; % Transformation from joint 1 to zero
P10=T{1}(1:3,4)'
y1=Rot2Rep(T10(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G1=[2.5763074054E-007, 2.3959927101E-002, 0.2230369976];
S1=(T10)\([G1 1 ]');
MS1=M1*S1'





%% JOINT 2
T20=T{2}; % Transformation from joint 2 to zero
P20=T{2}(1:3,4)'
y2=Rot2Rep(T20(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G2=[2.5666654467E-007, -2.5900655368E-002, 0.3861087515];
S2=(T20)\([G2 1 ]');
MS2=M2*S2'


%% JOINT 3
T30=T{3}; % Transformation from joint 3 to zero
P30=T{3}(1:3,4)'
y3=Rot2Rep(T30(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G3=[-2.5667012097E-007, -2.5959926585E-002, 0.6230369984];
S3=(T30)\([G3 1 ]');
MS3=M3*S3'

%% JOINT 4
T40=T{4}; % Transformation from joint 4 to zero
P40=T{4}(1:3,4)'
y4=Rot2Rep(T40(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G4=[-2.5757981305E-007, 2.3900655902E-002, 0.7861087528];
S4=(T40)\([G4 1 ]');
MS4=M4*S4'

%% JOINT 5
T50=T{5}; % Transformation from joint 5 to zero
P50=T{5}(1:3,4)'
y5=Rot2Rep(T50(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G5=[1.5450766138E-006, 2.1572606281E-002, 0.9911574559];
S5=(T50)\([G5 1 ]');
MS5=M5*S5'

%% JOINT 6
T60=T{6}; % Transformation from joint 6 to zero
P60=T{6}(1:3,4)'
y6=Rot2Rep(T60(1:3,1:3));


% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G6=[5.9769152275E-008, -3.9737606962E-003, 1.0975782182];
S6=(T60)\([G6 1 ]');
MS6=M6*S6'

%% JOINT 7
T70=T{7}; % Transformation from joint 7 to zero
P70=T{7}(1:3,4)'
y7=Rot2Rep(T70(1:3,1:3));

% CENTRE OF MASS IN GLOBAL FRAME IN CANDLE POSITION - ADAMS CALC
% Transform to link frame
% Multiply by adams Parameters to get MX MY MZ
G7=[0.0, 5.0E-004, 1.1627027111];
S7=(T70)\([G7 1 ]');
MS7=M7*S7';
