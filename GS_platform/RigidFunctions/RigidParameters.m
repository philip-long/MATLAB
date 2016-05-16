
clear all,clc,close all
% I have to define the initial position of the
% platform frame here
% Initial Joint Conditions
global T0M_init q1init q2init q3init q4init q5init q6init
global qdotinit qddotinit Vinit Vdotinit Xinit
global Leg1Origin Leg2Origin Leg3Origin Leg4Origin Leg5Origin Leg6Origin Tw0
global PM1 PM2 PM3 PM4 PM5 PM6
global XX2 XY2 XZ2 YY2 YZ2 ZZ2
global XX3 XY3 XZ3 YY3 YZ3 ZZ3 MX3 MY3 MZ3
global IA3 M3 FZ3  FV3 FS3 CX3 CY3 CZ3
global FX3 FY3 MZ2 MY2 MX2 CX2 CY2 CZ2 IA2 
global FV2 FS2 ZZ1 MY1 MX1 CZ1 IA1 FV1 FS1
global G3


Tw0=[1,0,0,0.1;    0,1,0,-0.2;    0,0,1,0;    0,0,0,1];
Leg1Origin=[0.1,-0.2]; Leg2Origin=[0.3,-0.2]; Leg3Origin=[1.3,0.4];
Leg4Origin=[1.0,1.0];  Leg5Origin=[-0.2,0.9]; Leg6Origin=[-0.7,0.5];

qdotinit=zeros(1,18);
qddotinit=zeros(1,18);
Vinit=zeros(1,6);
Vdotinit=zeros(1,6);

q1init=[ -1.67052291858514, 1.76719211975172, 1.02468346456505];   
q2init=[   -1.37340080343882 ,1.76445463930196 ,1.03923047136536];
q3init=[-2.01602298775505,1.81447526016778, 1.14174787421098];
q4init=[ -1.95750106005062,1.85857262515691,1.12603636827087];
q5init=[  -1.65602378746603,1.38947424950535,1.02037051497794];
q6init =[  -1.93636638772483,1.34509329151894,1.09862637690922];

% Position of  attachment points with respect to
% platform they are fixed in rigid case
ControlledOrigin=[0.0,0.0,0.0];

PM1=(-ControlledOrigin+[-0.25,-0.433,0])'
PM2=(-ControlledOrigin+[0.25,-0.433,0])'
PM3=(-ControlledOrigin+[0.5 ,0.0,0])'
PM4=(-ControlledOrigin+[0.25 ,0.433,0])'
PM5=(-ControlledOrigin+[ -0.25,0.433,0])'
PM6=(-ControlledOrigin+[-0.5,0.0,0])'


% Position of platform Frame w.r.t the leg frames 
% Taking Platform frame at center of origin i.e
% [0.25,0.4333,0.0]

T0M_init=[1  0   0  ControlledOrigin(1)
          0  1   0  ControlledOrigin(2)
          0  0   1  1.0
          0  0   0  1.0];
      
 Xinit=[T0M_init(1,4) T0M_init(2,4) T0M_init(3,4) Rot_to_ZXZ(T0M_init(1:3,1:3))];
 
 
 
 %% Links
% Link 1
XX1=0.0005; YY1=0.0005; ZZ1=0.001;M1=0.25; XY1=0.0;XZ1=0.0;YZ1=0.0;MX1=0.0;MY1=0.0;MZ1=0.0;IA1=0.0;
% Link 2
XX2=0.0005; YY2=0.0005; ZZ2=0.001;M2=0.25;XY2=0.0;XZ2=0.0;YZ2=0.0;MX2=0.0;MY2=0.0;MZ2=0.0;IA2=0.0;
% Link 3
XX3=0.0075; YY3=0.0075; ZZ3=0.0015;M3=0.4;XY3=0.0;XZ3=0.0;YZ3=0.0;MX3=0.0;MY3=0.0;MZ3=0.0;IA3=0.0;

FV1=0.0;FS1=0.0;FV2=0.0;FS2=0.0;FV3=0.0;FS3=0.0;FX3=0.0;FY3=0.0; FZ3=0.0;CX3=0.0; CY3=0.0; CZ3=0.0;
CX2=0.0; CY2=0.0; CZ2=0.0;CZ1=0.0;

G3=-9.80665;


%% Rigid Platform Parameters 
% D:/Documents/DeformableObjectModeling/StewartPlatform/yg_30gpa_carbon_0.mnf
% global Mp MSP Ig
% Mp= 12.9899997166;
% MSP=[0 ;0 ;0];
% G1M=[0.2500000005, 0.433000003, 0.0]; % With respect to local origin (Attachment pt. 1)
%  
% 
% % Inertia Tensor  : (relative to the Local Body Reference Frame) 
% IXX=3.1374695077;% kg-meter**2
% IYY=1.5138527646;% kg-meter**2
% IZZ=4.6513222723;% kg-meter**2
% IXY=-1.4061618211;% kg-meter**2
% IZX=0.0;% kg-meter**2
% IYZ=0.0;% kg-meter**2
% 
% IP=[IXX IXY IZX
%     IXY IYY IYZ
%     IZX IYZ IZZ];
% 
% % Must Transform Inertia Tensor to centre of mass
% % ie my origin in fact I am trasnforming back to
% % the centre of gravity!!
% Sj=-G1M;
% Ig=IP+Mp*(skew(Sj)*skew(Sj))

% My Calculation
%% Ok I have a simple hexagon








%% Rigid Platform Two
%   MNF name        : D:/Documents/DeformableObjectModeling/StewartPlatform/yg_200gpa_lowdenisty_0.mnf

% global Mp G1M MSP IP 
% Mp= 6.4949998511;
% MSP=[0 ;0 ;0];
% G1M=[0.2500000005, 0.433000003, 0.0]; % With respect to local origin (Attachment pt. 1)
%  % Inertia Tensor  : (relative to the Local Body Reference Frame) 
% IXX=1.5687347539 ;% kg-meter**2
% IYY=0.7569263823;% kg-meter**2
% IZZ=2.3256611361;% kg-meter**2
% IXY=0.7030809105;% kg-meter**2
% IZX=0.0;% kg-meter**2
% IYZ=0.0;% kg-meter**2
% 
% IP=[IXX IXY IZX
%     IXY IYY IYZ
%     IZX IYZ IZZ];



%% Rigid Platform Parameters 
%D:/Documents/DeformableObjectModeling/StewartPlatform/highmeshmat2_0.mnf
global Mp MSP Ig
Mp= 6.4949998101;
MSP=[0 ;0 ;0];
G1M=[0.00, 0.000, 0.0]; % With respect to local origin (Attachment pt. 1)
 

% Inertia Tensor  : (relative to the Local Body Reference Frame) 
IXX=0.3387687874;% kg-meter**2
IYY=0.338788656;% kg-meter**2
IZZ=0.6775574433;% kg-meter**2
IXY=0.0;% kg-meter**2
IZX=0.0;% kg-meter**2
IYZ=0.0;% kg-meter**2

IP=[IXX IXY IZX
    IXY IYY IYZ
    IZX IYZ IZZ];

% Must Transform Inertia Tensor to centre of mass
% ie my origin in fact I am trasnforming back to
% the centre of gravity!!
Sj=-G1M;
Ig=IP+Mp*(skew(Sj)*skew(Sj))
