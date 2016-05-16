clear all,clc
%% This cell is creating the input data
bMt=TransMat(pi/4,'rx','T')*TransMat(pi/5,'rz','T')*TransMat(rand(1),'rx','T')*TransMat(rand(3,1),'T');

MeasuredForce=rand(6,1)*10000000;

syms b1 b2 b3 b4 b5 b6 ComMx ComMy ComMz MassForceSensor Mass
Bias=[b1; b2; b3; b4; b5; b6];
COM=[ComMx; ComMy; ComMz];


%---------------------Definitions----------------
% Tool Frame Offset
Mx=0;My=0;Mz=0.14; 
%Force due to mass of tool
bFt_mass=[0;0;-9.81*Mass;0;0;0];
%Force due to mass of sensor
bFf_mass=[0;0;-9.81*MassForceSensor;0;0;0];   
%This is the transofrmation matrix from tool to force sensor
fMt=[0.0 1 0 Mx;-1 0 0 My; 0 0 1 Mz; 0 0 0 1];

%This is the transofrmation matrix from tool to force sensor
fMt=[0.0 1 0 Mx;-1 0 0 My; 0 0 1 Mz; 0 0 0 1];


% Method: Do calculation at force sensor frame
% then transform result to tool frame:
% Calc 1. get force sensor to base
fMb=fMt*(inv(bMt));
%Calc 2 inverse this matrix
tMf=inv(fMt);
%Calc 3.Get screw trans
LC=-skew([ComMx;ComMy;ComMz])*fMb(1:3,1:3);

% Calc 4. Get Force sensor weight in force sensor frame
fFf_mass=simple([fMb(1:3,1:3) zeros(3);zeros(3) fMb(1:3,1:3)]*bFf_mass);
% Calc 5. Weight of tool in force sensor frame
fFt=[fMb(1:3,1:3) zeros(3);LC fMb(1:3,1:3)]*bFt_mass;


% Calc 6: add a force sensor frame
fFrev=(MeasuredForce/1000000)-Bias-fFf_mass-fFt;

% Clac 7 another screw
LD=skew([Mx;My;Mz])*tMf(1:3,1:3);
%Calc 8 move to tool frame
tFrev=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*fFrev


%%
%y=Bx
% x is the vector containing the unknown
% parameters
% B=is the observation matrix
% y=are the measured torques in the tool frame
%
syms fMb11 fMb12 fMb13 fMb21 fMb22 fMb23 fMb31 fMb32 fMb33


fMb=[fMb11 fMb12 fMb13
      fMb21 fMb22 fMb23
     fMb31 fMb32 fMb33];
% If we put everything in the force sensor frame
% it should be easier
(MeasuredForce/1000000)=Bias+fFf_mass+fFt;
%fFf_mass
fFf_mass=[fMb(1:3,1:3) zeros(3);zeros(3) fMb(1:3,1:3)]*[0;0;-9.81*MassForceSensor;0;0;0];

% fFt
LC=-skew([ComMx;ComMy;ComMz])*fMb(1:3,1:3);
fFt=[fMb(1:3,1:3) zeros(3);LC fMb(1:3,1:3)]*[0;0;-9.81*Mass;0;0;0];
%



Soln=Bias+fFf_mass+fFt


%% Vector
syms MassComX MassComY MassComZ
x=[b1;b2;b3;b4;b5;b6;Mass;MassForceSensor;MassComX;MassComY;MassComZ];
% I can also get B from partially differetiing
% Soln, rank of mass part is only 4
B=[1  0  0  0  0  0  -9.81*fMb13  -9.81*fMb13 0             0          0
   0  1  0  0  0  0  -9.81*fMb23  -9.81*fMb23 0             0          0
   0  0  1  0  0  0  -9.81*fMb33  -9.81*fMb33 0             0          0
   0  0  0  1  0  0      0            0       0         9.81*fMb33  -9.81*fMb23
   0  0  0  0  1  0      0            0   -9.81*fMb33       0       9.81*fMb13
   0  0  0  0  0  1      0            0    9.81*fMb23  -9.81*fMb13     0];

Bsim=[eye(3) zeros(3) -9.81*fMb(:,3) -9.81*fMb(:,3) zeros(3)
     zeros(3) eye(3) zeros(3,2) -9.81*skew(fMb(:,3))] 
      
 







