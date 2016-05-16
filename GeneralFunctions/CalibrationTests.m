% This script is used to help me orgranie my
% thoughts regarding the online calibration of the
% force sensor not including dynamics:
clear all,clc
%% This cell is creating the input data
bMt=TransMat(pi/4,'rx','T')*TransMat(pi/5,'rz','T')*TransMat(rand(1),'rx','T')*TransMat(rand(3,1),'T');

MeasuredForce=rand(6,1);
%MeasuredForce=[1 2 3 1 2 17 ]';
%---------------------Definitions----------------

% Define Bias
Bias(1)=9.26;Bias(2)=-0.92;Bias(3)=-1.79;Bias(4)=-0.0065;Bias(5)=-1.435;Bias(6)=-0.014;
% Centre of mass of tool to force sensors
ComMx=0.0;ComMy=0.0;ComMz=0.02; 
% Tool Frame Offset
Mx=0.0;My=0.0;Mz=0.14; 
%Mass of Tool
Mass=0.2; 
%Mass of Flexible type of Sensor
MassForceSensor=0.09; 
%Force due to mass of tool
bFt_mass(1)=0;bFt_mass(2)=0;bFt_mass(3)=-9.81*Mass;bFt_mass(4)=0;bFt_mass(5)=0;bFt_mass(6)=0;
%Force due to mass of sensor
bFf_mass(1)=0;bFf_mass(2)=0;bFf_mass(3)=-9.81*MassForceSensor;bFf_mass(4)=0;bFf_mass(5)=0;bFf_mass(6)=0;    
  
%This is the transofrmation matrix from tool to force sensor
fMt=[0.0 1 0 Mx;-1 0 0 My; 0 0 1 Mz; 0 0 0 1];

%-------------------------------------------------

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method One: Objective write and understand
% void zeroforcesensortest(
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Method: Do calculation at force sensor frame
% then transform result to tool frame:
% Calc 1. get force sensor to base
fMb=fMt*(inv(bMt));
%Calc 2 inverse this matrix
tMf=inv(fMt);
%Calc 3.Get screw trans
LC=-skew([ComMx;ComMy;ComMz])*fMb(1:3,1:3);

% Calc 4. Get Force sensor weight in force sensor frame
fFf_mass=[fMb(1:3,1:3) zeros(3);zeros(3) fMb(1:3,1:3)]*bFf_mass';
% Calc 5. Weight of tool in force sensor frame
fFt=[fMb(1:3,1:3) zeros(3);LC fMb(1:3,1:3)]*bFt_mass';


% Calc 6: add a force sensor frame
fFrev=(MeasuredForce/1000000)-Bias'-fFf_mass-fFt

% Clac 7 another screw
LD=skew([Mx;My;Mz])*tMf(1:3,1:3);
%Calc 8 move to tool frame
tFrev=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*fFrev

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Simplified model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Do not know: Bias, mass of tool, mass of
% force sensor and centre of mass of tool to TCP
% Bias,bFf_mass,bFt_mass,ComMx,ComMy,ComMz
%
% => Need to formulate these into a vector and do
% the least squares method

x=[Bias';Mass;MassForceSensor;Mass*ComMx;Mass*ComMy;Mass*ComMz];

W=[eye(3) zeros(3) -9.81*fMb(1:3,3) -9.81*fMb(1:3,3) zeros(3)
      zeros(3) eye(3) zeros(3,2) -9.81*skew(fMb(1:3,3))] ;
 
 Y=-W*x;
 
 % Therefore take this matrix for many iterations
 % and then and then pseudoinverse it to get good
 % reading so we get Bias, Mass of tool sensor;
 % mass of force sensor and centre of 
 
tFrev_bysim=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*fFrev_bysimmethod


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method two: Objective write and understand
% void zeroforcesensor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Method
% Calc 1. get force sensor to base
%bMf=bMt*(inv(fMt));
tMb=inv(bMt);
% Calc 2. 
fMb=fMt*tMb;
% Calc 3 inverse this matrix
tMf=inv(fMt);

%Calc 4
fFf_mass=[fMb(1:3,1:3) zeros(3); zeros(3) fMb(1:3,1:3)]*bFf_mass';
% Calc 5 
Mforce=(MeasuredForce/1000000)-Bias'-fFf_mass;
LC=-skew(tMf(1:3,4))*tMf(1:3,1:3);
%Calc 5
tFt=[tMf(1:3,1:3) zeros(3);LC  tMf(1:3,1:3)]*Mforce;
%Calc 6
LD=skew([ComMx;ComMy;ComMz])*tMb(1:3,1:3);
%Clac 7
tFt_mass=[tMb(1:3,1:3) zeros(3);LD  tMb(1:3,1:3)]*bFt_mass'
%Calc 8
tFrev=tFt-tFt_mass

% Result




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Proposed Online Claibration:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





















