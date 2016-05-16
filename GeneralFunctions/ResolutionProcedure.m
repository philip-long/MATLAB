% Last of the three new files or first
% ,ResolutionForceatToolFrame,ResolutionParameters.ResolutionProcedure
% This file actually defines the resolution procedure so its more how the
% matrix is first created
%
%

% Force sensor to base
syms fMb11 fMb12 fMb13 fMb21 fMb22 fMb23 fMb31 fMb32 fMb33
syms tMf11 tMf12 tMf13 tMf21 tMf22 tMf23 tMf31 tMf32 tMf33
syms b1 b2 b3 b4 b5 b6 fPxt fPyt fPzt M_fs M_t

% Distance from Centre of mass of tool to force sensor
syms fPXcg_t fPYcg_t fPZcg_t

% 
syms mf1 mf2 mf3 mf4 mf5 mf6

%

Bias=[b1; b2; b3; b4; b5; b6];

MeasuredForce=[mf1;mf2;mf3;mf4;mf5;mf6];

fMb=[fMb11 fMb12 fMb13
      fMb21 fMb22 fMb23
     fMb31 fMb32 fMb33];
 
tMf=[tMf11 tMf12 tMf13
      tMf21 tMf22 tMf23
     tMf31 tMf32 tMf33];
 
 %% How to find reolved force at tool frame
 
% Force due to mass of tool in base frame
bFt_mass=[0;0;-9.81*M_t;0;0;0];
% Force due to mass of sensor in base frame
bFf_mass=[0;0;-9.81*M_fs;0;0;0];  
 
 
% Force due to mass of sensor in Force sensor Frame
% Simply change the frame
 fFf_mass=[fMb(1:3,1:3) zeros(3);zeros(3) fMb(1:3,1:3)]*[0;0;-9.81*M_fs;0;0;0];
 
%  Force due to mass of tool in  force sensor frame
% Change the frame and add torque due to distance between centre of
% mass of tool frame and force sensor MX_t,MY_t,MZ_t
LC=-skew([fPXcg_t;fPYcg_t;fPZcg_t])*fMb(1:3,1:3);
fFt=[fMb(1:3,1:3) zeros(3);LC fMb(1:3,1:3)]*bFt_mass;

% Resolved force at the sensor frame
% Algebraic addition of all forces
fFrev=-Bias-fFf_mass-fFt;


% Transform to tool frame
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3);

% Resolved forces at Tool frame due to static forces and bias
tFrev=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*fFrev

% Next find the matrix 
%
%  X are the parameters we want to identify
% bias, mass of force sensor, mass of tool M_t is always
% inseperable on fPXcg_t fPYcg_t fPZcg_t
X=[b1 b2 b3 b4 b5 b6 M_fs fPXcg_t fPYcg_t fPZcg_t]
W=jacobian(tFrev,X)
W=jacobian(fFrev,X)