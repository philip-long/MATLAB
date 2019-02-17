% The Parameters file which defines the sceneraio,
% inital position and other global variables


% Copyright (c) 2012 Philip Long
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.


clear all
clc
warning('OFF','MATLAB:TriScatteredInterp:DupPtsAvValuesWarnId')

%% Parameter file for the kinematics of robot-- Well Known---
% These parameters have been tested in terms of geomtric model
% with the actual robot output. Further more the same parameters
% have been tested with the Adams model.
% i.e for a given  q T70=KRL result = Adams results
%


% Joint initialisations 
global qinit qinitB qinitC qinitTable qlower qupper

% DH parameters
global r3 r5 r7 r1 Joint2Offset
global r7_B alpha d theta r sigma Joints 

% Transformation matrices to base 
global T0L T0L_Vision T0table

% initial position and orientations used as
% equilibrium points
global  X4d_B X4d X4d_C  Pinit 

% define the orientation representation
global RepInit rotdisp

% reundancy task dimension
global  Task_Dimension

% Parameters used to simulate vision
global Guide_Radius vdpts CameraParameters VP1 VP2 VP3 VP4


%---------------------------------------------------------------------------------
%---------------------------------------------------------------------------------
% Choice 1 is to use the KRL or the FRI convention
%---------------------------------------------------------------------------------
%---------------------------------------------------------------------------------
% This offset changes the zero position of joint 2
% In KRL when the robot is in candle configuration (transport position) the
% values are: [0 pi/2 0 0 0 0 0]
%
% In FRI when the robot is in candle configuration (transport position) the
% values are: [0 0 0 0 0 0 0]
%
% To manage the difference an offset must be added to t2 in all function
% and to the output fro Catia

while(1) %Supporting the two different zeroing positions of Kuka
    Joint2Offset=input('1. Enter Joint 2 offset? \n pi/2 for FRI work (Candle position @ q2=0) , \n 0 for KRL (Candle position @ q2=90)\n');
    if isempty(Joint2Offset)
        Joint2Offset=pi/2;
        disp 'Joint Offset defaulting to FRI'
        break
    elseif Joint2Offset~=0 && Joint2Offset~=pi/2
        disp 'Joint Offset not supported'
    else
        break
    end
end

%__________________________________________________________________________________
%__________________________________________________________________________________





%---------------------------------------------------------------------------------
%---------------------------Robot Parameters--------------------------------------
%---------------------------------------------------------------------------------

qlower=[-170 -120 -170 -120 -170 -130 -170]*pi/180;
qupper=-qlower;
Joints=7; %No of joints
Ant = [0,1,2,3,4,5,6];
sigma= [0,0,0,0,0,0,0];
b= [0,0,0,0,0,0,0];

%Assign r
r= [0.3105,0,0.4,0,0.39,0,0.078];
r7_B=0.078;
rcell=num2cell(r);
[r1, r2, r3, r4, r5, r6, r7]=rcell{:};
% %Assign d
d= [0,0,0,0,0,0,0];
dcell=num2cell(d);
[d1, d2 ,d3, d4, d5, d6, d7]=dcell{:};
gamma= [0,0,0,0,0,0,0];
alpha= [0,pi/2,-pi/2,-pi/2,pi/2,pi/2,-pi/2];
mu= [1,1,1,1,1,1,1];
theta= [0,-pi/2+Joint2Offset,0,0,0,0,0];
%__________________________________________________________________________________
%__________________________________________________________________________________


%---------------------------------------------------------------------------------
%--------Choice 2:           Define tool frame------------------------------------
%---------------------------------------------------------------------------------
while(1) %Supporting the knife as the tool frame, only difference is the task trajectory
    TF=input('\n 2. Enter the offset for tool frame? \n 0 for default, \n Adams knife is 0.1 \n');
    if  TF==0
        %TnE=eye(4);
        disp 'Defaulting to  '
        break
    elseif isempty(TF)  || TF==0.1
        %         TnE=[1 0 0 0
        %              0 1 0 0
        %              0 0 1 0.1
        %              0 0 0 1];
        r7=r7+0.1;
        disp 'New tool frame chosen'
        break
    else
        disp 'Knife not supported '
    end
end
%__________________________________________________________________________________
%__________________________________________________________________________________



Guide_Radius=0.005; % Cutting Cylinder in 5mm
rotdisp=3; % Orientation Representation
% 1=Quaternions [Scalar; Vector]
% 2=Euler Angles
% 3=Angle Axis [angle;axis]
% 4=Roll Pitch Yaw
% 5=DCM Matrix

%---------------------------------------------------------------------------------
%--------Choice 3:           Define Scenario--------------------------------------
%---------------------------------------------------------------------------------


while(1) %Supporting the two different zeroing positions of Kuka
    Choose=input(['\n 3. Choose qinit? \n 0 for all zeros, \n 1 for  nonsingular  known position qinit =[0 0 0 pi/4 0 -pi/4 0] \n 2 for  random within joint limits \n 3' ...
'for closed chain work work (qinit, qinitB) \n 4 for cutting of flexible object \n 5 for cutting of meat like object \n 6 for cutting of meater! like object'...
'\n 7 for robotic cell work']);
    
    if isempty(Choose)
        Choose=7; % Choose default case
    end
    
    switch Choose
        case 0 % All zeros
            qinit=zeros(7,1);
            break
        case 1 % for  nonsingular  known position
            qinit=[0 0 0 pi/4 0 -pi/4 0]';
            T0L=TransMat([0 0.7 0],'t','T');
            break   
        case 2 %  for  random within joint limits 
            qinit=rand(7,1).*(qupper'-qlower')+qlower';
            break   
        case 3 % for closed chain work work (qinit, qinitB)
            qinit=[pi/2 0 0 pi/4 0 -pi/4 0]';
            qinitB=[-pi/2 0 0 pi/4 0 -pi/4 0]';
            ClosedChainInitilization
            break
        case 4 %  for cutting of flexible object
            qinit=[pi/12 -pi/10 -pi/10 pi/2 pi/20 -pi/3 -pi/20]';
            qinitB=[-1.5175   -1.7285   -1.5708   -1.4453   -0.1579   -1.5002   -1.5511]';
            T0L=TransMat([0 0.7 0],'t','T')*TransMat(pi,'rz','T');
            V1init=TransMat([0.38 -0.09 0.3],'t','T');
            V2init=TransMat([.42 0.08 0.3],'t','T');
            break   
        case 5 % for cutting of meat like object 
            qinit=[pi/12 -pi/10 -pi/10 pi/2 pi/20 -pi/3 -pi/20]';
            qinitB=[ -0.0762   -0.9463    2.5850   -1.3710    1.6333   -0.9696   -0.8259]';
            T0L=TransMat([0 0.4 0],'t','T');          
            break 
        case 6 %for cutting of meater! like object
            qinit=[pi/12 -pi/10 -pi/10 pi/2 pi/20 -pi/3 -pi/20]';
            qinitB=[ -0.0871   -0.9326    2.5850   -1.4568    1.5653   -0.9588   -0.7304]';
            qinitC=[ pi/4   pi/10   -pi/10  (3*pi/5)    -pi/4   -pi/3 -pi/20]';
            VP1=[0.6843;1.612e-02;0.28];
           % VP1=[0.7443;1.612e-02;0.3087];
            VP2=[0.3915;1.475e-02;0.3087];
           % VP4=[0.7443;1.612e-02;0.3087];
            %VP3=[0.3915;1.475e-02;0.3087];
            T0L=TransMat([0 0.4 0],'t','T');  
            CameraParameters=[560; 560; 0;  374.4020879	;  180.2622981 ];
			Sdesired=[50;0;400;pi/6];
            break
        case 7 % Robotic Cell
            qinit=[pi/12 -pi/10 -pi/10 pi/2 pi/20 -pi/3 -pi/20]';
            qinitB=[ -0.0871   -0.9326    2.5850   -1.4568    1.5653   -0.9588   -0.7304]';
            qinitC=[ pi/4   pi/10   -pi/10  (3*pi/5)    -pi/4   -pi/3 -pi/20]';
            T0L=TransMat([0 0.4 0],'t','T');   
            T0L_Vision=TransMat([1.0 0 0],'t','T')*TransMat(pi,'rz','T');   
            X4d_C=T40(qinitC);
            X4d_C=X4d_C(1:3,4);
            Task_Dimension=[6,4,4,2];
            VisionrobotCamera=1;
            qinitTable=[0;0];
            % CameraParameters=[fu;  fv;  fs; u0; v0]
            CameraParameters=[560; 560; 0;  374.4020879	;  180.2622981 ];
            Sdesired=[100;0;200;pi/4];
            T0table=[ 0  1  0 0.55
            0  0  -1 -0.05
            -1 0  0 0.1092
            0 0 0 1];
            VP1=[0.6843;1.612e-02;0.3087];
            VP2=[0.3915;1.475e-02;0.3087];
           
            break
        otherwise
           disp 'Try again'
    end
end
%__________________________________________________________________________________
%__________________________________________________________________________________



clc
if isempty(qinitB)
    disp 'All robots have same joint config'
    X4d=T40(qinit);
    X4d=X4d(1:3,4);
    
else 
    
    X4d=T40(qinit);
    X4d=X4d(1:3,4);
    X4d_B=T40(qinitB);
    X4d_B=X4d_B(1:3,4);
    
end

Tinit=T70(qinit);
Pinit=(Tinit(1:3,4));
RepInit=reshape(Tinit(1:3,1:3),9,1);
disp 'vdpts should correspond to visual primitives'
vdpts=240
%vdpts=189
% Initialise the dynamic parameters
DynamicParams
% Initialise the gain parameters
Gains



%X4d=[0.1193949983; 3.1991793366E-002; 0.6909226065];
%X4d_B=[0.0210;-0.3945;0.2477];


% Lets define the secondary criteria from the
% initial joint position

% 
% Table Stuff comment out
% alpha=[0;-pi/2]
% d=[0;-0.1]
% theta=[0;pi/2]
% r=[0;0]
% sigma=[0;0]
% Joints=2


% %%
%
% %In order to find the Jacobian matrix at the tool frame rather than the last joints
% q=qinit
% T= T70(q);% Transformation matrix
% J= J70(q);
% P=TnE(1:3,4);   % Distance to tool frame
% L=T(1:3.1:3)*P; % Change the reference frame to the world one
% Lhat=skew(L);   % Skew the matrix
%
% J0e=[eye(3) -Lhat; zeros(3) eye(3) ]*J; % Change the point of the Jacobian
%
%