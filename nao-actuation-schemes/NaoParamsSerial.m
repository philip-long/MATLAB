%Here is defined the joint parameters as given by the documentation in Nao
%Link Information
clear all
close all
clc
global sigma alpha d Joint_offset r Angle_offset theta T5E
global qinit q_upper q_lower
global Qactuated Qpassive Qcut
global r1 r3 r5 r6 b1 b6  d3 d8 r8 r10
global g11 b11  alpha11 d11 t11 r11 T511
global CfgActs Joints
load ConfigActs


Joints=5;
%Robot Parameters  
%===================================================
UpperArmLength=105;LowerArmLength=55.95; %LowerArmLength=50.55 V3.2 or 55.55 V3.3
ShoulderOffsetZ=100;ShoulderOffsetY=98;
HandOffsetX=57.75;HandOffsetZ=12.31;ElbowOffset=15;
alpha=[-pi/2 pi/2 pi/2 -pi/2 pi/2];
d=zeros(1,5);
Joint_offset=[0 0 UpperArmLength 0 LowerArmLength];
r=zeros(1,5);
Angle_offset=[0 pi/2 0 0 0];
theta=zeros(1,5);
sigma=[0 0 0 0 0];
g1=0; % No angle change from base frame
g6=0; % No angle change from base frame

b1=ShoulderOffsetZ;% initial offset from world frame RArm, moving x to u
b6=ShoulderOffsetZ;% initial offset from world frame LArm, moving x to u

r1=-ShoulderOffsetY;% initial offset from world frame RArm
r6=ShoulderOffsetY; % initial offset from world frame LArm

d1=0;%Initial Offset for Right arm
d3=-ElbowOffset;%Elbow Offset for Right arm
d6=0;%Initial Offset for Left arm
d8=ElbowOffset;%Elbow Offset for Right arm
r3=UpperArmLength;%RArm
r5=LowerArmLength;%RArm
r8=UpperArmLength;%LArm
r10=LowerArmLength;%LArm
r=[r1 0 r3 0 r5 r6 0 r3 0 r5];
d=[d1 0 d3 0 0 d6 0 d8 0 0];
q_upper=[2.0857     0.3142      2.0857      1.5446      1.8238       2.0857     1.3265      2.0857      -0.0349      1.8238];
q_lower=[-2.0857    -1.3265     -2.0857     0.0349      -1.8238      -2.0857    -0.3142     -2.0857     -1.5446     -1.8238];

%===================================================

%Tool Offset
%================================================

disp 'Tool frame Offset'
T5E =[0.0000    1.0000         0    0.0000
     -0.0000    0.0000    1.0000   -HandOffsetZ
      1.0000   -0.0000    0.0000    HandOffsetX
           0         0         0    1.0000];
       
%T5E=eye(4); 




%%=========================================================================
%%=========================================================================
%%=========================================================================
%%=========================================================================
%

%Actuated Joints
%===============================================
 %Choose Actuated Joints
disp 'Chosen Actuation Scheme'
Qactuated=[1,2,4,6]
Qpassive=[3,5,7,8,9]
Qcut=[10]
for i=1:10
    if any(ismember(Qactuated,i)) && any(ismember(Qpassive,i)) || any(ismember(Qactuated,i)) && any(ismember(Qcut,i)) || any(ismember(Qpassive,i)) && any(ismember(Qcut,i))
        disp 'Error double definition of joint'
        pause()
    end
end
%===============================================
for i =1:length(CfgActs)
    if all(ismember(Qactuated,CfgActs(i,:)))
        disp '                                   '
        disp '***********************************'
        disp '-----------------------------------'
        disp 'Scheme is Archetectural Singularity'
        disp '-----------------------------------'
        disp '***********************************'
        disp '                                   '
    end
end


%% Initialization of geometric parameters based on joint values at a predetermined grasp position
qinit=rand(1,10)*2*pi-(pi);
%qinit(6)=qinit(1)
%qinit=[0 0 0 -pi/4 0 0 0 0 -pi/2 0 ];
%qinit(7)=-qinit(2)
qinit(4)=0 %Serial Singularity Rarm
%qinit(9)=0 %Serial Singularity Larm
%qinit(2)=atan(r3/d3) %Serial Singularity Rarm
%qinit(7)=atan(r8/d8) %Serial Singularity Rarm


[T_Output] = ClosedLoopGeo(qinit);
T_right=T_Output(1:4,1:4);
T_left=T_Output(5:8,1:4);
%T511=inv(T_right)*T(T_left)
T511=T_right\T_left;
%[g11 b11 d11 alpha11 t11 r11]=extractDH(T511)
[g11 b11  alpha11 d11 t11 r11]=param_fT0(T511);

