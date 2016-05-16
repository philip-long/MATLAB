%
%
% Condition for constraint singularity q1=0, q6=0, q2=atan(r3/(-d3)), q7=atan(r7/d7) 
% In fact its independant of the first joint variables as long as they are
% equal
%

%%
% ------------------------------------------------------------
%  Initialisation of variables and matrices
% ------------------------------------------------------------


clc
clear all
syms r3 d3 TorsoY TorsoZ
% Robot Link Parameters
UpperArmLength=105.00; LowerArmLength=55.95; ShoulderOffsetZ=100; ShoulderOffsetY=98.0; HandOffsetX=57.75; HandOffsetZ=12.31; ElbowOffset=15.0;
%UpperArmLength=r3; ElbowOffset=d3;ShoulderOffsetZ=TorsoZ;ShoulderOffsetY=TorsoY;
%Preallocation of Matrices and cells for speed
AxisOfJoints=cell([5 1]); VectorToOrigin=cell([5 1]); TwistRight=cell([5 1]); TwistLeft=cell([5 1]); Mat1=sym(zeros(5,6));Mat2=sym(zeros(5,6));
VectorToOrigin2=cell([5 1]);TwistRight2=cell([5 1]);
%Matrix which is postmultpied by twist  
% Eq 2.3 Kong Gosselan Pi=[ 0     I_{3}]
%                         [ I_{3} 0    ]
PreMul=[zeros(3) eye(3);eye(3) zeros(3)];


%%
% ------------------------------------------------------------
% Cell 1-  Right Arm ctrl+enter to run
% -------------------------------------------------------------

%Joint variables
syms q1 q2 q3 q4 q5 
 q=[q1 q2 q3 q4 q5];
 
 
% DH PARAMETERS
% Right Arm Parameters
alpha=[-sym(pi)/2 sym(pi)/2 sym(pi)/2 -sym(pi)/2 sym(pi)/2]; d=[0 0 -ElbowOffset 0 0];r=[-ShoulderOffsetY 0 UpperArmLength 0 LowerArmLength];theta=[0 sym(pi)/2 0 0 0];sigma=[0 0 0 0 0];

% Function creating transformation matrices
% Returns Transforms{1}=T01
% Returns Transforms{2}=T02
% Returns Transforms{3}=T03 and so on

[Transforms]=GENJAC(sigma,alpha,d,theta,r,q,5); 


for i=1:5
  
    % Cycles through all the transformation matrices
    
   AxisOfJoints{i}=Transforms{i}(1:3,3); % Consists of the Z axis of the transformation matrix
    
   VectorToOrigin{i}=Transforms{i}(1:3,4); % Vector from origin to joint axis in present state
   
    
    %  $=  [    s     ]= [  s   ]
    %      [s x r + hs]  [s x r ]
    % since they are all rotation joints h =0
    
    TwistRight{i}=[  AxisOfJoints{i}
                     cross(AxisOfJoints{i},-VectorToOrigin{i})];
    
                 
                 
    % This repetition is to find the twists of the system when referred to the speherical joint intersection             
    %1) Find the transformation matrix from joint i to spherical intersection
    % T3i=transpose(T03)*T0i

    TiSphere=inv(Transforms{3})*Transforms{i};
    
    VectorToOrigin2{i}=simple(TiSphere(1:3,4)); % Vector from TiSphere to joint axis in present state
    
  
    
    %  $=  [    s     ]= [  s   ]
    %      [s x r + hs]  [s x r ]
    % since they are all rotation joints h =0
    
    TwistRight2{i}=[  AxisOfJoints{i}
                     cross(AxisOfJoints{i},-VectorToOrigin2{i})];     
    %MAT1(i)=[0 0 0 1 0 0]    [    s(i)      ]
    %        [0 0 0 0 1 0]    [ s(i)  x  r(i)]
    %        [0 0 0 0 0 1]
    %        [1 0 0 0 0 0]  *  
    %        [0 1 0 0 0 0]
    %        [0 0 1 0 0 0]
    %
    %
    %
   
    Mat1(i,:)=PreMul*TwistRight{i}; % Creates Matrix that when multiplied by reciprocal twist gives zeros
                                    % Automtically transposes result to
                                    % place in matrix row
    
end
%Screw reciprocal to that of twists of RArm is null space of this matrix
% In serial singularity i.e q4=0 null space grows

for i=1:5
    MatofTwists(:,i)=TwistRight{i};
end
for i=1:5
    MatofTwists2(:,i)=TwistRight2{i};
end

Tau_RArm=null(Mat1);
Pright=VectorToOrigin;

%%
% ------------------------------------------------------------
% Cell 1-  Left Arm ctrl+enter to run
% -------------------------------------------------------------

%Repeat same process for LArm except with different DH parameters
syms q6 q7 q8 q8 q10
alpha=[-sym(pi)/2 sym(pi)/2 sym(pi)/2 -sym(pi)/2 sym(pi)/2]; d=[0 0 ElbowOffset 0 0];r=[ShoulderOffsetY 0 UpperArmLength 0 LowerArmLength]; theta=[0 sym(pi)/2 0 0 0]; sigma=[0 0 0 0 0];

% q=[0.699 -0.31 -0.5261 -0.37826 -1.044];
q=[q6 q7 q8 q8 q10];
%q=zeros(1,5)

%Transformation matrices
[Transforms]=GENJAC(sigma,alpha,d,theta,r,q,5);


%Vector to Origins

for i=1:5
    %Using equivalent angle axis theorom as eignvectors of Sym matrices cannot be found by
    %Matlab
    
    AxisOfJoints{i}=Transforms{i}(1:3,3);    
    VectorToOrigin{i}=Transforms{i}(1:3,4); % Vector from origin to joint axis in present state
    
    
    
    %  $=  [    s     ]= [  s   ]
    %      [s x r + hs]  [s x r ]
    % since they are all rotation joints h =0
    
    TwistLeft{i}=[AxisOfJoints{i}
              cross(AxisOfJoints{i},-VectorToOrigin{i})]; 
    
          
    %MAT1(i)=[0 0 0 1 0 0]    [    s(i)      ]
    %        [0 0 0 0 1 0]    [ s(i)  x  r(i)]
    %        [0 0 0 0 0 1]
    %        [1 0 0 0 0 0]  *  
    %        [0 1 0 0 0 0]
    %        [0 0 1 0 0 0]
    %
    %
    %
   
    Mat2(i,:)=PreMul*TwistLeft{i}; % Creates Matrix that when multiplied by reciprocal twist gives zeros
    
end

Tau_LArm=null(Mat2);

%% Normalise the screws
% TRD=simple(Tau_RArm)
% TLD=simple(Tau_LArm)
% 
% sr=Tau_RArm(1:3);
% sl=Tau_LArm(1:3);
% srnorm=sqrt(sum((sr.^2)));
% slnorm=sqrt(sum((sl.^2)));
% srN=sr/srnorm;
% slN=sl/slnorm;
% Tr=simple([srN;Tau_RArm(4:6)]);
% Tl=simple([slN;Tau_LArm(4:6)]);
% Tr % with first vector normalised
% Tl % with first vector normalised
% 
% 
% %Check if its pure force if zero then lambda=0 and its pure force
% transpose(srN)*Tau_RArm(4:6)
% transpose(slN)*Tau_LArm(4:6)