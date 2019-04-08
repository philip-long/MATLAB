function [J_right,J_left] = Jacobian(u)
% Function giving Jacobian matrices for the serial arms of the nao robot
% reffered to the torso frame
% The right arm jacobian tool frame is located at the same frame as the
% left arm tool frame Frame 11=Frame 10


%%   THIS IS FOR KINEMATIC VERSION 3.3 OF NAO T14

%%
global sigma alpha d Joint_offset r Angle_offset theta 
global r1 r3 r5 r6 b1 b6  d3 d8 T511

qR=u(1:5);
qL=u(6:10);

n=5;


%Transformation matrix from end effector frame to joint 5

T0j=eye(4); %Initialise T06 as Identity
T_left=eye(4);
T_right=eye(4);
J_right=eye(6,n);
J_left=eye(6,n);
AJ=zeros(3,n+1);
PJ=zeros(3,n+1);
for i=0:1%Does Both Arms
    
    if i==0 %Right Arm 
        q=qR;
        T0j=[1 0 0 0
            0 1 0 r1
            0 0 1 b1
            0 0 0 1];
        T5E=T511;
        d=[0 0 d3 0 0]; 
    else %%Selects Right Left joint variables and transformation matrix to world
        
        q=qL;
        T0j=[1 0 0 0
            0 1 0 r6
            0 0 1 b6
            0 0 0 1];
        T5E=eye(4);
        d=[0 0 d8 0 0]; 
    end
        
    for j=1:n
        
        if sigma(j)==1 %Checking if joint is prismatic or revolute
            r(j)=q(j)+Joint_offset(j);
            
            theta(j)=Angle_offset(j);
        else
            
            theta(j)=q(j)+Angle_offset(j);
            
            r(j)=Joint_offset(j);
        end
        
        Tj=[cos(theta(j)) -sin(theta(j)) 0 d(j)
            cos(alpha(j))*sin(theta(j)) cos(alpha(j))*cos(theta(j)) -sin(alpha(j)) -r(j)*sin(alpha(j))
            sin(alpha(j))*sin(theta(j)) sin(alpha(j))*cos(theta(j)) cos(alpha(j)) r(j)*cos(alpha(j))
            0 0 0 1]; %Using modified DH parameters to find Transformation Matrix
        
        T0j=T0j*Tj; %Successively Multplying Transformation matrices to find 0T1 0T2 0T3 0T4 etc.
        T0j;
        p=T0j(1:3,4); % For each Transformation Matrix extract Displacement Px Py Pz
        a=T0j(1:3,3); % For each Transformation Matrix extract ax ay az
        AJ(:,j)=a;
        PJ(:,j)=p;
    end
    
    T0Tn=T0j*T5E;  %Final Matrix is 0T6
   
    p=T0Tn(1:3,4); % For each Transformation Matrix extract Displacement Px Py Pz
    a=T0Tn(1:3,3); % For each Transformation Matrix extract ax ay az
    AJ(:,n+1)=a;
    PJ(:,n+1)=p;
    
    if i==0
        
        T_right=T0Tn;
        for k=1:n
            askew=[0        -AJ(3,k) AJ(2,k)
                AJ(3,k)   0        -AJ(1,k)
                -AJ(2,k)  AJ(1,k)       0];
            J_right(:,k)=[sigma(k)*AJ(:,k)+(1-sigma(k))*askew*(PJ(:,n+1)-PJ(:,k))
                (1-sigma(k))*AJ(:,k)];
        end
    else
        
        T_left=T0Tn;
        J_left=eye(6,n);
        for k=1:n
            askew=[0        -AJ(3,k) AJ(2,k)
                AJ(3,k)   0        -AJ(1,k)
                -AJ(2,k)  AJ(1,k)       0];
            J_left(:,k)=[sigma(k)*AJ(:,k)+(1-sigma(k))*askew*(PJ(:,n+1)-PJ(:,k))
                (1-sigma(k))*AJ(:,k)];
        end
    end
    
end
J_left;
J_right;


