%===GEOMETRIC MODEL OF A GENERAL SERIAL ROBOT===========
function [T_Output] = ClosedLoopGeo(u)

%%   THIS IS FOR KINEMATIC VERSION 3.3 OF NAO T14
 
    %THIS FUNCTION IS SPECFICALLY FOR THE INITIALIZATION OF CLOSED LOOP
    %CALCULATIONS

global r1 r3 r5 r6 b1 b6  d3 d8...
    alpha 




sigma=zeros(1,10);
alpha=[-pi/2 pi/2 pi/2 -pi/2 pi/2];
Joint_offset=[0 0 r3 0 r5];
Angle_offset=[0 pi/2 0 0 0];

qR=u(1:5);
qL=u(6:10);
n=5;

T_right=eye(4);
T_left=eye(4);
T5E=eye(4);


for i=0:1%Does Both Arms
    
    if i==0 %Right Arm
        q=qR;
        T0j=[1 0 0 0
            0 1 0 r1
            0 0 1 b1
            0 0 0 1];
        d=[0 0 d3 0 0];     
    else %%Selects Right Left joint variables and transformation matrix to world
        q=qL;
        T0j=[1 0 0 0
            0 1 0 r6
            0 0 1 b6
            0 0 0 1];
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
        
        
    end
    
       
    T0Tn=T0j*T5E;  %Final Matrix is 0T6
    
    if i==0
        
        T_right=T0Tn;
        
    else 
        
        T_left=T0Tn;
    end
    
end


T_Output=[T_right;T_left];
