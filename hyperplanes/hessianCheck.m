% http://www.orocos.org/node/829
% I have checked the hessian symbolically and this is working!! :0
clear all,clc,close all
load('Hessian_mats')
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))

qdot_arm_max=2;
qdot_arm_min=-4;

%
qpos=randRange(-pi,pi,7)



qpos=randRange(-pi,pi,7);
joint=randi(7);
%joint=1;
J=J70(qpos);

step=0.01;
for q3=0:step:pi
    Jlast=J;
    
    qpos(joint)=q3;
    T=T70(qpos);
    J=J70(qpos);

    
       
    
    for i=1:size(J,2) % going throught all columns 
        for j=1:size(J,2) % going through all cols again
           % if(i<j)
           twist_j=J(:,j);
           d_twist_dqi{j}(:,i)=[skew(J(4:6,i)) zeros(3)
                          zeros(3) -skew(J(4:6,i))] * twist_j;
           if(i<j)
               d_twist_dqi{j}(1:3,i)=[skew(J(4:6,i)) zeros(3)]*twist_j;
               d_twist_dqi{j}(4:6,i)=zeros(3,1);
           elseif(i>j)
              d_twist_dqi{j}(1:3,i)=[skew(J(4:6,j)) zeros(3)]*J(:,i); 
           end
           
            
            Htensor(:,i,j)=[skew(J(4:6,i))*J(1:3,j)
                skew(J(4:6,i))*J(4:6,j)];
           % end
        end
    end
    
     % [ dx/dq1dq1 dx/dq1dq2 dx/dq1dq3 ... dx/dq1dqn] 
     % [ dx/dq2dq1 dx/dq2dq2 dx/dq2dq3 ... dx/dqdqn]
    numerical_gradient=(J-Jlast)/step;
    
    rows=randi(6);
    cols=randi(7);
    numerical_gradient(rows,cols)
    Htensor(rows,cols,joint)
    Htensor(rows,joint,cols)
       
     if(norm(numerical_gradient(rows,cols) - Htensor(rows,joint,cols))>0.01)
                      q1=qpos(1);     q2=qpos(2);     q3=qpos(3);
       q4=qpos(4);    q5=qpos(5);     q6=qpos(6);     q7=qpos(7);
         rows
         cols
         joint

     end

end