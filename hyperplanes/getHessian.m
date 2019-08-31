function [H] = getHessian(J)
%getHessian get kinematic Hessian matrix for a robot of revolute joints
%   Input  Jacobian J= [ v_1 v_2 ....v_n] matrix of unit twists 6 x n
%   Output Hessian Tessor nxmxn H{j}=[ dv_1/dq_j dv_2/dq_j dv_n/dq_j ] 

H=cell(1,size(J,2));
  for i=1:size(J,2) % going throught all columns 
        for j=1:size(J,2) % going through all cols again
           twist_j=J(:,j);
           twist_i=J(:,i);
           if(i<j)
               H{j}(:,i)=[skew(J(4:6,i)) zeros(3); zeros(3,6)]*twist_j;
           elseif(i>=j)
                H{j}(:,i)=[skew(J(4:6,j)) zeros(3); zeros(3) skew(J(4:6,j))] * twist_i;
           end
        end
  end
    
  
end

