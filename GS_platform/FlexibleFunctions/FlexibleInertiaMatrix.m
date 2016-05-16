function A = FlexibleInertiaMatrix( q_e)


% FlexibleInertiaMatrix Returns the flexible inertia matrix in the platform frame
% with respect to the platform origin. 
global M MS_r b_k Beta_k lambda_ik I_re I_er I_ee m_ee
global ModesNo I_rr
% Using Boyer Notation from Book

    MS_hat=skew([0;0;0]);
celldisp(b_k)
    for i=1:ModesNo
        MS_hat=MS_hat+(skew(b_k{i}).*q_e(i))
    end
    
MS_hat=MS_hat+skew(MS_r);

MS_de=[b_k{1} b_k{2} b_k{3}];


    sumI_8=zeros(3);
    sumI_9=zeros(3);
    for i=1:ModesNo
       sumI_8= sumI_8+(I_re{i}+I_er{i})*q_e(i);
       for j=1:ModesNo
           sumI_9=sumI_9+I_ee{i,j}*q_e(i)*q_e(j);
       end
    end
    sumI_8
    sumI_9
I =I_rr-(sumI_8)-sumI_9;

MS_re=zeros(3,3);
for i=1:ModesNo
    sumlambda_ik=zeros(3,1);
    for j=1:ModesNo
        sumlambda_ik=sumlambda_ik+lambda_ik{j}*q_e(j);
    end
   MS_re(:,i)=(Beta_k{i}+sumlambda_ik);
end


%%
A=[M                transpose(MS_hat)     MS_de
   MS_hat                 I               MS_re
   transpose(MS_de) transpose(MS_re)     m_ee];
end

