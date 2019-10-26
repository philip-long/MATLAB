function [c,ceq,c_grad,ceq_grad] = gammaConstraint(q,qdot_max,qdot_min,vertices)
%gammaConstraint Constrain the hyperplanes with gradient

deltaq=qdot_max-qdot_min;
% Get kinematics
T=T70(q);
J=J70(q);
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;

% Get Hessian
HE=getHessian(JE);
JE=JE(1:3,:);
[n,hp,hm,n_grad,hp_grad,hm_grad]=getHyperplanes(JE,HE,deltaq);
%
for vertex=1:size(vertices,1)
    Gamma_plus(:,vertex)=hp + n*JE*qdot_min -  (n * vertices(vertex,:)'); % Valid solution is \geq 0
    Gamma_minus(:,vertex)=hm + n*JE*qdot_min+(n * vertices(vertex,:)');   % Valid solution is \leq 0
    for joint=1:size(JE,2)
        Gamma_p_gradient{joint}(:,vertex)=hp_grad{joint} + (n_grad{joint} *JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) - (n_grad{joint} *vertices(vertex,:)');
        Gamma_m_gradient{joint}(:,vertex)=hm_grad{joint}  + (n_grad{joint} *JE*qdot_min)  + (n*HE{joint}(1:3,:)*qdot_min) + (n_grad{joint} *vertices(vertex,:)');
    end
end


c=[ - Gamma_plus(:)
    Gamma_minus(:)];
%c=[ - Gamma_plus(:)];
ceq=[];
c_grad=[];
ceq_grad=[];


 for joint=1:size(JE,2)
     c_grad=[c_grad; -Gamma_p_gradient{joint}(:)', Gamma_m_gradient{joint}(:)'];
 end

end

