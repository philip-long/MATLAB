function [ c_ext ] = getExtendedManipulability( q, ObjectPositions ,q_max,q_min)
%getExtendedManipulability Returns Varehenkamps extended manipulability
%measure for a 2DOF planar arm
Hyperoctant=generateVertexSet([1 1],[-1 -1]);

%% Kinematic data
wTr=getTransforms(q);
wJr=getJacobians(q);  % Get all Jacobians
q_arm_max=q_max;
q_arm_min=q_min;

n_T_ee=eye(4); n_T_ee(1,4)=0.55;   % terminal point
% Define Control point and publish frame
% This is necessary  since my chain ends with the last joint
[wTe,wJe]=getEEInfo(wTr{2},wJr{2},n_T_ee,'world',false);
Je=wJe([1 2],:);
wTr{3}=wTe;
wJr{3}=wJe;


%% Joint Limits
h=zeros(2,1);
pjplus=zeros(2,1);
pjminus=zeros(2,1);
cart_dirs=2;

for k=1:length(Hyperoctant)
    L{k}=zeros(cart_dirs,2);
    for i=1:cart_dirs
        for j=1:2
            numerator=(q(j)-q_arm_min(j))*(q(j)-q_arm_min(j)) * (2*q(j) - q_arm_max(j) - q_arm_min(j));
            denominator=4*(q_arm_max(j)-q(j))*(q_arm_max(j)-q(j))*(q(j)-q_arm_min(j) )*(q(j)-q_arm_min(j) );
            h(j)=numerator/denominator;
            
            if(  (abs(  q(j)-q_arm_min(j) )) >(abs(  q_arm_max(j)-q(j)) )  )
                pjminus(j)=1;
                pjplus(j)=1/sqrt(1+abs(h(j)));
            else
                pjminus(j)=1/sqrt(1+abs(h(j)));
                pjplus(j)=1;
            end
            
            if(  (  sign(Je(i,j)) * sign(Hyperoctant(k,i)) )<0 )
                L{k}(i,j)    = pjminus(j);
            else
                L{k}(i,j)    = pjplus(j);
                
            end
        end
    end
end

%%

%  In Vahremkamp et al. for each object we find the closest
% point on the manipulator


%plotObjects(ObjectPositions,2);
min_distance_=100;
%  In Vahremkamp et al. for each object we find the closest
% point on the manipulator
for o=1: size(ObjectPositions,1)
    pt=wTr{end}(1:3,4);
      obj_point=ObjectPositions(o,:)';
    vec=[obj_point-pt;0;0;0];
    d=norm(vec);
    if(d<min_distance_)
                min_distance_=d;
                candidate_v=vec;
                o_candidate=o;
    end
end


alpha=1;
Beta=1;
d=min_distance_;
dlp_dld=exp(-alpha*d)*(d^-Beta)*((Beta*(1/d))+alpha);
dld_dlq=(1/d)*(transpose(wJr{end})*candidate_v)';
dlp_dltq=dlp_dld*dld_dlq;


for k=1:length(Hyperoctant)
    O{k}=zeros(cart_dirs,2);
    for i=1:cart_dirs
        for j=1:2                        

            if(candidate_v(i)<=0 && i<3)
                ojminus(j)=1/sqrt(1+abs(dlp_dltq(j)));
            else
                ojminus(j)=1;
            end
            
            if(candidate_v(i)>0 && i<3)
                ojplus(j)=1/sqrt(1+abs(dlp_dltq(j)));
            else
                ojplus(j)=1;
            end
            
            if(  (sign(Hyperoctant(k,i)) )<0 )
                O{k}(i,j)    = ojminus(j);
            else
                O{k}(i,j)    = ojplus(j);
                
            end
            
        end
    end
end



%%
sing_values=[];
for k=1:length(Hyperoctant)
    for i=1:cart_dirs
        for j=1:2
            J_ext{k}(i,j)=L{k}(i,j)*O{k}(i,j)*wJr{end}(i,j);
        end
    end
    sing_values=[sing_values;svd(J_ext{k}(1:2,:))];
end

c_ext=min(sing_values)/max(sing_values);

end

