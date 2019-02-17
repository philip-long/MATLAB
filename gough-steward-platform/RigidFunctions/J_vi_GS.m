function J = J_vi_GS(X,q,l)
% J_vi_GS transform the Vm, the velocity of the platform to the attacment point
%
%   vi=J_vi*Vm
%   vi=[1 \hat{L}_{im}] Vm

T0M=GS_X_2_T(X);

global PM1 PM2 PM3 PM4 PM5 PM6 

switch l
    case 1
        PLM=PM1; 
    case 2
        PLM=PM2;
    case 3
        PLM=PM3;
    case 4
        PLM=PM4;
    case 5
        PLM=PM5;
    case 6    
        PLM=PM6;
    otherwise
        disp 'defaulting to leg 1'
        PLM=PM1;    
end


L0L=T0M(1:3,1:3)*PLM;

J=[eye(3) -skew(L0L(1:3))];

    




end