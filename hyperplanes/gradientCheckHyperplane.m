% Attempt 1 to get analytical gradient

clear all,clc,close all
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/robot-functions'))
addpath(genpath('~/tbxmanager'))

qdot_arm_max=2;
qdot_arm_min=-4;

% Capcity margin
PA_desired=[eye(3) ;-eye(3)];
PB_desired=[0.1 ; 0.1 ;0.1 ;0.4;0.3;0.2];
desired_twist=Polyhedron(PA_desired,PB_desired);

active_joint=1:4; % Only considering 4 joint for illustrative case
sigmoid_factor=200;
%%
G=[]
qpos=randRange(-pi,pi,7);
joint=randi(4);

for q3=0:0.01:pi
    
    qpos(3)=q3;
    T=T70(qpos);
    J=J70(qpos);
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
    HE=getHessian(JE(:,active_joint));
    JE=JE(1:3,active_joint);
    
    %
    % Take a combination of d − 1 linearly independdoft cable unit wrdofches w i that can
    % define n, a unit vector perpendicular to a hyperplane that includes all these unit wrdofches.
    
    % Jacobian is a d x n matrix i.e. in this case 3 times 4; d=3 n=4
    n_joints=size(JE,2);  % linearly independent directions
    m=size(JE,1); % Number of degrees of freedom
    
    % Return combination of selected joints and selected joints
    [N,Nnot]=getDofCombinations(active_joint,m);

    
    
    % We start from an initial hyperplane that includes the origin and whose normal is n. The unit
    % wrdofches w i chosdof at step 1 define the oridoftation of the two faces parallel to this hyperplane. The remaining w j
    % will define the position of the faces, or how the initial hyperplane is shifted along n to coincide with the two support-
    % ing hyperplanes.
    
    n=zeros(size(N,1),3);
    d_n_dq=zeros(size(N,1),3); % Gradient of n
    hplus=zeros(size(N,1),1);
    d_hplus_dq=zeros(size(N,1),1);
    hminus=zeros(size(N,1),1);
    d_hminus_dq=zeros(size(N,1),1);
 
    qdot_max=ones(4,1)*qdot_arm_max;
    qdot_min=ones(4,1)*qdot_arm_min;
    deltaq=qdot_max-qdot_min;
    
    
    
    for i = 1:size(N,1)
        v1=JE(:,N(i,1));
        v2=JE(:,N(i,2));
        d_v1_dq=HE{joint}(1:3,N(i,1)); % Gradient with respect to one joint
        d_v2_dq=HE{joint}(1:3,N(i,2));
    
        n(i,:)=cross(v1,v2)/norm(cross(v1,v2)); % This is ok so far
        d_n_dq(i,:)= getGradientn(v1,v2,d_v1_dq,d_v2_dq);  % Gradient with respect to one joint
        
        %
        JE_remaining=JE(:,Nnot(i,:));
        
        hplus(i)=0.0;
        d_hplus_dq(i)=0.0;
        hminus(i)=0.0;
        d_hminus_dq(i)=0.0;
        
        for j=1:(n_joints-(m-1))
            vk=JE(:,Nnot(i,j));    
            d_vk_dq=HE{joint}(1:3,Nnot(i,j));
            ntvk=vk'*n(i,:)';
            d_ntvk_dq=(d_n_dq(i,:)*vk) +n(i,:)*d_vk_dq;
            
            dsig_nt_vk_dq=sigmoidGradient(ntvk,200)*d_ntvk_dq;
            
            d_hplus_dq(i)=d_hplus_dq(i)+(sigmoidGradient(ntvk,200)*d_ntvk_dq*deltaq(Nnot(i,j))*ntvk) + sigmoid(ntvk,sigmoid_factor)*deltaq(Nnot(i,j))*d_ntvk_dq;
            d_hminus_dq(i)=d_hminus_dq(i)+(sigmoidGradient(-ntvk,200)*d_ntvk_dq*deltaq(Nnot(i,j))*ntvk) + sigmoid(-ntvk,sigmoid_factor)*deltaq(Nnot(i,j))*d_ntvk_dq;

            hplus(i)=hplus(i)+sigmoid(ntvk*sigmoid_factor)* deltaq(Nnot(i,j))*ntvk;
            hminus(i)=hminus(i)+sigmoid(-ntvk*sigmoid_factor)* deltaq(Nnot(i,j))*ntvk;
        end
        
    end
    
    % Write code to check the gradients of each individual component
    
   
    Gamma_i=hplus(3)+ n(3,:)*JE*qdot_min - (n(3,:) * desired_twist.V(1,:)');
    G=[G;Gamma_i];
end

plot(0:0.01:pi,G)