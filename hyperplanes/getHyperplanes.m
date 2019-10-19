function [n,hplus,hminus,d_n_dq,d_hplus_dq,d_hminus_dq] = getHyperplanes(JE,HE,joint,deltaq,active_joint)
%getHyperplanes Gets the hyperplanes parameters
%   gets the parameters that can be used to construct the cartesian
%   velocitity polytopes using hyperplane method as well as their gradients, 


% Jacobian is a d x n matrix i.e. in this case 3 times 4; d=3 n=4
n_joints=size(JE,2);  % linearly independent directions
m=size(JE,1); % Number of degrees of freedom

% Return combination of selected joints and selected joints

[N,Nnot]=getDofCombinations(active_joint,m);

% Take a combination of d − 1 linearly independdoft cable unit wrdofches w i that can
% define n, a unit vector perpendicular to a hyperplane that includes all these unit wrdofches.

n=zeros(size(N,1),m);
d_n_dq=zeros(size(N,1),3); % Gradient of n
hplus=zeros(size(N,1),1);
d_hplus_dq=zeros(size(N,1),1);
hminus=zeros(size(N,1),1);
d_hminus_dq=zeros(size(N,1),1);

% the steepness of the sigmoid function
sigmoid_slope=200;


% We start from an initial hyperplane that includes the origin and whose normal is n. The unit
% wrdofches w i chosdof at step 1 define the oridoftation of the two faces parallel to this hyperplane. The remaining w j
% will define the position of the faces, or how the initial hyperplane is shifted along n to coincide with the two support-
% ing hyperplanes.





for i = 1:size(N,1)
    v1=JE(:,N(i,1));
    v2=JE(:,N(i,2));
    d_v1_dq=HE{joint}(1:3,N(i,1)); % Gradient with respect to one joint
    d_v2_dq=HE{joint}(1:3,N(i,2));
    
    n(i,:)=cross(v1,v2)/norm(cross(v1,v2)); % This is ok so far
    d_n_dq(i,:)= getGradientn(v1,v2,d_v1_dq,d_v2_dq);  % Gradient with respect to one joint
       
    hplus(i)=0.0;
    d_hplus_dq(i)=0.0;
    hminus(i)=0.0;
    d_hminus_dq(i)=0.0;
    
    for j=1:(n_joints-(m-1))
        vk=JE(:,Nnot(i,j));
        d_vk_dq=HE{joint}(1:3,Nnot(i,j));
        ntvk=vk'*n(i,:)';
        d_ntvk_dq=(d_n_dq(i,:)*vk) +n(i,:)*d_vk_dq;

        d_hplus_dq(i)=d_hplus_dq(i)+(sigmoidGradient(ntvk,sigmoid_slope)*d_ntvk_dq*deltaq(Nnot(i,j))*ntvk) + sigmoid(ntvk,sigmoid_slope)*deltaq(Nnot(i,j))*d_ntvk_dq;
        d_hminus_dq(i)=d_hminus_dq(i)+(sigmoidGradient(-ntvk,sigmoid_slope)*d_ntvk_dq*deltaq(Nnot(i,j))*ntvk) + sigmoid(-ntvk,sigmoid_slope)*deltaq(Nnot(i,j))*d_ntvk_dq;
        
        hplus(i)=hplus(i)+sigmoid(ntvk*sigmoid_slope)* deltaq(Nnot(i,j))*ntvk;
        hminus(i)=hminus(i)+sigmoid(-ntvk*sigmoid_slope)* deltaq(Nnot(i,j))*ntvk;
    end
    
end


end

