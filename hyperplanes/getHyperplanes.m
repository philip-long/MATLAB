function [n,hplus,hminus,d_n_dq,d_hplus_dq,d_hminus_dq] = getHyperplanes(JE,HE,deltaq,active_joint)
%getHyperplanes Gets the hyperplanes parameters
%   gets the parameters that can be used to construct the cartesian
%   velocitity polytopes using hyperplane method as well as their gradients, 

% Jacobian is a d x n matrix i.e. in this case 3 times 4; d=3 n=4
n_joints=size(JE,2);  % linearly independent directions
m=size(JE,1); % Number of degrees of freedom

if(nargin<4)
    active_joint=1:n_joints;
else
    n_joints=length(active_joint);
end

% Return combination of selected joints and selected joints

[N,Nnot]=getDofCombinations(active_joint,m);

% Take a combination of d − 1 linearly independdoft cable unit wrdofches w i that can
% define n, a unit vector perpendicular to a hyperplane that includes all these unit wrdofches.

n=zeros(size(N,1),m);
hplus=zeros(size(N,1),1);
hminus=zeros(size(N,1),1);

h_plus_save=zeros(size(N,1),1);
h_minus_save=zeros(size(N,1),1);

d_n_dq=cell(length(active_joint),1);
d_hplus_dq=cell(length(active_joint),1);
d_hminus_dq=cell(length(active_joint),1);




% the steepness of the sigmoid function
sigmoid_slope=200;


% We start from an initial hyperplane that includes the origin and whose normal is n. The unit
% wrdofches w i chosdof at step 1 define the oridoftation of the two faces parallel to this hyperplane. The remaining w j
% will define the position of the faces, or how the initial hyperplane is shifted along n to coincide with the two support-
% ing hyperplanes.


for i = 1:size(N,1)
    v1=JE(:,N(i,1));
    v2=JE(:,N(i,2));
    n(i,:)=cross(v1,v2)/norm(cross(v1,v2)); % This is ok so far
    
    for joint=active_joint
    d_v1_dq{joint}=HE{joint}(1:3,N(i,1)); % Gradient with respect to one joint
    d_v2_dq{joint}=HE{joint}(1:3,N(i,2));       
    d_n_dq{joint}(i,:)= getGradientn(v1,v2,d_v1_dq{joint},d_v2_dq{joint});  % Gradient with respect to one joint
    d_hplus_dq{joint}(i,1)=0.0;
    d_hminus_dq{joint}(i,1)=0.0;
    end
    
    
    hplus(i)=0.0;    
    hminus(i)=0.0;
    
    h_plus_save(i)=0.0;
    h_minus_save(i)=0.0;
    
    for j=1:(n_joints-(m-1))

        vk=JE(:,Nnot(i,j));
       
        ntvk=vk'*n(i,:)';
        
        for joint=active_joint
        d_vk_dq{joint}=HE{joint}(1:3,Nnot(i,j));
        d_ntvk_dq{joint}=(d_n_dq{joint}(i,:)*vk) +n(i,:)*d_vk_dq{joint};
        d_hplus_dq{joint}(i)=  d_hplus_dq{joint}(i)+ (sigmoidGradient(ntvk,sigmoid_slope) *d_ntvk_dq{joint}*deltaq(Nnot(i,j))*ntvk) + sigmoid(ntvk,sigmoid_slope) *deltaq(Nnot(i,j))*d_ntvk_dq{joint};
        %d_hminus_dq{joint}(i)= d_hminus_dq{joint}(i)+(sigmoidGradient(ntvk,-sigmoid_slope)*d_ntvk_dq{joint}*deltaq(Nnot(i,j))*ntvk) + sigmoid(ntvk,-sigmoid_slope)*deltaq(Nnot(i,j))*d_ntvk_dq{joint};        
        d_hminus_dq{joint}(i)= d_hminus_dq{joint}(i)+(sigmoidGradient(ntvk,-sigmoid_slope)*d_ntvk_dq{joint}*deltaq(Nnot(i,j))*ntvk) + sigmoid(ntvk,-sigmoid_slope)*deltaq(Nnot(i,j))*d_ntvk_dq{joint};
        end
        
        hplus(i) =hplus(i)  +sigmoid(ntvk*sigmoid_slope)*  deltaq(Nnot(i,j))*ntvk;
        hminus(i)=hminus(i) +sigmoid(ntvk*-sigmoid_slope)* deltaq(Nnot(i,j))*ntvk;
        
        
%         % original calculation
%         for alpha= 0:1
%              h_tahir(alpha+1,:)=alpha* deltaq(Nnot(i,j))*ntvk;%%% $$$ check it again
%         end
%         h_plus_t=max(max(h_tahir));
%         h_minus_t=min(min(h_tahir));
%         
%         h_plus_save(i) = h_plus_save(i)+h_plus_t;
%         h_minus_save(i) = h_minus_save(i)+h_minus_t;
        
        
        
    end
    
    
    
end
% h_plus_save
% hplus
% h_minus_save
% hminus
% pause()

end

