%% Caculating the inertia matrix of the flexible platform
%  I need to write the expression which I have
%  developed (M.1) and also those that have been
%  developed by ADAMS (M.2) and ensure they are ok!!
%
% However we must remember that the calculations
% and formulations I have used and contiuous
% whereas ADAMs uses a discrete method
% 
%
% I also should take care since adams respresentation, of orientation is different it is in Euler angles ZXZ 
%
% Therefore everything must be converted in angualar velocity
%
%
%
% M.1
%InertiaMatrix=\left[
%   \begin{array}{ccc}
%                m\textbf{1}_{3}               & 	\textbf{M}\hat{\textbf{S}}^{T}_{p} & \textbf{M}\textbf{S}_{de} \\
%           	\textbf{M}\hat{\textbf{S}}_{p} &    \textbf{I}_{0p}                    & 	\textbf{M}\textbf{S}_{re} \\
%               \textbf{M}\textbf{S}_{de}^{T}  & 	\textbf{M}\textbf{S}_{re}^{T}      & \textbf{m}_{ee} \\
%    \end{array}\right] 
%
% M.2 
%Adams Inertia Matrix
%   \begin{array}{ccc}
%                M_{tt}               & 	M_{tr}     & M_{tm}  \\
%                M_{tr}^{T}           & 	M_{rr}     & M_{tm}  \\
%                M_{tm}^{T}           & 	M_{rm}^{T} & M_{mm}  \\
%    \end{array}\right] 
%
%

%

% As globals we have the Inertia Invariants
global Mp ModesNo Irr 
global I_1 I_2 I_3 I_4 I_5 I_6  I_8 I_9


% Our Objective is to compute the inertia matrix for a current configuration
%       
%       B.1 B.2 B.3
%       B.4 B.5 B.6
%       B.7 B.8 B.9
%
%% Inputs
q_e=rand(3,1); % Current deformation variables
Psi=0;theta=0;Phi=0;
R0B=ZXZ_to_Rot([Psi,theta,Phi]);

B=[0 cos(Psi) sin(Psi)*sin(theta)
   0 sin(Psi) -cos(Psi)*sin(theta)
   1 0         cos(theta)];


% B=inv([-sin(Psi)*cot(theta) ,  cos(Psi)*cot(theta) ,     1
%             cos(Psi)             ,  sin(Psi), 0
%             sin(Psi)/sin(theta) ,  -cos(Psi)/sin(theta)  ,    0]);


%% I need to find the matrices A and B 
%  Matrix A is used to convert orientation to world frame
%  gAj=gAb*bAp*pAj --Adams notation
%  R0j=R0b*Rbp*Rpj --My notation
%
% gAj/R0j is the orientation of marker j with respect to the ground
% R0B is the rotation matrix from the local body frame B (int_node_1 i think) and the ground
% 
% RBP is the rotation matrix due to the deformation
% RPJ is the constant transformation matrix that was defined by user

% Calculate the A matrix that is R0B
% RbP defined by deformation
% RPJ is identifyty
%
% Matrix B is used to convert the derivative of the angular angles to an angular velocity 
% 
theta_p=Phi_P(4:6,:)*q_e;
R0P=R0B*(eye(3)+skew(theta_p));


%% Block 1 linear Force linear acceleration
M=I_1; % Adams & Boyer 

%% Block 2 Linear Force Angualar acceleration 

% Adams 
sumI_3=zeros(3);
for i=1:ModesNo
    sumI_3=sumI_3+skew(I_3(:,i))*q_e(i);
end
M_tr=-(skew(I_2)+ sumI_3);


M_tr=-R0P*(M_tr)*B; % Convert into base frame and into angular velocity

% MS^T Boyer
MShat=skew([0;0;0]);
for i=1:ModesNo
    MShat=MShat+(skew(b_k{i}).*q_e(i));
end
MShat=MShat+skew(MS_r);
MShat=transpose(MShat)
%% Block 3. % Linear Force generalized acceleration coorindates

% Adams M_tm
M_tm=R0P*I_3;

% Boyer MS_de
MS_de=[b_k{1} b_k{2} b_k{3}];


%% Block 4. Torque, Linear acceleration

% Adams M_tr^T
M_tr_T=transpose(M_tr)

% Boyer MS_de_T=
MS_hat_T=transpose(MShat);

%% Block 5 Torque Angualr acceleration

% Adams and Boyer here are exactly the same format
I_7=Irr;
sumI_8=zeros(3);
sumI_9=zeros(3);
for i=1:ModesNo
   sumI_8= sumI_8+(I_8{i}+transpose(I_8{i}))*q_e(i)
   for j=1:ModesNo
       sumI_9=sumI_9+I_9{i,j}*q_e(i)*q_e(j);
   end
end
Mrr=I_7-(sumI_8)-sumI_9
Mrr=transpose(B)*Mrr*B


%% Block 6 Torque and acceleration of generalized Coordinates

% Again they correspond but be careful of transpose
% Adams  M_rm
sumI_5=zeros(3);
for i=1:ModesNo
   sumI_5=sumI_5+(I_5{i}* q_e(i));
end
M_rm=I_4+sumI_5;
M_rm=transpose(B)*M_rm;
% Boyer MS_RE
MS_RE=zeros(3,3)
for i=1:ModesNo
    sumlambda_ik=zeros(3,1);
    for j=1:ModesNo
        sumlambda_ik=sumlambda_ik+lambda_ik{j}*q_e(j);
    end
   MS_RE(:,i)=(Beta_k{i}+sumlambda_ik);
end

MS_RE

%% Block 7, Generalized Forces nad acceleration of linear

% Again there is a difference here due to the B matrix  

% Adams M_tm_transpose
M_tm_T=transpose(M_tm)

% Boyer Tranpose M_de
M_de_T=transpose(MS_de) 

%% Block 8 Generalized Forces nad acceleration of angualar

%M_rm_T
M_rm_T=transpose(M_rm)

% Boyer 
MS_RE_T=transpose(MS_RE)

%% Block 9 Generalized Forces nad acceleration of generalized coordinaes

% Adams_mm
M_mm=I_6

% Boyer is the same





