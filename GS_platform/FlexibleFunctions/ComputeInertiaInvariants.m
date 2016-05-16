%% Check if I can dervie the rigid moment of inertia fromthe nodes
%  I have two concerns with these calculations:
%       1. c.o.g is not exactly aligned with node1261
%       2. r{n} may be badly defined at the moment its defined as vector from c.o.g to point p
%    Both these issues can be easily changed, in Rigid Parameters
%
%
%
clear all
load 'NodePositions'
load 'ModalFunctions'
Mp= 18.1860007386 ;
MSP=[0 ;0 ;0];
G1M=[0.00, 0.000, 0.0]; % With respect to local origin (Attachment pt. 1)
 
Phi_P=PhiM{7}

% 
% Phi_P=eye(46);
% Inertia Tensor  : (relative to the Local Body Reference Frame) 
IXX=0.9485526709;% kg-meter**2
IYY=0.948608303;% kg-meter**2
IZZ=1.8971609739;% kg-meter**2
IXY=0.0;% kg-meter**2
IZX=0.0;% kg-meter**2
IYZ=0.0;% kg-meter**2


IP=[IXX IXY IZX
    IXY IYY IYZ
    IZX IYZ IZZ];

% Must Transform Inertia Tensor to centre of mass
% ie my origin in fact I am trasnforming back to
% the centre of gravity!!
Sj=[0,0,0]-G1M;
I_rr=IP+Mp*(skew(Sj)*skew(Sj));
Sj=N(1261,2:end)-G1M;
IG_node1261=I_rr-Mp*(skew(Sj)*skew(Sj));



ModesNo=46; 
%global I_1 I_2 I_3 I_4 I_5 I_6  I_8 I_9
cg=[0.0, 0.0, 0.0];
for n=1:length(N)
    plot(N(n,2),N(n,3),'r*')
    hold on
    r{n}=-cg+N(n,2:4);
    
end
plot(cg(1),cg(2),'bo')
plot(0,0,'go')
% Nodes 3 appraoxiamtes the continuous behavior 

dm=Mp/length(N);

%% Calculate the inertia invariants
%
%  As the name suggests these Matrices do not change with the robot configuation
%
%
%  Notes in general there is very good corespondence between the two works however ADAMS switches between
% Matrical PHI and Phi_j whereas boyer keeps Phi_j
% Example  I^{3}  and b_{k}
%   
%  Boyer computes three vectors each of 3 \times M and calls them b_{k} k for each mode shape
%  Then to compute for exampe MS_{de} (otherwise known as M_{tm} in adams) Boyer concancetes all b_{k}s 
%  whereas Adams it is already done due to matrix notation 
%  therefore be careful when this matricial notation is introduced I_{4} I_{5} I_{4} I_{6}
%
%
%%   Mass Matrix    Boyer M                        adams:I^{1} 
M=eye(3).*Mp;
I_1=M;

%% Inertia Matrix I_rr                              adams: I^{7}
I_rr=0;
for n=1:length(N)
    I_rr=I_rr+(transpose(skew(r{n}))*skew(r{n}))*dm;
end
I_rr;

% Al

%% Vector of rigid first moments of inertia adams: I^{2} Boyer MS_{r} it should be zero I think
MS_r=[0 0 0];
for n=1:length(N)
    MS_r=MS_r+(r{n})*dm;
end
MS_r=[0 0 0]';
I_2=[0 0 0]';
%% Vector of elastic first moments of inertia adams: I^{3} Boyer b_k or a__{alpha}

clear('b_k')
for i=1:length(N)
    for j=1:ModesNo
        b_k{ModesNo}=Phi_P(1:3,i)*dm;
    end
end
celldisp(b_k);

% Using adam matrical notation Phi_P(1:3,:)*dm % gives same result
clear('I_3')

for i=1:length(N)
    I_3=I_3+(dm*PhiM
I_3=Phi_P(1:3,:)*dm;
I_3=I_3(:,7:end)

%%  adams: I^{4} Boyer Beta or \alpha_{beta}
clear('Beta_k')
%Beta_k={};
for j=1:ModesNo
    Beta_k{j}=zeros(3,1);
    for i=1:length(N)
       Beta_k{j}=Beta_k{j}+skew(-cg+N(i,2:4))*Phi_P(1:3,j)*dm;
    end
end
celldisp(Beta_k);

% Using adam matrical notation Phi_P(1:3,:)*dm % gives same result
I_4=0;
for n=1:length(N)
       I_4=I_4+skew(r{n})*dm;
end
I_4=I_4*Phi_P(1:3,:);

%% I^{5} Boyer equivalent \lambda_{ik}
clear('lambda_ik')
for i=1:ModesNo
    for k=1:ModesNo
        lambda_ik{i,k}=zeros(3,1);
        for n=1:length(N)
        lambda_ik{i,k}=lambda_ik{i,k}+skew(Phi_P(1:3,i))*Phi_P(1:3,k)*dm;
        end
    end
end
celldisp(lambda_ik);

% Adams version to check

for j=1:ModesNo
    I_5{j}=zeros(3);
    for n=1:length(N)
        I_5{j}=I_5{j}+skew(Phi_P(1:3,j))*dm;
    end
    I_5{j}=I_5{j}*Phi_P(1:3,:);
end
celldisp(I_5);

%% adams: I^{6} Boyer m_{ee} or J_{ee} Rayleigh-Ritz masses Nodal Inertias are zeros

diagI=eye(ModesNo);
for j=1:ModesNo
    diagI(j,j)=dm*transpose(Phi_P(1:3,j))*Phi_P(1:3,j);
end
I6=zeros(ModesNo,ModesNo);
for P=1:1261
    I6=I6+(dm*transpose(Phi_P(:,j))*Phi_P(:,j));
end

m_ee=diagI;
I_6=diagI;

%% adams I^{7} Boyer equivalent




%% I_re_j or adams:I^{8} one for each mode shape
clear('I_re')
for j=1:ModesNo
	I_re{j}=0;
	for n=1:length(N)
		I_re{j}=I_re{j}+(skew(N(n,2:4)-cg)*skew(Phi_P(1:3,j)))*dm;
	end
end
celldisp(I_re);
I_8=I_re;

%% I_er_j or I_re_j^{T} one for each mode shape adams I^{8}^{T}
clear('I_er')
for j=1:ModesNo
    I_er{j}=transpose(I_re{j});
end
celldisp(I_er) ;


%% I_ee_jk a.k.a I^{9}
clear('I_ee')
for j=1:ModesNo
        for k=1:ModesNo
        I_ee{j,k}=skew(Phi_P(1:3,j))*skew(Phi_P(1:3,k))*Mp/length(N);
        end
end
celldisp(I_ee);
I_9=I_ee;
