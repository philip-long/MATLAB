%% main_ext_manip
%
% roslaunch constrained_manipulation planar_humanoids_2018.launch
%
%  In this case we try to formulate a fair comparison between N. Vahremps
% manipulability analysis work and my polytope work with a toy planar
% example.

close all,clear all,
clc
addpath(genpath('~/tbxmanager'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
% test configurations q1=[0.8,-0.9], q2=[0.8,-1.53]
try
    joint= rossubscriber('/joint_states');
catch
    rosinit
    joint= rossubscriber('/joint_states');
end

scandata = receive(joint,0.5);
joint_names={'q2', 'q3'};
total_frames=length(joint_names);
q_pos=scandata.Position(contains(scandata.Name,joint_names));

%
% N.V's work: They apply penalizations on each term of the Jacobian matrix.
% These penalization change depending on the instigated motion hence the
% space of potential movements must be partitioned for each motion.  The
% space is defined as 2^6 hyperoctants or for planar case 2^3 hyperoctants
% i.e. 8
Hyperoctant=generateVertexSet([1 1],[-1 -1]);
% For each permutation in GAMMA we generate a jacobian matrix that
% describes the maneuvrability in the corresponding hyperoctant of the
% space of potential movements.

%% Joint limits
% Firsly we need to generate Lij which is the penalization due to joint
% limits
T=getTransforms(q_pos);
J=getJacobians(q_pos);  % Get all Jacobians
q_upper=[pi/2;pi/2];
q_lower=[-pi/2;-pi/2];

TnE=eye(4); TnE(1,4)=0.55;   % terminal point
% Define Control point and publish frame
% This is necessary  since my chain ends with the last joint
[T0E,J0E]=getEEInfo(T{total_frames},J{total_frames},TnE,'world');
T{end+1}=T0E;
J{end+1}=J0E;

%%  Object initialization
ObjectPositions=[];
ObjectPositions(1,:)=[0.1;0.28;0.0]; % in world frame
ObjectPositions(2,:)=[0.8;0.45;0.0]; % in world frame

Obj_info={}; % is a cell (number of objects) of cells (number of links),

for i=1:size(ObjectPositions,1)
    LinkOrien=rot2Quat_xyzw(T{i}(1:3,1:3)); % used just to align object and link    
    pub = rospublisher('/visualization_marker','visualization_msgs/Marker');
    marker=publishObjectInfoRviz(ObjectPositions(i,:),'world',i,LinkOrien);
    send(pub,marker) % oublish the marker
end



%%
h=zeros(2,1);
pjplus=zeros(2,1);
pjminus=zeros(2,1);
cart_dirs=2;
% examples
% Space is Partitioned into Hyperoctants each one called GAMMA
% There are 2^cart_dirs Hyperoctants corresponding to possible motions
% in Cartesian space
% for each motion we compute a penaly base on requested motion
% and its effect of bring q closest or further away from joint limit.

for k=1:length(Hyperoctant)
    L{k}=zeros(cart_dirs,2);
    for i=1:cart_dirs
        for j=1:2
            % There is an error in paper, this is correct gradient
            numerator=( q_upper(j)-q_lower(j))*(q_upper(j)-q_lower(j)) * (2*q_pos(j) - q_upper(j) - q_lower(j));
            denominator=4*(q_upper(j)-q_pos(j))*(q_upper(j)-q_pos(j))*(q_pos(j)-q_lower(j) )*(q_pos(j)-q_lower(j) );
            h(j)=numerator/denominator;
           
            
            if(  (abs(  q_pos(j)-q_lower(j) )) >(abs(  q_upper(j)-q_pos(j)) )  )
                pjminus(j)=1;
                pjplus(j)=1/sqrt(1+abs(h(j)));
            else
                pjminus(j)=1/sqrt(1+abs(h(j)));
                pjplus(j)=1;
            end
            
            if(  (  sign(J{end}(i,j)) * sign(Hyperoctant(k,i)) )<0 )
                L{k}(i,j)    = pjminus(j);
            else
                L{k}(i,j)    = pjplus(j);
                
            end
        end
    end
end



%% Obstacles
%plotObjects(ObjectPositions,2);
min_distance_=100;
%  In Vahremkamp et al. for each object we find the closest
% point on the manipulator
for o=1: size(ObjectPositions,1)
    pt=T{end}(1:3,4);
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
dld_dlq=(1/d)*(transpose(J{end})*candidate_v)';
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




%% Plotting The resultsing ellipse
% 
% sing_values=[];
% 
% plot_order=[3,4,1,2];
% figure()
% for k=1:length(Hyperoctant)
%     
%     K=Hyperoctant(k,:);
%     J_ext{k}=zeros(cart_dirs,2);
%     for i=1:cart_dirs
%         for j=1:2
%              J_ext{k}(i,j)=L{k}(i,j)*O{k}(i,j)*J{end}(i,j);
%         end   
%     end
%     sing_values=[sing_values;svd(J_ext{k}(1:2,:))];
%     
%     %ellipsoidCreate(J{3}(1:2,:))
%     %
%     subplot(2,2,plot_order(k))
%     K
%     hold on
%     [s,v]=eig(J_ext{k}(1:2,:))
%     ellipsoidCreate(J_ext{k}(1:2,:),0.4)
%     ellipsoidCreate(J{end}(1:2,:),0.1)
%     xlim([-0.6316, 0.6316]);
%     ylim([ -1.8169, 1.8169]);
%     axis square
%     %ellipsoidQuadrantCreate(J_ext{k}(1:2,:),K)
%     title(strcat('\Gamma=[',num2str(K(1)),', ',num2str(K(2)),']'))
%     pause()
% end
% title('Extended Manipulability Ellipsoid')
% C_ext=min(sing_values)/max(sing_values)

%%

sing_values=[];

plot_order=[3,4,1,2];
figure()
for k=1:length(Hyperoctant)
    
    K=Hyperoctant(k,:);
    J_ext_oj{k}=zeros(cart_dirs,2);
    J_ext_o{k}=zeros(cart_dirs,2);
    for i=1:cart_dirs
        for j=1:2
             J_ext_oj{k}(i,j)=L{k}(i,j)*O{k}(i,j)*J{end}(i,j);
             J_ext_o{k}(i,j)=O{k}(i,j)*J{end}(i,j);
        end   
    end
    sing_values=[sing_values;svd(J_ext_oj{k}(1:2,:))];
    
    %ellipsoidCreate(J{3}(1:2,:))
    %
    subplot(2,2,plot_order(k))
   % K
    hold on
   %[s,v]=eig(J_ext_oj{k}(1:2,:))

    ellipsoidCreate(J{end}(1:2,:),0.2,[1.0,0.0,0.0])
    ellipsoidCreate(J_ext_o{k}(1:2,:),0.4,[0.0,1.0,0.0])
    ellipsoidCreate(J_ext_oj{k}(1:2,:),0.6,[0.0,0.0,1.0])    
    xlim([-0.6316, 0.6316]);
    ylim([ -1.8169, 1.8169]);
    xticks([-0.6 -0.4 -0.2 0 0.2 0.4 0.6])
    yticks([-1.5 -1.0 -0.5 0 0.5 1.0 1.5])
    axis square
    title(strcat('\Gamma=[',num2str(K(1)),', ',num2str(K(2)),']'))
    grid on
    pause()
end
svd_orig=svd(J{end}(1:2,:));
C_org=min(svd_orig)/max(svd_orig)
C_ext=min(sing_values)/max(sing_values)
C_ext/C_org



%% 
% The following comment code finds the closest point on the manipulator
% However, it doesn't make sense to do this with the extended
% manipulability measure as the penalization is done in Cartesian space
% at the end effector not along the body
% for o=1: size(ObjectPositions,1)
%     for i=1:(size(wTr,2)-1)
%         iTip1=inv(wTr{i})*wTr{i+1};
%         iPip1=iTip1(1:3,4);
%         if(norm(iPip1)<0.001)% Links are coicident
%             disp ' Links coincident'
%         else  % Links are not
%             segment=wTr{i+1}(1:3,4)-wTr{i}(1:3,4)
%             obj_point=ObjectPositions(o,:)';
%             [pt,d]=getMinDistPtSeg(wTr{i+1}(1:3,4),wTr{i}(1:3,4),obj_point);
%             distance_along_link=pt-wTr{i}(1:3,4);
%             Lhat=skew(distance_along_link);
%             screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
%             JJ=screwTransform*wJr{i};
%             vec=[obj_point-pt;0;0;0];
%             if(d<min_distance_)
%                 jac=JJ;
%                 min_distance_=d;
%                 v=vec;
%                 o_candidate=o;
%                 l_candidate=i;
%             end
%         end
%     end
% end