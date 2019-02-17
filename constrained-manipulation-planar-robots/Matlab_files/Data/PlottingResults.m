
clear all,clc,close all 
DirList = dir(fullfile('./', '*OBJ.mat'));
for i=1:length(DirList)
        load(fullfile('./', DirList(i).name));

% Getting Volume Informaion
V=[1 0 0
   0 1 0];
% Volume Information
if( isempty(polytope_information_deformed{end}))
    
    [k q_vol]=convhull(polytope_information_original{end-1}.jointVertex);  
    [kd q_vold]=convhull(polytope_information_deformed{end-1}.deformedJointVertex);      
    assert(all(k==kd))    
    [vc,volC]=getPolytopeInfo(polytope_information_original{end-1}.cartesianVertex,V);
    [vc,volCd]=getPolytopeInfo(polytope_information_deformed{end-1}.deformedCartesianVertex,V);
    
else  
    
    [k q_vol]=convhull(polytope_information_original{end}.jointVertex);  
    [kd q_vold]=convhull(polytope_information_deformed{end}.deformedJointVertex);      
    assert(all(k==kd))    
    [vc,volC]=getPolytopeInfo(polytope_information_original{end}.cartesianVertex,V);
    [vc,volCd]=getPolytopeInfo(polytope_information_deformed{end}.deformedCartesianVertex,V);
end
pp=strcat(DirList(i).name(1:end-4),'_vol_info_pp');
save(pp,'q_vol','q_vold','volC','volCd')
end

%% Plotting Cartesian polytopes
% First step
clear all,clc,close all
load('C1B_1OBJ.mat')
fig=figure(10);
if( isempty(polytope_information_deformed{end}))
    [h,vol_el] = ellipsoidCreate(polytope_information_original{end-1}.jacobian);
    plotPolytopeLin(polytope_information_original{end-1}.cartesianVertex,'k-',0.1,fig);
    %plotPolytopeLin(polytope_information{end-1}.deformedCartesianVertex,'k',0.1,fig,true);
else
    plotPolytopeLin(polytope_information_original{end}.cartesianVertex,'k-',0.1,fig);
    plotPolytopeLin(polytope_information_deformed{end}.deformedCartesianVertex,'k',0.1,fig,true);
end
axis([-0.6 0.6 -1.8 1.8])
grid on


%% PLotting Joint polytopes
clear all,clc,close all
load('C1B_1OBJ.mat')
fig=figure(3);
if( isempty(polytope_information_deformed{end}))
    plotPolytopeLin(polytope_information_original{end-1}.cartesianVertex,'k-',0.1,fig);
    %plotPolytopeLin(polytope_information{end-1}.deformedCartesianVertex,'k',0.1,fig,true);
else
    plotPolytopeJoint(polytope_information_original{end}.deformedJointVertex ,'k-',0.1,fig);
    [k vol_org]=convhull(polytope_information_original{end}.deformedJointVertex)
    plotPolytopeJoint(polytope_information_deformed{end}.deformedJointVertex,'k',0.1,fig,true);
    [k vol_def]=convhull(polytope_information_deformed{end}.deformedJointVertex)
    
end

eta=vol_def/vol_org;
qu=zeros(2,1);
ql=zeros(2,1);
e1=0.0;
for i=1:2
    QQ=polytope_information_deformed{end}.deformedJointVertex(:,i)>0;
    qu(i)=min(polytope_information_deformed{end}.deformedJointVertex(QQ,i));
    QQ=polytope_information_deformed{end}.deformedJointVertex(:,i)<0;
    ql(i)=max(polytope_information_deformed{end}.deformedJointVertex(QQ,i));
    e1=e1+(qu(i)-ql(i))/2;
end

polyQ=expandJointPolytope([1 2],qu,ql);

plotPolytopeJoint(polyQ.vertex,'k',0.05,fig,true)
 [k vol_def2]=convhull(polyQ.vertex)

 
% Calculate the etas
eta
eta2=vol_def2/vol_org;
mean_reduction=e1/2




vertices=getCartesianVertex(polytope_information_deformed{end}.jacobian,ql,qu, [0;0])

plotPolytopeLin(vertices.cartesianVertex,'k',0.05,fig,true);

%% Get POlytope information Properly and cleanly this time
clear all,clc,close all
load('C2B_2OBJ.mat')
% V=[1 0 0
%    0 1 0];
% [vc,volC]=getPolytopeInfo(polytope_information_original{end}.cartesianVertex,V);
% [vc,volCd]=getPolytopeInfo(polytope_information_deformed{end}.deformedCartesianVertex,V);

[k q_vol]=convhull(polytope_information_original{end}.jointVertex);  
[kd q_vold]=convhull(polytope_information_deformed{end}.deformedJointVertex);  

eta=q_vold/q_vol;

qu=zeros(2,1);
ql=zeros(2,1);
e1=0.0;
for i=1:2
    QQ=polytope_information_deformed{end}.deformedJointVertex(:,i)>0;
    qu(i)=min(polytope_information_deformed{end}.deformedJointVertex(QQ,i));
    QQ=polytope_information_deformed{end}.deformedJointVertex(:,i)<0;
    ql(i)=max(polytope_information_deformed{end}.deformedJointVertex(QQ,i));
    e1=e1+(qu(i)-ql(i))/2;
end
eta_r=e1/2
polyQ=expandJointPolytope([1 2],qu,ql);
[k vol_def2]=convhull(polyQ.vertex)
eta_small=vol_def2/q_vol;

reduced_poly_quql=getCartesianVertex(polytope_information_deformed{end}.jacobian,ql,qu, [0;0])
%[vc,volC]=getPolytopeInfo(reduced_poly_quql.cartesianVertex,V);


save('C22_bar_info','eta_r','eta','eta_small');