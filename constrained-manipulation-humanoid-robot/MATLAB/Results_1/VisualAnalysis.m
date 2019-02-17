%% Visual Analysis
clc,close all,clear all
ResultFiles=dir('*.mat');
SEPARATE=false;
SQP=true; % ANALYSE SQP RESULTS TRUE==SQP FALSE=INTERIOR_POINT
for o=1:2
for j=1:20
    wp_result_valid=false;
    wpstar_result_valid=false;
    for i=1:length(ResultFiles)
        if(contains(ResultFiles(i).name,'Fail'))
            continue; % If either fail we skip
        elseif(SEPARATE && SQP~=contains(ResultFiles(i).name,'sqp')) %     
            continue;
        elseif(contains(ResultFiles(i).name,strcat('wp_t',int2str(j),'obj_',int2str(o))))
            WP_RESULTS=load(ResultFiles(i).name);    
            wp_result_valid=true;
        elseif(contains(ResultFiles(i).name,strcat('wpstar_t',int2str(j),'obj_',int2str(o))))
            WPSTAR_RESULTS=load(ResultFiles(i).name);            
            wpstar_result_valid=true;
        end        
    end
    if(wp_result_valid && wpstar_result_valid)
        
        % Initial Object Location
        lf_T_op=constructOpFrame(WP_RESULTS.tinit);
        lf_Pd=lf_T_op*[WP_RESULTS.op_P_d;1]; % get pose in left foot frame
        lf_objects=getObjectsLeftFootFrame(lf_T_op,WP_RESULTS.Table,WP_RESULTS.Book1,WP_RESULTS.Book2,WP_RESULTS.Book3);
        showObjects(lf_objects);
        disp 'here'
%         publishControlPoint(lf_Pd,1,'leftFoot');        
%         desired_mesh=Polyhedron(transformMesh(lf_T_op,WP_RESULTS.Desired_Vertices.V));
%         showMesh(getMeshRviz(desired_mesh,'leftFoot', [0.5,1.0,1.0,0.2]));
%         
%         
%         % Final Location WP        
%         lf_T_op=constructOpFrame(WP_RESULTS.x_lf);
%         lf_Pd=lf_T_op*[WP_RESULTS.op_P_d;1]; % get pose in left foot frame
%         lf_objects=getObjectsLeftFootFrame(lf_T_op,WP_RESULTS.Table,WP_RESULTS.Book1,WP_RESULTS.Book2,WP_RESULTS.Book3);
%         showObjects(lf_objects);
%         publishControlPoint(lf_Pd,1,'leftFoot');
%         showConfigRviz(WP_RESULTS.x_joints,WP_RESULTS.qnames);
%         desired_mesh=Polyhedron(transformMesh(lf_T_op,WP_RESULTS.Desired_Vertices.V));
%         showMesh(getMeshRviz(desired_mesh,'leftFoot', [0.5,1.0,1.0,0.2]));
%         
%         % Final Location WPSTAR
%         lf_T_op=constructOpFrame(WPSTAR_RESULTS.x_lf);
%         lf_Pd=lf_T_op*[WPSTAR_RESULTS.op_P_d;1]; % get pose in left foot frame
%         lf_objects=getObjectsLeftFootFrame(lf_T_op,WPSTAR_RESULTS.Table,WPSTAR_RESULTS.Book1,WPSTAR_RESULTS.Book2,WPSTAR_RESULTS.Book3);
%         showObjects(lf_objects);
%         
%         publishControlPoint(lf_Pd,1,'leftFoot');
%         showConfigRviz(WPSTAR_RESULTS.x_joints,WPSTAR_RESULTS.qnames);
%         
        
    end    
end
end
