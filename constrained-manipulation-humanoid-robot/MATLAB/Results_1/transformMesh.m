function [ lf_mesh_object ] = transformMesh( Lf_T_opt,mesh_object )
%getObjectsLeftFootFrame Transform the objects from object frame to left
%foot frame


lf_mesh_object=mesh_object; 

for i=1:length(mesh_object)
    temp=Lf_T_opt*[mesh_object(i,:),1]';
    lf_mesh_object(i,:)=temp(1:3);
end

end

