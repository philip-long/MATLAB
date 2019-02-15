function [ s ] = expandJointPolytope(used_joints,qupper,qlower)
%expandJointPolytope creates vertices of joint velocity polytope
%  by expansion
    Q=[];
    for i=1:length(used_joints)
    Q=expandPolytope(Q,qlower(used_joints(i)),qupper(used_joints(i)));
    end
 
    field1='used_joints';
    field2='vertex';
    s=struct(field1,used_joints,field2,Q);
end

