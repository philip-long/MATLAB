function [ s ] = expandJointPolytope(used_joints,qupper,qlower)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    Q=[]
    for i=1:length(used_joints)
    Q=expandPolytope(Q,qlower(used_joints(i)),qupper(used_joints(i)));
    end
 
    field1='used_joints';
    field2='vertex';
    s=struct(field1,used_joints,field2,Q);
end

