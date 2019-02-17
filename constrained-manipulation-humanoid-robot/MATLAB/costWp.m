function [ cost ] = costWp(x,r2,x0,qnames,RobotChain,object_constants,gains)
%costWp humanoid costs plus manipulability


q3=x(4:end); % exclude floating base decision variables
lf_T_opt=constructOpFrame(x(1:3));
objects=getObjectsLeftFootFrame(lf_T_opt,object_constants);
mindist_robot_obj=100;

for obj = 1:length(objects)
    try
        dist_leftarm_obj=100;
        dist_lhp_obj=getMinObjectDistance(r2,q3,'leftHipPitchLink',objects{obj});
        dist_rkp_obj=getMinObjectDistance(r2,q3,'rightKneePitchLink',objects{obj});
        for l=1:length(RobotChain.LEFT_ARM_LINKS) % All arm links on manipulator
            d3=getMinObjectDistance(r2,q3,RobotChain.LEFT_ARM_LINKS{l},objects{obj});
            dist_leftarm_obj=min(d3,dist_leftarm_obj);
        end
        mindist_robot_obj=min([mindist_robot_obj,dist_lhp_obj,dist_rkp_obj,dist_leftarm_obj]);
    catch
        disp 'error'
        mindist_robot_obj=0.0;
    end
end
obj_cost=gains.a1*exp ( gains.b1*-mindist_robot_obj);
% ZMP cost
lf_rightSupport= RobotChain.lf_leftSupport;
lfTrf=getTransform(r2,q3,'rightFoot');
for i=1:length(RobotChain.lf_leftSupport)
    temp=lfTrf*[RobotChain.lf_leftSupport(i,:),1]';
    lf_rightSupport(i,:)=temp(1:3);
end
% Total support polygon formed by two feet
supportPolygon=Polyhedron( [RobotChain.lf_leftSupport(:,1:2);      lf_rightSupport(:,1:2)]);
com=r2.centerOfMass(q3);
c=capacityMargin(supportPolygon.A', supportPolygon.b',com(1:2));
zmp_cost=exp ( gains.b1*-10*c);
wp=getManipulability(r2,q3,qnames,RobotChain); % Normal Manipulability Cost
manip_cost=gains.a1*exp ( gains.b1*-wp/100);


cost=norm(x0-q3)+zmp_cost+0.2*obj_cost +manip_cost;

end
