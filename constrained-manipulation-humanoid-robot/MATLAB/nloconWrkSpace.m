function [ c_inequality,c_equality ] = nloconWrkSpace(x,r2,lfTrfd,objects,RobotChain )
%nloconWrkSpace non-linear workspace constraints, ensuing the system
%remains in the feasible space represented by the IK solution

q3=x;
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


% ZMP cost
lf_rightSupport= RobotChain.lf_leftSupport;
lfTrf=getTransform(r2,q3,'rightFoot');
for i=1:length(RobotChain.lf_leftSupport)
    temp=lfTrf*[RobotChain.lf_leftSupport(i,:),1]';
    lf_rightSupport(i,:)=temp(1:3);
end
supportPolygon=Polyhedron( [RobotChain.lf_leftSupport(:,1:2);      lf_rightSupport(:,1:2)]);
com=r2.centerOfMass(q3);
% Ok when positire
c_zmp=capacityMargin(supportPolygon.A', supportPolygon.b',com(1:2));


q=x;
% Posture is frozen now
lfTrf=getTransform(r2,q,'rightFoot');
foot_position=norm(lfTrfd(1:3,4)-lfTrf(1:3,4));
utheta=rot2AngleAxis(transpose(lfTrf(1:3,1:3))*lfTrfd(1:3,1:3));

d_min=0.01;

c_obj=d_min-mindist_robot_obj;
%error-0.001;
c_inequality=[foot_position-0.001;norm(utheta(1))-0.01;-c_zmp;c_obj];
%c_inequality=[];
c_equality=[];

end

