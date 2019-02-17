function [ cost ] = costWpStar(x,r2,x0,qnames,RobotChain,object_constants,gains)
%costWp humanoid costs plus constrained manipulability

q3=x(4:end); % exclude floating base decision variables

% Collision cost leftHipPitch rightKneePitch
lf_T_opt=constructOpFrame(x(1:3));
objects=getObjectsLeftFootFrame(lf_T_opt,object_constants);

% arm collisions are implicity considered by constrained manipulability
% only take lower body collisions
% (These two links are typically enough, could add others but optimization
%  time increases a lot)
d1=getMinObjectDistance(r2,q3,'leftHipPitchLink',objects{1});
d2=getMinObjectDistance(r2,q3,'rightKneePitchLink',objects{1});
mindist_robot_obj=min([d1,d2]);

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

V=getConstrainedManipulability(r2,q3,qnames,objects,RobotChain,gains);
if(V.isEmptySet)
    vol=0.0;    
else
    vol=V.volume;
end
manip_cost=gains.a1*exp ( gains.b1*-vol/100); 


cost=norm(x0-q3)+zmp_cost+0.2*obj_cost +manip_cost;
end

