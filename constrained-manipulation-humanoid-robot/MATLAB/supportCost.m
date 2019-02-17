function [ cost ] = supportCost(q3,r2,x0,RobotChain)
%supportCost Cost associated with stability constraints
%   Detailed explanation goes here


lf_rightSupport= RobotChain.lf_leftSupport;     
lfTrf=getTransform(r2,q3,'rightFoot');
for i=1:length(RobotChain.lf_leftSupport)      
    temp=lfTrf*[RobotChain.lf_leftSupport(i,:),1]';
    lf_rightSupport(i,:)=temp(1:3);
end
supportPolygon=Polyhedron( [RobotChain.lf_leftSupport(:,1:2);      lf_rightSupport(:,1:2)]);
com=r2.centerOfMass(q3);
c=capacityMargin(supportPolygon.A', supportPolygon.b',com(1:2));

cost=norm(x0-q3)-(10*c);
end

