function [c,ceq]=poseConstraint(q,xd,yd)
% poseConstraint pose constraint for simple planar robot
Xd=[0.5,0.5];
q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
px=q1 + (11*cos(q2 + q3))/20 + (4*cos(q2))/5;
py=q0 + (11*sin(q2 + q3))/20 + (4*sin(q2))/5;
c=(xd-px)^2 +(yd-py)^2;% + 0.0001;
ceq=[];
end

