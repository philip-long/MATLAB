function [q,qdot,qddot] = JointTrajectory(qinit,qfinal,t)


rt=(10*((t/(tfinal))^3))-(15*((t/(tfinal))^4))+(6*((t/(tfinal))^5));
rtdot =(30*t^2)/(tfinal)^3 - (60*t^3)/(tfinal)^4 + (30*t^4)/(tfinal)^5;
rtddot=((120*t^3)/(tfinal)^5 - (180*t^2)/(tfinal)^4 + (60*t)/(tfinal)^3);

 D=qfinal-qinit; % Takes the difference between the intial and final points of a section in position
q=qinit+rt*D;
qdot=rtdot*D;
qddot=rtddot*D;

end
