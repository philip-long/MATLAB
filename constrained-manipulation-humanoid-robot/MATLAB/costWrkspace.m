function [ cost ] = costWrkspace(x,r2,~,lf_Pd,RobotChain)
%wrkspaceCost Cost excluding all manipulability stuff.



q3=x;
% Collision cost leftHipPitch rightKneePitch
lf_T_e=getTransform(r2,q3,'leftPalm')*RobotChain.Tne;
error=norm(lf_T_e(1:3,4)-[lf_Pd(1);lf_Pd(2);lf_Pd(3)]);

if(error<0.001)
    cost=0.0;
else
cost=10*error;%;
end


end

