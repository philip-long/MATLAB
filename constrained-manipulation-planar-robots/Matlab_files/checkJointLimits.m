function [ configs] = checkJointLimits( Q_configs,qmax,qmin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% step one rearrange the joints such that elbow down configuration is always
% the first element
configs=[];


% Float comparison
for i=1:2
    if(all( Q_configs(i,:)< qmax+1e-10 ) && all(Q_configs(i,:)> qmin-1e-10))
    configs=[configs i];
    end
end
end


