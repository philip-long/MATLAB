function [Ga Gac Gp Gpc Gc]=ExtractGaGpGc(u)
% This function gets the matrces that relate the passive joint velocities
% to the active joint velocities for the Nao robot, for the globally
% defined actuation scheme
%
%
global Qactuated Qpassive Qcut

% Get the generic matrix of qa=[1,2,3,4]; qp=[5;6;7;8;9]
J=VelocityConstraints(u);

Qactuated;
Qpassive;
Qcut;


Qa=sort(Qactuated);
Qp=sort(Qpassive);
Qc=sort(Qcut);

%Ga
Ga=zeros(5,length(Qa));
Gac=zeros(1,length(Qa));
Gp=zeros(5,length(Qp));
Gpc=zeros(1,length(Qp));
Gc=J(6,Qc);
for i=1:length(Qa)
    Ga(:,i)=J(1:5,Qa(i));
    Gac(:,i)=J(6,Qa(i));
end

for i=1:length(Qp)
    Gp(:,i)=J(1:5,Qp(i));
    Gpc(:,i)=J(6,Qp(i));
end