function [ c_inequality,c_equality ] = nlconPose(op_P_d,x,r2 )
%nlconPose non-linear pose constraints
%   Detailed explanation goes here
Tne=eye(4); Tne(2,4)=0.08;

q=x(4:end);
lf_T_op=constructOpFrame(x(1:3));
lf_Pd=lf_T_op*[op_P_d;1]; % get pose in left foot frame


% Pose constraints
lf_T_e=getTransform(r2,q,'leftPalm')*Tne;


% Right foot on ground constraints
lfTrf=getTransform(r2,q,'rightFoot');
rz=norm(lfTrf(3,4))^2;
ry=lfTrf(2,4)+0.18;
utheta=rot2AngleAxis(lfTrf(1:3,1:3));

% 

error=norm(lf_T_e(1:3,4)-lf_Pd(1:3));
c_inequality=[error-0.0001;ry;norm(utheta(1))-0.001];
c_equality=rz; % 


end

