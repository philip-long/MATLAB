function R=AngleAxis_to_Rot(ut)
% Converts from a 4 times 1 vector (theta u) to a rotation matrix

theta=ut(1);
u=ut(2:end);

if size(u,1)==1 
    u=u';
end

R=(u*transpose(u))  +    (cos(theta) *(eye(3) - (u*transpose(u)) )) + (sin(theta)*skew(u));

%R=(cos(theta)*eye(3)) + sin(theta)*skew(u) + (1-cos(theta))*(u*transpose(u))

